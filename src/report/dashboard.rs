//! Live dashboard: a tiny HTTP server (std only) that serves one page and
//! streams every tick and every wire frame to it as server-sent events.
//!
//! GET /        the page (compiled into the binary)
//! GET /events  text/event-stream, one JSON object per event
//!
//! Event kinds (field `k`):
//! - "hello"  sent first on every connection: calibrations, rate, limits
//! - "t"      one teleop tick: joints, goals, timing, optional telemetry
//! - "w"      one transport call: port, direction, raw bytes, time
//! - "end"    the loop stopped, with its stats

use std::io::{self, BufRead, BufReader, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::wire::transport::{Transport, TransportError};

const PAGE: &str = include_str!("dashboard.html");
/// SO-101 meshes and kinematic tree, packed by tools/build_model.py from
/// TheRobotStudio's SO-ARM100 simulation assets (Apache-2.0).
const MODEL_JSON: &[u8] = include_bytes!("assets/model.json");
const MODEL_BIN: &[u8] = include_bytes!("assets/model.bin");

/// Fans events out to every connected page.
pub struct Hub {
    clients: Mutex<Vec<Sender<Arc<str>>>>,
    hello: Mutex<Option<Arc<str>>>,
    epoch: Instant,
}

impl Hub {
    pub fn new() -> Arc<Self> {
        Arc::new(Self { clients: Mutex::new(Vec::new()), hello: Mutex::new(None), epoch: Instant::now() })
    }

    /// Microseconds since the hub started: the one clock every event uses.
    pub fn t_us(&self) -> u64 {
        self.epoch.elapsed().as_micros() as u64
    }

    /// The event every new page receives first.
    pub fn set_hello(&self, json: String) {
        let json: Arc<str> = json.into();
        *self.hello.lock().unwrap() = Some(json.clone());
        self.publish_arc(json);
    }

    pub fn publish(&self, json: String) {
        self.publish_arc(json.into());
    }

    fn publish_arc(&self, json: Arc<str>) {
        let mut clients = self.clients.lock().unwrap();
        clients.retain(|tx| tx.send(json.clone()).is_ok());
    }

    fn subscribe(&self) -> Receiver<Arc<str>> {
        let (tx, rx) = mpsc::channel();
        if let Some(hello) = self.hello.lock().unwrap().clone() {
            let _ = tx.send(hello);
        }
        self.clients.lock().unwrap().push(tx);
        rx
    }
}

/// Start serving in the background. Returns the bound address.
pub fn serve(hub: Arc<Hub>, addr: &str) -> io::Result<SocketAddr> {
    let listener = TcpListener::bind(addr)?;
    let local = listener.local_addr()?;
    std::thread::spawn(move || {
        for stream in listener.incoming().flatten() {
            let hub = hub.clone();
            std::thread::spawn(move || {
                let _ = handle(stream, &hub);
            });
        }
    });
    Ok(local)
}

fn handle(stream: TcpStream, hub: &Hub) -> io::Result<()> {
    stream.set_nodelay(true)?;
    let mut reader = BufReader::new(stream.try_clone()?);
    let mut request_line = String::new();
    reader.read_line(&mut request_line)?;
    // Drain the headers; we do not need any of them.
    let mut line = String::new();
    while reader.read_line(&mut line)? > 2 {
        line.clear();
    }
    let target = request_line.split_whitespace().nth(1).unwrap_or("/");
    let path = target.split('?').next().unwrap_or("/");
    let mut out = stream;
    match path {
        "/" | "/index.html" => {
            // SO101_DASHBOARD=path/to/dashboard.html serves the file from disk,
            // so the page can be edited without rebuilding.
            let page = std::env::var("SO101_DASHBOARD").ok().and_then(|p| std::fs::read_to_string(p).ok());
            let page = page.as_deref().unwrap_or(PAGE);
            write!(
                out,
                "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: {}\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n",
                page.len()
            )?;
            out.write_all(page.as_bytes())?;
            out.flush()
        }
        "/model.json" => send(&mut out, "application/json", MODEL_JSON),
        "/model.bin" => send(&mut out, "application/octet-stream", MODEL_BIN),
        "/events" => stream_events(out, hub.subscribe()),
        _ => {
            out.write_all(b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n")?;
            out.flush()
        }
    }
}

fn send(out: &mut TcpStream, content_type: &str, body: &[u8]) -> io::Result<()> {
    write!(
        out,
        "HTTP/1.1 200 OK\r\nContent-Type: {content_type}\r\nContent-Length: {}\r\nCache-Control: max-age=3600\r\nConnection: close\r\n\r\n",
        body.len()
    )?;
    out.write_all(body)?;
    out.flush()
}

fn stream_events(mut out: TcpStream, rx: Receiver<Arc<str>>) -> io::Result<()> {
    out.write_all(
        b"HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nCache-Control: no-store\r\nConnection: keep-alive\r\n\r\n",
    )?;
    out.flush()?;
    let mut batch = String::with_capacity(16 * 1024);
    loop {
        match rx.recv_timeout(Duration::from_secs(10)) {
            Ok(first) => {
                batch.clear();
                push_event(&mut batch, &first);
                // Coalesce whatever else is already queued into one write.
                while let Ok(next) = rx.try_recv() {
                    push_event(&mut batch, &next);
                    if batch.len() > 256 * 1024 {
                        break;
                    }
                }
                out.write_all(batch.as_bytes())?;
                out.flush()?;
            }
            Err(RecvTimeoutError::Timeout) => {
                out.write_all(b": keepalive\n\n")?;
                out.flush()?;
            }
            Err(RecvTimeoutError::Disconnected) => return Ok(()),
        }
    }
}

fn push_event(batch: &mut String, json: &str) {
    batch.push_str("data: ");
    batch.push_str(json);
    batch.push_str("\n\n");
}

/// Wraps a transport and publishes every byte it moves, like `Recorder`
/// does for the log file. Port 0 is the leader, 1 the follower.
pub struct Tap {
    inner: Box<dyn Transport>,
    port: u8,
    hub: Arc<Hub>,
}

impl Tap {
    pub fn new(inner: Box<dyn Transport>, label: &str, hub: Arc<Hub>) -> Self {
        Self { inner, port: if label == "follower" { 1 } else { 0 }, hub }
    }

    fn publish(&self, dir: u8, bytes: &[u8]) {
        let hex: String = bytes.iter().map(|b| format!("{b:02X}")).collect();
        self.hub.publish(format!(r#"{{"k":"w","t":{},"p":{},"d":{},"h":"{}"}}"#, self.hub.t_us(), self.port, dir, hex));
    }
}

impl Transport for Tap {
    fn send(&mut self, bytes: &[u8]) -> Result<(), TransportError> {
        self.publish(0, bytes);
        self.inner.send(bytes)
    }

    fn receive_exact(&mut self, buf: &mut [u8], timeout: Duration) -> Result<(), TransportError> {
        let result = self.inner.receive_exact(buf, timeout);
        let got = match &result {
            Ok(()) => buf.len(),
            Err(TransportError::Timeout { got, .. }) => *got,
            Err(_) => 0,
        };
        if got > 0 {
            self.publish(1, &buf[..got]);
        }
        result
    }

    fn discard_input(&mut self) -> Result<(), TransportError> {
        self.inner.discard_input()
    }

    fn name(&self) -> &str {
        self.inner.name()
    }
}
