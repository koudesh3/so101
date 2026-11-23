//! The UART to the servos: moving bytes to and from the line with no knowledge of what they mean.

use std::fs::File;
use std::io::{self, BufWriter, Read, Write};
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::arm::{Error, Result};

pub const BAUD_RATE: u32 = 1_000_000;

// note: the SO-101 bus board is 1 Mbaud, 8N1, no flow control.
pub struct SerialPort {
    port: Box<dyn serialport::SerialPort>,
    path: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Direction {
    HostToServos,
    BusToHost,
}

// One line of the byte trace: `{"t_us":..,"port":..,"dir":"tx"|"rx","hex":"FFFF.."}`.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize, PartialEq)]
pub struct TraceRecord {
    pub t_us: u64,
    pub port: String,
    pub dir: String,
    pub hex: String,
}

// One trace file shared by every transport in the process, on one clock.
pub struct ByteTrace {
    writer: Mutex<BufWriter<File>>,
    epoch: Instant,
}

pub struct Traced<T: Transport> {
    inner: T,
    label: String,
    trace: Arc<ByteTrace>,
}

pub trait Transport: Send {
    fn send(&mut self, bytes: &[u8]) -> Result<()>;
    // Fill `buf` entirely or fail with how many bytes arrived.
    fn receive_exact(&mut self, buf: &mut [u8], timeout: Duration) -> Result<()>;
    fn discard_input(&mut self) -> Result<()>;
    fn name(&self) -> &str;
}

impl SerialPort {
    pub fn open(path: &str) -> Result<Self> {
        let port = serialport::new(path, BAUD_RATE)
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .flow_control(serialport::FlowControl::None)
            .timeout(Duration::from_millis(20))
            .open()
            .map_err(other)?;
        Ok(Self { port, path: path.to_string() })
    }
}

impl Direction {
    pub fn as_str(self) -> &'static str {
        match self {
            Direction::HostToServos => "tx",
            Direction::BusToHost => "rx",
        }
    }
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "tx" => Some(Direction::HostToServos),
            "rx" => Some(Direction::BusToHost),
            _ => None,
        }
    }
}

impl TraceRecord {
    pub fn bytes(&self) -> Vec<u8> {
        (0..self.hex.len()).step_by(2).filter_map(|i| u8::from_str_radix(&self.hex[i..i + 2], 16).ok()).collect()
    }
}

impl ByteTrace {
    pub fn create(path: &Path) -> io::Result<Arc<Self>> {
        Ok(Arc::new(Self { writer: Mutex::new(BufWriter::new(File::create(path)?)), epoch: Instant::now() }))
    }

    pub fn record(&self, port: &str, dir: Direction, bytes: &[u8]) {
        let rec = TraceRecord {
            t_us: self.epoch.elapsed().as_micros() as u64,
            port: port.to_string(),
            dir: dir.as_str().to_string(),
            hex: bytes.iter().map(|b| format!("{b:02X}")).collect(),
        };
        if let Ok(mut w) = self.writer.lock() {
            let _ = serde_json::to_writer(&mut *w, &rec);
            let _ = w.write_all(b"\n");
        }
    }

    pub fn flush(&self) {
        if let Ok(mut w) = self.writer.lock() {
            let _ = w.flush();
        }
    }
}

impl<T: Transport> Traced<T> {
    pub fn new(inner: T, label: &str, trace: Arc<ByteTrace>) -> Self {
        Self { inner, label: label.to_string(), trace }
    }
}

impl Transport for SerialPort {
    fn send(&mut self, bytes: &[u8]) -> Result<()> {
        self.port.write_all(bytes)?;
        self.port.flush()?;
        Ok(())
    }

    fn receive_exact(&mut self, buf: &mut [u8], timeout: Duration) -> Result<()> {
        let deadline = Instant::now() + timeout;
        let mut got = 0;
        while got < buf.len() {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Err(Error::Timeout { wanted: buf.len(), got });
            }
            self.port.set_timeout(remaining).map_err(other)?;
            match self.port.read(&mut buf[got..]) {
                Ok(0) => return Err(Error::Timeout { wanted: buf.len(), got }),
                Ok(n) => got += n,
                Err(e) if e.kind() == io::ErrorKind::TimedOut => return Err(Error::Timeout { wanted: buf.len(), got }),
                Err(e) => return Err(Error::Io(e)),
            }
        }
        Ok(())
    }

    fn discard_input(&mut self) -> Result<()> {
        self.port.clear(serialport::ClearBuffer::Input).map_err(other)
    }

    fn name(&self) -> &str {
        &self.path
    }
}

impl<T: Transport> Transport for Traced<T> {
    fn send(&mut self, bytes: &[u8]) -> Result<()> {
        self.trace.record(&self.label, Direction::HostToServos, bytes);
        self.inner.send(bytes)
    }

    fn receive_exact(&mut self, buf: &mut [u8], timeout: Duration) -> Result<()> {
        let result = self.inner.receive_exact(buf, timeout);
        let got = match &result {
            Ok(()) => buf.len(),
            Err(Error::Timeout { got, .. }) => *got,
            Err(_) => 0,
        };
        if got > 0 {
            self.trace.record(&self.label, Direction::BusToHost, &buf[..got]);
        }
        result
    }

    fn discard_input(&mut self) -> Result<()> {
        self.inner.discard_input()
    }

    fn name(&self) -> &str {
        self.inner.name()
    }
}

fn other(e: serialport::Error) -> Error {
    Error::Io(io::Error::other(e.to_string()))
}

pub fn read_trace(path: &Path) -> io::Result<Vec<TraceRecord>> {
    let text = std::fs::read_to_string(path)?;
    Ok(text.lines().filter(|l| !l.trim().is_empty()).filter_map(|l| serde_json::from_str(l).ok()).collect())
}
