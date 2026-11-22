//! Moving bytes to and from a bus with no knowledge of what they mean:
//! the `Transport` seam, the serial port behind it, and the byte log.

use std::fs::File;
use std::io::{self, BufWriter, Read, Write};
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

pub const BAUD_RATE: u32 = 1_000_000;

#[derive(Debug)]
pub enum TransportError {
    Io(io::Error),
    Timeout { wanted: usize, got: usize },
}

impl From<io::Error> for TransportError {
    fn from(e: io::Error) -> Self {
        if e.kind() == io::ErrorKind::TimedOut {
            TransportError::Timeout { wanted: 0, got: 0 }
        } else {
            TransportError::Io(e)
        }
    }
}

impl std::fmt::Display for TransportError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TransportError::Io(e) => write!(f, "io: {e}"),
            TransportError::Timeout { wanted, got } => write!(f, "timeout after {got} of {wanted} bytes"),
        }
    }
}

impl std::error::Error for TransportError {}

pub trait Transport: Send {
    fn send(&mut self, bytes: &[u8]) -> Result<(), TransportError>;
    // Fill `buf` entirely or fail with how many bytes arrived.
    fn receive_exact(&mut self, buf: &mut [u8], timeout: Duration) -> Result<(), TransportError>;
    fn discard_input(&mut self) -> Result<(), TransportError>;
    fn name(&self) -> &str;
}

// note: the SO-101 bus board is 1 Mbaud, 8N1, no flow control.
pub struct SerialTransport {
    port: Box<dyn serialport::SerialPort>,
    path: String,
}

impl SerialTransport {
    pub fn open(path: &str) -> Result<Self, TransportError> {
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

fn other(e: serialport::Error) -> TransportError {
    TransportError::Io(io::Error::other(e.to_string()))
}

impl Transport for SerialTransport {
    fn send(&mut self, bytes: &[u8]) -> Result<(), TransportError> {
        self.port.write_all(bytes)?;
        self.port.flush()?;
        Ok(())
    }

    fn receive_exact(&mut self, buf: &mut [u8], timeout: Duration) -> Result<(), TransportError> {
        let deadline = Instant::now() + timeout;
        let mut got = 0;
        while got < buf.len() {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Err(TransportError::Timeout { wanted: buf.len(), got });
            }
            self.port.set_timeout(remaining).map_err(other)?;
            match self.port.read(&mut buf[got..]) {
                Ok(0) => return Err(TransportError::Timeout { wanted: buf.len(), got }),
                Ok(n) => got += n,
                Err(e) if e.kind() == io::ErrorKind::TimedOut => {
                    return Err(TransportError::Timeout { wanted: buf.len(), got })
                }
                Err(e) => return Err(TransportError::Io(e)),
            }
        }
        Ok(())
    }

    fn discard_input(&mut self) -> Result<(), TransportError> {
        self.port.clear(serialport::ClearBuffer::Input).map_err(other)
    }

    fn name(&self) -> &str {
        &self.path
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Direction {
    HostToBus,
    BusToHost,
}

impl Direction {
    pub fn as_str(self) -> &'static str {
        match self {
            Direction::HostToBus => "tx",
            Direction::BusToHost => "rx",
        }
    }
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "tx" => Some(Direction::HostToBus),
            "rx" => Some(Direction::BusToHost),
            _ => None,
        }
    }
}

// One line of the byte log: `{"t_us":..,"port":..,"dir":"tx"|"rx","hex":"FFFF.."}`.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize, PartialEq)]
pub struct LogRecord {
    pub t_us: u64,
    pub port: String,
    pub dir: String,
    pub hex: String,
}

impl LogRecord {
    pub fn bytes(&self) -> Vec<u8> {
        (0..self.hex.len())
            .step_by(2)
            .filter_map(|i| u8::from_str_radix(&self.hex[i..i + 2], 16).ok())
            .collect()
    }
}

// One log file shared by every transport in the process, on one clock.
pub struct ByteLog {
    writer: Mutex<BufWriter<File>>,
    epoch: Instant,
}

impl ByteLog {
    pub fn create(path: &Path) -> io::Result<Arc<Self>> {
        Ok(Arc::new(Self { writer: Mutex::new(BufWriter::new(File::create(path)?)), epoch: Instant::now() }))
    }

    pub fn record(&self, port: &str, dir: Direction, bytes: &[u8]) {
        let rec = LogRecord {
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

// A transport that logs every byte passing through it.
pub struct Recorder<T: Transport> {
    inner: T,
    label: String,
    log: Arc<ByteLog>,
}

impl<T: Transport> Recorder<T> {
    pub fn new(inner: T, label: &str, log: Arc<ByteLog>) -> Self {
        Self { inner, label: label.to_string(), log }
    }
}

impl<T: Transport> Transport for Recorder<T> {
    fn send(&mut self, bytes: &[u8]) -> Result<(), TransportError> {
        self.log.record(&self.label, Direction::HostToBus, bytes);
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
            self.log.record(&self.label, Direction::BusToHost, &buf[..got]);
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

pub fn read_log(path: &Path) -> io::Result<Vec<LogRecord>> {
    let text = std::fs::read_to_string(path)?;
    Ok(text
        .lines()
        .filter(|l| !l.trim().is_empty())
        .filter_map(|l| serde_json::from_str(l).ok())
        .collect())
}
