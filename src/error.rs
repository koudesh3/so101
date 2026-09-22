//! The one error type everything above the transport returns.

use std::fmt;
use std::io;

use crate::wire::protocol::ServoError;
use crate::wire::transport::TransportError;

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug)]
pub enum Error {
    Transport(TransportError),
    // Nothing came back from this servo before the deadline.
    NoReply { id: u8 },
    // Bytes arrived but were not a valid frame.
    Corrupt(String),
    // A well-formed reply from a servo we did not address.
    WrongId { expected: u8, got: u8 },
    // The servo answered and is reporting a fault.
    ServoFault { id: u8, error: ServoError },
    // The servo answered but is not the model this driver is written for.
    UnexpectedModel { id: u8, model: i32 },
    // A write refused before touching the wire.
    Refused(String),
    Calibration(String),
    Io(io::Error),
    Other(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Transport(e) => write!(f, "{e}"),
            Error::NoReply { id } => write!(f, "no reply from servo {id}"),
            Error::Corrupt(s) => write!(f, "corrupt reply: {s}"),
            Error::WrongId { expected, got } => write!(f, "reply from servo {got}, expected {expected}"),
            Error::ServoFault { id, error } => write!(f, "servo {id} fault: {error}"),
            Error::UnexpectedModel { id, model } => write!(f, "servo {id} reports model {model}, not an STS3215"),
            Error::Refused(s) => write!(f, "refused: {s}"),
            Error::Calibration(s) => write!(f, "calibration: {s}"),
            Error::Io(e) => write!(f, "{e}"),
            Error::Other(s) => write!(f, "{s}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<TransportError> for Error {
    fn from(e: TransportError) -> Self {
        Error::Transport(e)
    }
}

impl From<io::Error> for Error {
    fn from(e: io::Error) -> Self {
        Error::Io(e)
    }
}

impl From<String> for Error {
    fn from(s: String) -> Self {
        Error::Other(s)
    }
}

impl From<&str> for Error {
    fn from(s: &str) -> Self {
        Error::Other(s.to_string())
    }
}
