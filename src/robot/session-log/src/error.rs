use std::fmt;

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug)]
pub enum Error {
    Io(std::io::Error),
    Mcap(mcap::McapError),
    Decode(prost::DecodeError),
    Contract(contracts::ContractError),
    /// The file isn't a log this crate wrote (unknown topic or schema, missing metadata, ...).
    Format(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io(e) => write!(f, "{e}"),
            Error::Mcap(e) => write!(f, "mcap: {e}"),
            Error::Decode(e) => write!(f, "protobuf: {e}"),
            Error::Contract(e) => write!(f, "{e}"),
            Error::Format(s) => write!(f, "log format: {s}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Error::Io(e)
    }
}

impl From<mcap::McapError> for Error {
    fn from(e: mcap::McapError) -> Self {
        Error::Mcap(e)
    }
}

impl From<prost::DecodeError> for Error {
    fn from(e: prost::DecodeError) -> Self {
        Error::Decode(e)
    }
}

impl From<contracts::ContractError> for Error {
    fn from(e: contracts::ContractError) -> Self {
        Error::Contract(e)
    }
}
