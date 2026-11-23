use std::fmt;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WireError(pub String);

impl WireError {
    pub fn new(msg: impl Into<String>) -> Self {
        Self(msg.into())
    }
}

impl fmt::Display for WireError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "invalid message: {}", self.0)
    }
}

impl std::error::Error for WireError {}
