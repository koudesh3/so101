use std::fmt;

/// A message that doesn't satisfy its contract: a wrong array length, a missing required field,
/// an unknown enum value.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ContractError(pub String);

impl ContractError {
    pub fn new(msg: impl Into<String>) -> Self {
        Self(msg.into())
    }
}

impl fmt::Display for ContractError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "contract violation: {}", self.0)
    }
}

impl std::error::Error for ContractError {}
