//! A host-side driver for Feetech STS3215 servos on one half-duplex serial bus. Each layer is
//! testable without the one above it.

pub mod bus;
pub mod error;
pub mod protocol;
pub mod registers;
pub mod sim;
pub mod transport;

pub use error::{Error, Result};
