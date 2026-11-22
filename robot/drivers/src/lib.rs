//! Hardware drivers. Today: the SO-101 arm (`arm::so101`).

pub mod arm;
pub mod error;

pub use error::{Error, Result};
