//! Hardware drivers. Today: the SO-101's servo stack, bottom up, each layer testable
//! without the one above:
//! - `so101::wire`    protocol frames, the STS3215 control table, the transport seam
//! - `so101::device`  one servo bus, real or simulated
//! - `so101::robot`   joints, calibration, the arm

pub mod error;
pub mod so101;

pub use error::{Error, Result};
