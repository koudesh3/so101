//! so101: a from-scratch driver for the LeRobot SO-101 arm pair.
//!
//! Bottom up, each layer testable without the one above:
//! - `wire`     protocol frames, the control table, the transport seam
//! - `device`   one servo bus, real or simulated
//! - `robot`    joints, calibration, the arm
//! - `control`  ticker, calibration sweep, teleop loop
//! - `report`   byte log -> HTML

pub mod control;
pub mod device;
pub mod error;
pub mod report;
pub mod robot;
pub mod wire;

pub use error::{Error, Result};
