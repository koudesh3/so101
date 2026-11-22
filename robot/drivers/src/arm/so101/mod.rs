//! The SO-101 arm: Feetech STS3215 servos on one serial bus. Bottom up, each layer testable
//! without the one above:
//! - bytes on a serial line and the table they address, knowing nothing of an arm:
//!   `protocol` (frames), `registers` (the STS3215 control table), `transport` (the serial
//!   port, and the byte log)
//! - one servo bus: `bus`, the real one behind a transport, and `sim`, a simulated one that
//!   is a transport
//! - six joints with names, units and calibration, on one bus: `joint`, `calibration`, `arm`

pub mod arm;
pub mod bus;
pub mod calibration;
pub mod joint;
pub mod protocol;
pub mod registers;
pub mod sim;
pub mod transport;
