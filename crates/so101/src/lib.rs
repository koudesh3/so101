//! The SO-101, one folder per thing: the arm, the camera, the policy, the control loop, and the
//! recorded session, plus the clock they all share. `cli.rs` puts them together.

pub mod arm;
pub mod camera;
pub mod clock;
pub mod control;
pub mod policy;
pub mod session;
