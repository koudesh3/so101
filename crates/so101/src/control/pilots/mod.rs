//! The ways the follower can be driven: each `Pilot` turns something (the leader arm, a policy's
//! action chunks, a script) into a target every tick. Teleop so far; the action queue and scripts
//! join it in Phase 1.

mod teleop;

pub use teleop::{FineControl, Teleop};

use crate::arm::{JointState, Pose, Result};
use crate::clock::Timestamp;
use crate::control::{Message, Source};

/// What decides the follower's target each tick.
pub trait Pilot: Send {
    /// Recorded with every target it produces.
    fn source(&self) -> Source;
    /// The target for this tick, given the follower's latest state, and anything the pilot
    /// observed to decide it (the leader's state, say), to be recorded.
    fn target(&mut self, now: Timestamp, follower: &JointState) -> Result<(Pose, Vec<Message>)>;
}
