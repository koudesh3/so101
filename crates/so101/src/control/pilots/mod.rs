//! The ways the follower can be driven: each `Pilot` turns something (the leader arm, a policy's
//! action chunks, a script) into a target every tick. Teleop so far; the action queue and scripts
//! join it in Phase 1.

mod teleop;

pub use teleop::Teleop;

use crate::arm::{JointState, Pose, Result};
use crate::camera::Seen;
use crate::clock::Timestamp;
use crate::control::{Message, Source};

/// What a pilot has to decide on, at a tick: the follower's latest state and each camera's newest
/// frame then. The tick records exactly these, so a recording shows what every decision saw.
#[derive(Clone, Copy, Debug)]
pub struct Observation<'a> {
    pub t: Timestamp,
    pub follower: &'a JointState,
    pub wrist: Option<Seen>,
    pub scene: Option<Seen>,
}

/// What decides the follower's target each tick (30 Hz).
pub trait Pilot: Send {
    /// Recorded with every target it produces.
    fn source(&self) -> Source;
    /// The target for this tick, and anything the pilot observed to decide it (the leader's
    /// state, say), to be recorded.
    fn target(&mut self, seen: &Observation) -> Result<(Pose, Vec<Message>)>;
}
