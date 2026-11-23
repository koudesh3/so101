//! The `Tick`: what the control loop saw and sent at one step, and why the two differ (contract 2).

use crate::{FrameRef, Joint, Pose, Source, Timestamp};

/// What the control loop used at one step: downstream reads the loop's view only from here (contract 2).
#[derive(Clone, Debug, PartialEq)]
pub struct Tick {
    pub t: Timestamp,
    /// The servo slot whose `JointState` this tick used and in which its target was written, so
    /// `state` and `action` can be recomputed from raw counts under another calibration.
    pub slot: u32,
    pub state: Pose,
    pub wrist: Option<FrameRef>,
    pub scene: Option<FrameRef>,
    /// Sent to the follower, after clipping. Training reads this.
    pub action: Pose,
    pub source: Source,
    /// What the source asked for, before clipping.
    pub requested: Pose,
    pub clipped: Vec<Clip>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Limit {
    /// LeRobot's `max_relative_target`.
    Step,
    Range,
    /// Held at a contact point after a stall or an overload trip.
    Stall,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Clip {
    pub joint: Joint,
    pub limit: Limit,
}
