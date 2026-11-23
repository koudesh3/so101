//! What a policy is given (`Obs`) and what it returns (`Chunk`).

use std::time::Duration;

use crate::arm::Pose;
use crate::camera::Frame;
use crate::clock::Timestamp;

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct CheckpointId(pub String);

#[derive(Clone, Debug, PartialEq)]
pub struct Obs {
    pub t: Timestamp,
    pub pose: Pose,
    pub wrist: Option<Frame>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Chunk {
    pub t_obs: Timestamp,
    /// Between consecutive actions: the rate the policy was trained at, never assumed.
    pub period: Duration,
    pub actions: Vec<Pose>,
    pub checkpoint_id: CheckpointId,
}
