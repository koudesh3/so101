//! The tick: what the control loop used at one 30 Hz step (contract 2). Anything downstream
//! reads the loop's view of the world only from ticks and the messages they reference.
//!
//! `action` means the same thing in demos and in runs: the target actually sent to the follower,
//! after clipping. What the source asked for (the leader's reading in a demo, the policy's
//! output in a run) is `requested`. Training reads `action`, so it learns what was executed.

use crate::joint::{joint_from_pb, joints_from_pb};
use crate::pb::so101::v1 as pb;
use crate::{ContractError, FrameRef, Joint, Joints, Source, Timestamp};

type Result<T> = std::result::Result<T, ContractError>;

#[derive(Clone, Debug, PartialEq)]
pub struct Tick {
    pub t: Timestamp,
    pub state: Joints,
    pub wrist: Option<FrameRef>,
    pub scene: Option<FrameRef>,
    /// The target actually sent to the follower, after clipping.
    pub action: Joints,
    pub source: Source,
    /// What the source asked for, before clipping.
    pub requested: Joints,
    /// The joints where `action` differs from `requested`, and which limit did it. Countable
    /// per episode: frequent clipping in demos means the step limit is fighting the operator.
    pub clipped: Vec<Clip>,
}

/// Which limit made the sent target differ from the requested one.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Limit {
    /// The per-tick step limit (LeRobot's `max_relative_target`).
    Step,
    /// The joint's allowed range.
    Range,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Clip {
    pub joint: Joint,
    pub limit: Limit,
}

impl From<&Clip> for pb::Clip {
    fn from(c: &Clip) -> Self {
        let limit = match c.limit {
            Limit::Step => pb::Limit::Step,
            Limit::Range => pb::Limit::Range,
        };
        pb::Clip { joint: pb::Joint::from(c.joint) as i32, limit: limit as i32 }
    }
}

impl TryFrom<pb::Clip> for Clip {
    type Error = ContractError;
    fn try_from(c: pb::Clip) -> Result<Self> {
        let limit = match pb::Limit::try_from(c.limit) {
            Ok(pb::Limit::Step) => Limit::Step,
            Ok(pb::Limit::Range) => Limit::Range,
            _ => return Err(ContractError::new(format!("unknown limit {}", c.limit))),
        };
        Ok(Clip { joint: joint_from_pb(c.joint)?, limit })
    }
}

impl From<&Tick> for pb::Tick {
    fn from(m: &Tick) -> Self {
        pb::Tick {
            t_ns: m.t.0,
            state: m.state.0.to_vec(),
            wrist: m.wrist.map(Into::into),
            scene: m.scene.map(Into::into),
            action: m.action.0.to_vec(),
            source: Some((&m.source).into()),
            requested: m.requested.0.to_vec(),
            clipped: m.clipped.iter().map(Into::into).collect(),
        }
    }
}

impl TryFrom<pb::Tick> for Tick {
    type Error = ContractError;
    fn try_from(m: pb::Tick) -> Result<Self> {
        Ok(Tick {
            t: Timestamp(m.t_ns),
            state: joints_from_pb(&m.state, "Tick.state")?,
            wrist: m.wrist.map(TryInto::try_into).transpose()?,
            scene: m.scene.map(TryInto::try_into).transpose()?,
            action: joints_from_pb(&m.action, "Tick.action")?,
            source: m.source.try_into()?,
            requested: joints_from_pb(&m.requested, "Tick.requested")?,
            clipped: m.clipped.into_iter().map(TryInto::try_into).collect::<Result<_>>()?,
        })
    }
}
