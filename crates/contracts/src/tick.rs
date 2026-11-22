//! The tick: what the control loop used at one 30 Hz step (contract 2). Anything downstream
//! reads the loop's view of the world only from ticks and the messages they reference.

use crate::joint::joints_from_pb;
use crate::pb::so101::v1 as pb;
use crate::{ContractError, FrameRef, Joints, Source, Timestamp};

type Result<T> = std::result::Result<T, ContractError>;

#[derive(Clone, Debug, PartialEq)]
pub struct Tick {
    pub t: Timestamp,
    pub state: Joints,
    pub wrist: Option<FrameRef>,
    pub scene: Option<FrameRef>,
    pub action: Joints,
    pub source: Source,
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
        })
    }
}
