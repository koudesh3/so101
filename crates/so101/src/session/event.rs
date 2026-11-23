use prost::Message as _;

use super::pb::foxglove as fpb;
use super::pb::so101::v1 as pb;
use super::{Error, Result};
use crate::{ArmRole, Camera, Event, Message, Timestamp};

pub(crate) const SCHEMA_JOINT_STATE: &str = "so101.v1.JointState";
pub(crate) const SCHEMA_ACTUATOR: &str = "so101.v1.ActuatorStatus";
pub(crate) const SCHEMA_JOINT_TARGET: &str = "so101.v1.JointTarget";
pub(crate) const SCHEMA_TICK: &str = "so101.v1.Tick";
pub(crate) const SCHEMA_CHUNK: &str = "so101.v1.Chunk";
pub(crate) const SCHEMA_RECORDER_DROP: &str = "so101.v1.RecorderDrop";
pub(crate) const SCHEMA_IMAGE: &str = "foxglove.CompressedImage";

pub(crate) const TOPIC_RECORDER_DROPS: &str = "/session/drops";

impl Message {
    pub fn topic(&self) -> &'static str {
        use ArmRole::*;
        match self {
            Message::JointState(Leader, _) => "/leader/state",
            Message::JointState(Follower, _) => "/follower/state",
            Message::Actuator(Leader, _) => "/leader/actuators",
            Message::Actuator(Follower, _) => "/follower/actuators",
            Message::JointTarget(Leader, _) => "/leader/target",
            Message::JointTarget(Follower, _) => "/follower/target",
            Message::Tick(_) => "/tick",
            Message::Frame(f) => match f.camera {
                Camera::Wrist => "/cam/wrist",
                Camera::Scene => "/cam/scene",
            },
            Message::Chunk(_) => "/policy/chunk",
            Message::RecorderDrop(_) => TOPIC_RECORDER_DROPS,
        }
    }

    pub(crate) fn schema(&self) -> &'static str {
        match self {
            Message::JointState(..) => SCHEMA_JOINT_STATE,
            Message::Actuator(..) => SCHEMA_ACTUATOR,
            Message::JointTarget(..) => SCHEMA_JOINT_TARGET,
            Message::Tick(_) => SCHEMA_TICK,
            Message::Frame(_) => SCHEMA_IMAGE,
            Message::Chunk(_) => SCHEMA_CHUNK,
            Message::RecorderDrop(_) => SCHEMA_RECORDER_DROP,
        }
    }

    pub(crate) fn encode(&self) -> Vec<u8> {
        use super::wire::Wire;
        match self {
            Message::JointState(_, m) => m.to_pb().encode_to_vec(),
            Message::Actuator(_, m) => m.to_pb().encode_to_vec(),
            Message::JointTarget(_, m) => m.to_pb().encode_to_vec(),
            Message::Tick(m) => m.to_pb().encode_to_vec(),
            Message::Frame(f) => super::wire::frame_to_pb(f).encode_to_vec(),
            Message::Chunk(m) => m.to_pb().encode_to_vec(),
            Message::RecorderDrop(m) => m.to_pb().encode_to_vec(),
        }
    }

    /// A frame's sequence and receive time come from its record, not its payload.
    pub(crate) fn decode(topic: &str, data: &[u8], sequence: u32, log_time: Timestamp) -> Result<Message> {
        use super::wire::Wire;
        let role = |name: &str| match name {
            "leader" => Ok(ArmRole::Leader),
            "follower" => Ok(ArmRole::Follower),
            _ => Err(Error::Format(format!("unknown arm in topic {topic}"))),
        };
        let parts: Vec<&str> = topic.trim_start_matches('/').split('/').collect();
        Ok(match parts.as_slice() {
            [a, "state"] => Message::JointState(role(a)?, Wire::from_pb(pb::JointState::decode(data)?, "JointState")?),
            [a, "actuators"] => {
                Message::Actuator(role(a)?, Wire::from_pb(pb::ActuatorStatus::decode(data)?, "ActuatorStatus")?)
            }
            [a, "target"] => {
                Message::JointTarget(role(a)?, Wire::from_pb(pb::JointTarget::decode(data)?, "JointTarget")?)
            }
            ["tick"] => Message::Tick(Wire::from_pb(pb::Tick::decode(data)?, "Tick")?),
            ["cam", _] => {
                Message::Frame(super::wire::frame_from_pb(fpb::CompressedImage::decode(data)?, sequence, log_time)?)
            }
            ["policy", "chunk"] => Message::Chunk(Wire::from_pb(pb::Chunk::decode(data)?, "Chunk")?),
            ["session", "drops"] => {
                Message::RecorderDrop(Wire::from_pb(pb::RecorderDrop::decode(data)?, "RecorderDrop")?)
            }
            _ => return Err(Error::Format(format!("unknown topic {topic}"))),
        })
    }
}

impl Event {
    pub fn topic(&self) -> &'static str {
        self.message.topic()
    }
}
