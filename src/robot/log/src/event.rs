//! What travels on the bus and lands in the log: an [`Event`] (one domain message and where it
//! came from) wrapped in an [`Envelope`] (its record times and sequence number).

use contracts::pb::foxglove as fpb;
use contracts::pb::so101::v1 as pb;
use contracts::{Camera, Chunk, Frame, JointState, JointTarget, LogDrop, ServoTelemetry, Tick, Timestamp};
use prost::Message;

use crate::{Error, Result};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Arm {
    Leader,
    Follower,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Event {
    JointState(Arm, JointState),
    ServoTelemetry(Arm, ServoTelemetry),
    JointTarget(Arm, JointTarget),
    Tick(Tick),
    Frame(Frame),
    Chunk(Chunk),
    LogDrop(LogDrop),
}

pub(crate) const SCHEMA_JOINT_STATE: &str = "so101.v1.JointState";
pub(crate) const SCHEMA_SERVO_TELEMETRY: &str = "so101.v1.ServoTelemetry";
pub(crate) const SCHEMA_JOINT_TARGET: &str = "so101.v1.JointTarget";
pub(crate) const SCHEMA_TICK: &str = "so101.v1.Tick";
pub(crate) const SCHEMA_CHUNK: &str = "so101.v1.Chunk";
pub(crate) const SCHEMA_LOG_DROP: &str = "so101.v1.LogDrop";
pub(crate) const SCHEMA_IMAGE: &str = "foxglove.CompressedImage";

pub(crate) const TOPIC_LOG_DROPS: &str = "/log/drops";

impl Event {
    /// The MCAP topic this event is logged on.
    pub fn topic(&self) -> &'static str {
        match self {
            Event::JointState(Arm::Leader, _) => "/leader/state",
            Event::JointState(Arm::Follower, _) => "/follower/state",
            Event::ServoTelemetry(Arm::Leader, _) => "/leader/telemetry",
            Event::ServoTelemetry(Arm::Follower, _) => "/follower/telemetry",
            Event::JointTarget(Arm::Leader, _) => "/leader/target",
            Event::JointTarget(Arm::Follower, _) => "/follower/target",
            Event::Tick(_) => "/tick",
            Event::Frame(f) => match f.camera {
                Camera::Wrist => "/cam/wrist",
                Camera::Scene => "/cam/scene",
            },
            Event::Chunk(_) => "/policy/chunk",
            Event::LogDrop(_) => TOPIC_LOG_DROPS,
        }
    }

    /// When the thing happened: the record's `publish_time` (contract 3).
    pub fn time(&self) -> Timestamp {
        match self {
            Event::JointState(_, m) => m.t,
            Event::ServoTelemetry(_, m) => m.t,
            Event::JointTarget(_, m) => m.t,
            Event::Tick(m) => m.t,
            Event::Frame(f) => f.t_capture,
            Event::Chunk(m) => m.t_obs,
            Event::LogDrop(m) => m.t,
        }
    }

    pub fn is_frame(&self) -> bool {
        matches!(self, Event::Frame(_))
    }

    pub(crate) fn schema(&self) -> &'static str {
        match self {
            Event::JointState(..) => SCHEMA_JOINT_STATE,
            Event::ServoTelemetry(..) => SCHEMA_SERVO_TELEMETRY,
            Event::JointTarget(..) => SCHEMA_JOINT_TARGET,
            Event::Tick(_) => SCHEMA_TICK,
            Event::Frame(_) => SCHEMA_IMAGE,
            Event::Chunk(_) => SCHEMA_CHUNK,
            Event::LogDrop(_) => SCHEMA_LOG_DROP,
        }
    }

    pub(crate) fn encode(&self) -> Vec<u8> {
        match self {
            Event::JointState(_, m) => pb::JointState::from(m).encode_to_vec(),
            Event::ServoTelemetry(_, m) => pb::ServoTelemetry::from(m).encode_to_vec(),
            Event::JointTarget(_, m) => pb::JointTarget::from(m).encode_to_vec(),
            Event::Tick(m) => pb::Tick::from(m).encode_to_vec(),
            Event::Frame(f) => fpb::CompressedImage::from(f).encode_to_vec(),
            Event::Chunk(m) => pb::Chunk::from(m).encode_to_vec(),
            Event::LogDrop(m) => pb::LogDrop::from(m).encode_to_vec(),
        }
    }

    /// Rebuild an event from its record. Frames take their sequence and receive time from the
    /// record, the rest of each message from its payload.
    pub(crate) fn decode(topic: &str, data: &[u8], sequence: u32, log_time: Timestamp) -> Result<Event> {
        let arm = |name: &str| match name {
            "leader" => Ok(Arm::Leader),
            "follower" => Ok(Arm::Follower),
            _ => Err(Error::Format(format!("unknown arm in topic {topic}"))),
        };
        let parts: Vec<&str> = topic.trim_start_matches('/').split('/').collect();
        Ok(match parts.as_slice() {
            [a, "state"] => Event::JointState(arm(a)?, pb::JointState::decode(data)?.try_into()?),
            [a, "telemetry"] => Event::ServoTelemetry(arm(a)?, pb::ServoTelemetry::decode(data)?.try_into()?),
            [a, "target"] => Event::JointTarget(arm(a)?, pb::JointTarget::decode(data)?.try_into()?),
            ["tick"] => Event::Tick(pb::Tick::decode(data)?.try_into()?),
            ["cam", _] => Event::Frame(Frame::from_pb(fpb::CompressedImage::decode(data)?, sequence, log_time)?),
            ["policy", "chunk"] => Event::Chunk(pb::Chunk::decode(data)?.try_into()?),
            ["log", "drops"] => Event::LogDrop(pb::LogDrop::decode(data)?.into()),
            _ => return Err(Error::Format(format!("unknown topic {topic}"))),
        })
    }
}

/// An event with its record header: what the bus carries and the log stores.
#[derive(Clone, Debug, PartialEq)]
pub struct Envelope {
    /// The producer's counter: the slot index for joint state (contract 5), the kernel's frame
    /// number for a camera.
    pub sequence: u32,
    /// When our process received it.
    pub log_time: Timestamp,
    /// When the thing happened (also inside the message).
    pub publish_time: Timestamp,
    pub event: Event,
}

impl Envelope {
    /// Wrap a non-frame event. `publish_time` is the event's own time.
    pub fn new(event: Event, sequence: u32, log_time: Timestamp) -> Self {
        debug_assert!(!event.is_frame(), "use Envelope::frame: a frame carries its own sequence and receive time");
        Self { sequence, log_time, publish_time: event.time(), event }
    }

    /// Wrap a camera frame: sequence, receive time and capture time all come from the frame.
    pub fn frame(frame: Frame) -> Self {
        Self {
            sequence: frame.sequence,
            log_time: frame.t_receive,
            publish_time: frame.t_capture,
            event: Event::Frame(frame),
        }
    }

    pub fn topic(&self) -> &'static str {
        self.event.topic()
    }
}
