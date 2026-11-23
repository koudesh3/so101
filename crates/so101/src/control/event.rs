use crate::{ActuatorStatus, Chunk, Frame, JointState, JointTarget, Tick, Timestamp};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ArmRole {
    Leader,
    Follower,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Message {
    JointState(ArmRole, JointState),
    Actuator(ArmRole, ActuatorStatus),
    JointTarget(ArmRole, JointTarget),
    Tick(Tick),
    Frame(Frame),
    Chunk(Chunk),
    LogDrop(LogDrop),
}

impl Message {
    /// When the thing happened (contract 3).
    pub fn time(&self) -> Timestamp {
        match self {
            Message::JointState(_, m) => m.t,
            Message::Actuator(_, m) => m.t,
            Message::JointTarget(_, m) => m.t,
            Message::Tick(m) => m.t,
            Message::Frame(f) => f.t_capture,
            Message::Chunk(m) => m.t_obs,
            Message::LogDrop(m) => m.t,
        }
    }

    pub fn is_frame(&self) -> bool {
        matches!(self, Message::Frame(_))
    }
}

/// An event with when it happened, when we received it, and its producer's counter (contract 3).
#[derive(Clone, Debug, PartialEq)]
pub struct Event {
    /// The slot index for joint state (contract 5), the kernel's frame number for a camera.
    pub sequence: u32,
    pub log_time: Timestamp,
    pub publish_time: Timestamp,
    pub message: Message,
}

impl Event {
    pub fn new(message: Message, sequence: u32, log_time: Timestamp) -> Self {
        debug_assert!(!message.is_frame(), "use Event::frame: a frame carries its own sequence and receive time");
        Self { sequence, log_time, publish_time: message.time(), message }
    }

    pub fn frame(frame: Frame) -> Self {
        Self {
            sequence: frame.sequence,
            log_time: frame.t_receive,
            publish_time: frame.t_capture,
            message: Message::Frame(frame),
        }
    }
}

/// A run of consecutive messages on one topic that a consumer dropped because it fell behind.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LogDrop {
    pub t: Timestamp,
    pub topic: String,
    pub first_sequence: u32,
    pub last_sequence: u32,
    pub count: u32,
}
