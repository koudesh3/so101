use std::sync::mpsc;
use std::sync::{Arc, RwLock};

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
    RecorderDrop(RecorderDrop),
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
            Message::RecorderDrop(m) => m.t,
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
pub struct RecorderDrop {
    pub t: Timestamp,
    pub topic: String,
    pub first_sequence: u32,
    pub last_sequence: u32,
    pub count: u32,
}

/// `accept` must return without blocking, dropping (and saying so) rather than waiting: the
/// control loop publishes.
pub trait Sink: Send + Sync {
    fn accept(&self, event: &Event);
}

#[derive(Default, Clone)]
pub struct EventBus {
    sinks: Arc<RwLock<Vec<Arc<dyn Sink>>>>,
}

impl EventBus {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn attach(&self, sink: Arc<dyn Sink>) {
        self.sinks.write().expect("bus lock").push(sink);
    }

    /// An unbounded in-process subscription, for consumers that keep up (checks, the dashboard).
    /// The recorder is a bounded sink instead (see `Recorder`).
    pub fn subscribe(&self) -> mpsc::Receiver<Event> {
        let (tx, rx) = mpsc::channel();
        self.attach(Arc::new(ChannelSink(tx)));
        rx
    }

    pub fn publish(&self, event: Event) {
        for sink in self.sinks.read().expect("bus lock").iter() {
            sink.accept(&event);
        }
    }
}

struct ChannelSink(mpsc::Sender<Event>);

impl Sink for ChannelSink {
    fn accept(&self, event: &Event) {
        // A dropped receiver just means nobody is listening any more.
        let _ = self.0.send(event.clone());
    }
}
