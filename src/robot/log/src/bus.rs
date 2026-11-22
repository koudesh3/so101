//! In-process fan-out of envelopes. Publishing never blocks: every sink must accept (or drop,
//! and say so) without waiting.

use std::sync::mpsc;
use std::sync::{Arc, RwLock};

use crate::Envelope;

/// Something on the bus. `accept` must return without blocking (the control loop publishes).
pub trait Sink: Send + Sync {
    fn accept(&self, envelope: &Envelope);
}

#[derive(Default, Clone)]
pub struct Bus {
    sinks: Arc<RwLock<Vec<Arc<dyn Sink>>>>,
}

impl Bus {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn attach(&self, sink: Arc<dyn Sink>) {
        self.sinks.write().expect("bus lock").push(sink);
    }

    /// An unbounded in-process subscription, for consumers that keep up (checks, the dashboard).
    /// The recorder is a bounded sink instead (see `Recorder`).
    pub fn subscribe(&self) -> mpsc::Receiver<Envelope> {
        let (tx, rx) = mpsc::channel();
        self.attach(Arc::new(ChannelSink(tx)));
        rx
    }

    pub fn publish(&self, envelope: Envelope) {
        for sink in self.sinks.read().expect("bus lock").iter() {
            sink.accept(&envelope);
        }
    }
}

struct ChannelSink(mpsc::Sender<Envelope>);

impl Sink for ChannelSink {
    fn accept(&self, envelope: &Envelope) {
        // A dropped receiver just means nobody is listening any more.
        let _ = self.0.send(envelope.clone());
    }
}
