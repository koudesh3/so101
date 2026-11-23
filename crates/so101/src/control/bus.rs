use std::sync::mpsc;
use std::sync::{Arc, RwLock};

use crate::Event;

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
