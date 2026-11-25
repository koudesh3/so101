//! `SyntheticCamera`: frames with exact, chosen times, for tests and `--sim`. It delivers each
//! frame `latency` after its capture time, on whatever clock it's given, so on a `ReplayClock` a
//! test knows exactly which frame exists when.

use std::sync::Arc;
use std::time::Duration;

use bytes::Bytes;

use crate::camera::{Camera, CameraRole, Description, Frame, Result};
use crate::clock::{Clock, Timestamp};

// A real (tiny) JPEG, so a console shows something.
const PICTURE: &[u8] = include_bytes!("synthetic.jpg");

pub struct SyntheticCamera {
    role: CameraRole,
    clock: Arc<dyn Clock>,
    period: Duration,
    latency: Duration,
    next_capture: Timestamp,
    sequence: u32,
    lose: Vec<u32>,
    picture: Bytes,
}

impl SyntheticCamera {
    /// The icSpring's measured rate (29.8 fps) and capture-to-receive latency (about 33 ms),
    /// starting one period after the clock's present time.
    pub fn new(role: CameraRole, clock: Arc<dyn Clock>) -> Self {
        let period = Duration::from_nanos(33_557_047); // 29.8 fps
        let next_capture = clock.now() + period;
        Self {
            role,
            clock,
            period,
            latency: Duration::from_millis(33),
            next_capture,
            sequence: 0,
            lose: Vec::new(),
            picture: Bytes::from_static(PICTURE),
        }
    }

    pub fn with_period(mut self, period: Duration) -> Self {
        self.next_capture = self.clock.now() + period;
        self.period = period;
        self
    }

    pub fn with_latency(mut self, latency: Duration) -> Self {
        self.latency = latency;
        self
    }

    /// These frames are never delivered, as if lost on the way: their sequence numbers are skipped.
    pub fn losing(mut self, sequences: &[u32]) -> Self {
        self.lose.extend_from_slice(sequences);
        self
    }
}

impl Camera for SyntheticCamera {
    fn role(&self) -> CameraRole {
        self.role
    }

    fn nominal_period(&self) -> Duration {
        self.period
    }

    fn describe(&self) -> Description {
        Description {
            device: format!("synthetic-{}", self.role.name()),
            settings: format!("period_ns={} latency_ns={}", self.period.as_nanos(), self.latency.as_nanos()),
        }
    }

    fn next_frame(&mut self) -> Result<Frame> {
        loop {
            let (sequence, t_capture) = (self.sequence, self.next_capture);
            self.sequence = self.sequence.wrapping_add(1);
            self.next_capture = self.next_capture + self.period;
            if self.lose.contains(&sequence) {
                continue;
            }
            let t_receive = t_capture + self.latency;
            self.clock.sleep_until(t_receive);
            return Ok(Frame { camera: self.role, sequence, t_capture, t_receive, jpeg: self.picture.clone() });
        }
    }
}
