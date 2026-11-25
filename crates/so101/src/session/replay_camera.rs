//! `ReplayCamera`: one camera's frames from a recorded session, delivered at their recorded times
//! offset onto the replay's clock (contract 3), with their original sequence numbers and JPEGs.

use std::sync::Arc;
use std::time::Duration;

use super::Session;
use crate::camera::{self, Camera, CameraRole, Description, Frame};
use crate::clock::{Clock, Timestamp};
use crate::control::Message;

pub struct ReplayCamera {
    role: CameraRole,
    clock: Arc<dyn Clock>,
    frames: std::vec::IntoIter<Frame>,
    // Added to every recorded time: the first frame's capture lands one period after `now`.
    offset_ns: i128,
    period: Duration,
    description: Description,
}

impl ReplayCamera {
    /// `role`'s frames from `session`, in capture order. None if it recorded none.
    pub fn new(session: &Session, role: CameraRole, clock: Arc<dyn Clock>) -> Option<Self> {
        let mut frames: Vec<Frame> = session
            .events
            .iter()
            .filter_map(|e| match &e.message {
                Message::Frame(f) if f.camera == role => Some(f.clone()),
                _ => None,
            })
            .collect();
        frames.sort_by_key(|f| f.t_capture);
        let first = frames.first()?.t_capture;
        let mut intervals: Vec<u64> = frames.windows(2).map(|w| w[1].t_capture.0 - w[0].t_capture.0).collect();
        intervals.sort_unstable();
        let period = Duration::from_nanos(intervals.get(intervals.len() / 2).copied().unwrap_or(33_333_333));
        let offset_ns = (clock.now() + period).0 as i128 - first.0 as i128;
        let recorded = session.header.cameras.get(role.name());
        let description = Description {
            device: format!("replay of {}", recorded.map_or("an unknown device", |c| c.device.as_str())),
            settings: recorded.map(|c| c.settings.clone()).unwrap_or_default(),
        };
        Some(Self { role, clock, frames: frames.into_iter(), offset_ns, period, description })
    }

    fn shift(&self, t: Timestamp) -> Timestamp {
        Timestamp((t.0 as i128 + self.offset_ns).max(0) as u64)
    }
}

impl Camera for ReplayCamera {
    fn role(&self) -> CameraRole {
        self.role
    }

    /// The recording's median capture interval.
    fn nominal_period(&self) -> Duration {
        self.period
    }

    fn describe(&self) -> Description {
        self.description.clone()
    }

    fn next_frame(&mut self) -> camera::Result<Frame> {
        let mut frame = self.frames.next().ok_or(camera::Error::Ended)?;
        frame.t_capture = self.shift(frame.t_capture);
        frame.t_receive = self.shift(frame.t_receive);
        self.clock.sleep_until(frame.t_receive);
        Ok(frame)
    }
}
