//! `SimulatedCamera`: frames timed the way our cameras time them (`CameraProfile`, measured), on
//! whatever clock it's given, so on a `ReplayClock` a test knows exactly which frame exists when.
//! The same seed gives the same intervals and losses every run.

use std::sync::Arc;
use std::time::Duration;

use bytes::Bytes;

use crate::camera::{Camera, CameraRole, Description, Frame, Result};
use crate::clock::{Clock, Timestamp};
use crate::rng::Rng;

// A real (tiny) JPEG, so a console shows something.
const PICTURE: &[u8] = include_bytes!("simulated.jpg");

/// How a camera times its frames. Measured 2026-11-24 through V4L2, two minutes of each camera
/// (/projects/spikes/cam-timing-20261124 on the Spark): intervals fall in two clusters, about 32
/// and 36 ms, and a long one is never followed by another; a frame arrives when the next one
/// starts, so capture-to-receive is about the next interval.
#[derive(Clone, Copy, Debug)]
pub struct CameraProfile {
    pub short: Duration,
    pub long: Duration,
    /// Spread of each interval about its cluster (one standard deviation).
    pub jitter: Duration,
    /// Chance a short interval is followed by a long one (a long is always followed by a short).
    pub long_after_short: f64,
    /// Capture to receive, beyond the next interval.
    pub delivery: Duration,
    /// On average one frame in this many never arrives; 0 never loses one.
    pub lose_one_in: u32,
}

pub struct SimulatedCamera {
    role: CameraRole,
    clock: Arc<dyn Clock>,
    profile: CameraProfile,
    rng: Rng,
    // The capture time of the next frame, and the interval after it.
    next_capture: Timestamp,
    interval: Duration,
    last_was_long: bool,
    sequence: u32,
    lose: Vec<u32>,
    picture: Bytes,
}

impl CameraProfile {
    /// The icSpring wrist camera, 640x480 MJPEG at 12 ms exposure: 29.80 fps, 61% short intervals,
    /// 1 frame lost in 3,605.
    pub const WRIST: CameraProfile = CameraProfile {
        short: Duration::from_micros(32_010),
        long: Duration::from_micros(35_990),
        jitter: Duration::from_micros(270),
        long_after_short: 0.64,
        delivery: Duration::from_micros(340),
        lose_one_in: 3600,
    };

    /// The Innomaker scene camera, 1280x720 MJPEG at 30 ms exposure: 29.51 fps, no loss in 3,571.
    pub const SCENE: CameraProfile = CameraProfile {
        short: Duration::from_micros(31_880),
        long: Duration::from_micros(36_140),
        jitter: Duration::from_micros(310),
        long_after_short: 0.90,
        delivery: Duration::from_micros(530),
        lose_one_in: 0,
    };

    /// Exact frames: every interval `period`, each received when the next is captured plus
    /// `delivery`, as the real cameras deliver; none lost.
    pub fn steady(period: Duration, delivery: Duration) -> Self {
        Self { short: period, long: period, jitter: Duration::ZERO, long_after_short: 0.0, delivery, lose_one_in: 0 }
    }

    pub fn of(role: CameraRole) -> Self {
        match role {
            CameraRole::Wrist => Self::WRIST,
            CameraRole::Scene => Self::SCENE,
        }
    }
}

impl SimulatedCamera {
    /// `role`'s measured profile, first frame captured about one interval after now.
    pub fn new(role: CameraRole, clock: Arc<dyn Clock>) -> Self {
        let mut camera = Self {
            role,
            clock,
            profile: CameraProfile::of(role),
            rng: Rng::new(0x5EED ^ role as u64),
            next_capture: Timestamp(0),
            interval: Duration::ZERO,
            last_was_long: false,
            sequence: 0,
            lose: Vec::new(),
            picture: Bytes::from_static(PICTURE),
        };
        camera.restart();
        camera
    }

    pub fn with_profile(mut self, profile: CameraProfile) -> Self {
        self.profile = profile;
        self.restart();
        self
    }

    /// A fault for tests: these frames never arrive, as if lost on the way; their sequence
    /// numbers are skipped.
    pub fn losing(mut self, sequences: &[u32]) -> Self {
        self.lose.extend_from_slice(sequences);
        self
    }

    fn restart(&mut self) {
        let first = self.draw_interval();
        self.next_capture = self.clock.now() + first;
        self.interval = self.draw_interval();
    }

    fn draw_interval(&mut self) -> Duration {
        let p = &self.profile;
        let long = !self.last_was_long && self.rng.chance(p.long_after_short);
        self.last_was_long = long;
        let base = if long { p.long } else { p.short };
        let jittered = base.as_secs_f64() + self.rng.jitter(p.jitter.as_secs_f64());
        Duration::from_secs_f64(jittered.max(0.001))
    }
}

impl Camera for SimulatedCamera {
    fn role(&self) -> CameraRole {
        self.role
    }

    fn nominal_period(&self) -> Duration {
        let p = &self.profile;
        // The mean: a long follows a short with chance q, so longs are q / (1 + q) of intervals.
        let longs = p.long_after_short / (1.0 + p.long_after_short);
        Duration::from_secs_f64(p.short.as_secs_f64() * (1.0 - longs) + p.long.as_secs_f64() * longs)
    }

    fn describe(&self) -> Description {
        Description { device: format!("simulated-{}", self.role.name()), settings: format!("{:?}", self.profile) }
    }

    fn next_frame(&mut self) -> Result<Frame> {
        loop {
            let (sequence, t_capture) = (self.sequence, self.next_capture);
            let until_next = self.interval;
            self.sequence = self.sequence.wrapping_add(1);
            self.next_capture = self.next_capture + until_next;
            self.interval = self.draw_interval();
            let lost = self.lose.contains(&sequence)
                || (self.profile.lose_one_in > 0 && self.rng.chance(1.0 / f64::from(self.profile.lose_one_in)));
            if lost {
                continue;
            }
            // Delivered when the next frame starts (measured).
            let t_receive = t_capture + until_next + self.profile.delivery;
            self.clock.sleep_until(t_receive);
            return Ok(Frame { camera: self.role, sequence, t_capture, t_receive, jpeg: self.picture.clone() });
        }
    }
}
