//! The cameras: which one (`CameraRole`), the frames they produce with their capture and receive
//! times (contract 3), and the `Camera` trait the real, synthetic and replayed cameras implement.
//! `stream` runs one on its own thread; `LatestFrame` is what the control loop reads, never waiting.

mod ffmpeg;
mod synthetic;

use std::fmt;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use bytes::Bytes;

use crate::clock::Timestamp;

pub use ffmpeg::{read_controls, FfmpegCamera};
pub use synthetic::SyntheticCamera;

/// Contract 3: a capture interval longer than this many nominal periods is a drop, even without a
/// sequence gap (the icSpring slowed to 16-20 fps under auto exposure with no gaps).
pub const LONG_INTERVAL: f64 = 1.5;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CameraRole {
    Wrist,
    Scene,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct FrameRef {
    pub camera: CameraRole,
    pub sequence: u32,
}

/// Logged as Foxglove's `CompressedImage`; `t_capture`, `t_receive` and `sequence` are the MCAP
/// record's `publish_time`, `log_time` and `sequence` (contract 3).
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub camera: CameraRole,
    /// The kernel's frame number for a V4L2 camera.
    pub sequence: u32,
    /// The kernel's buffer timestamp for a V4L2 camera; exposure starts about 10 ms earlier.
    pub t_capture: Timestamp,
    pub t_receive: Timestamp,
    pub jpeg: Bytes,
}

/// Which device a camera is and its settings as read back from it (contract 7).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Description {
    /// The physical device, e.g. its `/dev/v4l/by-id/...` path.
    pub device: String,
    /// Controls as read back, "name=value ...", in v4l2-ctl's names.
    pub settings: String,
}

/// A source of frames, real or not.
pub trait Camera: Send {
    fn role(&self) -> CameraRole;

    /// The period it's meant to deliver at; drops are judged against it (contract 3).
    fn nominal_period(&self) -> Duration;

    fn describe(&self) -> Description;

    /// Blocks until the next frame. `Error::Timeout` means none came yet; the caller may retry.
    fn next_frame(&mut self) -> Result<Frame>;
}

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug)]
pub enum Error {
    Io(std::io::Error),
    /// No frame within the wait; not fatal.
    Timeout,
    /// The source has no more frames (a replay at its end).
    Ended,
    /// The device refused or reported something we can't use.
    Device(String),
}

/// What contract 3 counts as a drop, kept apart: frames the host lost (sequence gaps) and
/// intervals the camera took too long over.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Continuity {
    pub frames: u64,
    /// Frames missing by sequence number.
    pub missing: u64,
    /// Capture intervals over `LONG_INTERVAL` nominal periods.
    pub long_intervals: u64,
    last: Option<(u32, Timestamp)>,
}

/// One camera on its own thread, handing every frame to a callback. Stop it with `stop`.
pub struct Streaming {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<StreamStats>>,
}

#[derive(Clone, Debug, Default)]
pub struct StreamStats {
    pub continuity: Continuity,
    /// Why it stopped, if not asked to.
    pub error: Option<String>,
}

/// The newest frame's identity and times, written by its camera's thread and read by the control
/// loop. Lock-free: the loop runs at real-time priority and must never wait on a camera thread.
pub struct LatestFrame {
    role: CameraRole,
    // A seqlock: odd while being written.
    version: AtomicU64,
    sequence: AtomicU64,
    t_capture: AtomicU64,
    t_receive: AtomicU64,
}

/// What `LatestFrame` holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Seen {
    pub id: FrameRef,
    pub t_capture: Timestamp,
    pub t_receive: Timestamp,
}

impl CameraRole {
    pub fn name(self) -> &'static str {
        match self {
            CameraRole::Wrist => "wrist",
            CameraRole::Scene => "scene",
        }
    }

    pub fn from_name(name: &str) -> Option<CameraRole> {
        match name {
            "wrist" => Some(CameraRole::Wrist),
            "scene" => Some(CameraRole::Scene),
            _ => None,
        }
    }
}

impl Frame {
    pub fn id(&self) -> FrameRef {
        FrameRef { camera: self.camera, sequence: self.sequence }
    }
}

impl Continuity {
    /// Counts `frame` against the one before it.
    pub fn add(&mut self, frame: &Frame, nominal_period: Duration) {
        self.frames += 1;
        if let Some((sequence, t)) = self.last {
            self.missing += u64::from(frame.sequence.wrapping_sub(sequence).saturating_sub(1));
            let interval = frame.t_capture.since(t);
            self.long_intervals += u64::from(interval.as_secs_f64() > LONG_INTERVAL * nominal_period.as_secs_f64());
        }
        self.last = Some((frame.sequence, frame.t_capture));
    }
}

/// Runs `camera` on its own thread until stopped or it fails, handing each frame to `on_frame`.
pub fn stream(mut camera: Box<dyn Camera>, mut on_frame: impl FnMut(Frame) + Send + 'static) -> Streaming {
    let stop = Arc::new(AtomicBool::new(false));
    let stop_seen = stop.clone();
    let name = format!("camera-{}", camera.role().name());
    let thread = std::thread::Builder::new()
        .name(name)
        .spawn(move || {
            let mut stats = StreamStats::default();
            let period = camera.nominal_period();
            while !stop_seen.load(Ordering::Relaxed) {
                match camera.next_frame() {
                    Ok(frame) => {
                        stats.continuity.add(&frame, period);
                        on_frame(frame);
                    }
                    Err(Error::Timeout) => {}
                    Err(e) => {
                        stats.error = Some(e.to_string());
                        break;
                    }
                }
            }
            stats
        })
        .expect("spawn a camera thread");
    Streaming { stop, thread: Some(thread) }
}

impl Streaming {
    /// Stops the camera (within its wait for a frame) and returns what it saw.
    pub fn stop(mut self) -> StreamStats {
        self.halt()
    }

    fn halt(&mut self) -> StreamStats {
        self.stop.store(true, Ordering::Relaxed);
        self.thread.take().and_then(|t| t.join().ok()).unwrap_or_default()
    }
}

impl Drop for Streaming {
    fn drop(&mut self) {
        self.halt();
    }
}

impl LatestFrame {
    pub fn new(role: CameraRole) -> Self {
        Self {
            role,
            version: AtomicU64::new(0),
            sequence: AtomicU64::new(0),
            t_capture: AtomicU64::new(0),
            t_receive: AtomicU64::new(0),
        }
    }

    /// From the camera's thread only: one writer per slot.
    pub fn put(&self, frame: &Frame) {
        let v = self.version.load(Ordering::Relaxed);
        self.version.store(v + 1, Ordering::Release);
        std::sync::atomic::fence(Ordering::Release);
        self.sequence.store(u64::from(frame.sequence), Ordering::Relaxed);
        self.t_capture.store(frame.t_capture.0, Ordering::Relaxed);
        self.t_receive.store(frame.t_receive.0, Ordering::Relaxed);
        self.version.store(v + 2, Ordering::Release);
    }

    /// The newest frame, or None before the first. Never blocks; retries only if it raced a write.
    pub fn get(&self) -> Option<Seen> {
        loop {
            let before = self.version.load(Ordering::Acquire);
            if before == 0 {
                return None;
            }
            if before % 2 == 1 {
                std::hint::spin_loop();
                continue;
            }
            let sequence = self.sequence.load(Ordering::Relaxed) as u32;
            let t_capture = Timestamp(self.t_capture.load(Ordering::Relaxed));
            let t_receive = Timestamp(self.t_receive.load(Ordering::Relaxed));
            std::sync::atomic::fence(Ordering::Acquire);
            if self.version.load(Ordering::Relaxed) == before {
                return Some(Seen { id: FrameRef { camera: self.role, sequence }, t_capture, t_receive });
            }
        }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io(e) => write!(f, "{e}"),
            Error::Timeout => write!(f, "no frame yet"),
            Error::Ended => write!(f, "no more frames"),
            Error::Device(s) => write!(f, "{s}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Error::Io(e)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(sequence: u32, ms: u64) -> Frame {
        Frame {
            camera: CameraRole::Wrist,
            sequence,
            t_capture: Timestamp(ms * 1_000_000),
            t_receive: Timestamp(ms * 1_000_000 + 5_000_000),
            jpeg: Bytes::new(),
        }
    }

    #[test]
    fn a_sequence_gap_and_a_long_interval_are_counted_apart() {
        let period = Duration::from_micros(33_333);
        let mut c = Continuity::default();
        for (seq, ms) in [(10, 0), (11, 33), (13, 100), (14, 133), (15, 200)] {
            c.add(&frame(seq, ms), period);
        }
        assert_eq!((c.frames, c.missing), (5, 1), "12 never arrived");
        assert_eq!(c.long_intervals, 2, "67 ms (with the gap) and 67 ms (without one)");
    }

    #[test]
    fn the_latest_frame_slot_is_empty_then_holds_the_newest() {
        let slot = LatestFrame::new(CameraRole::Wrist);
        assert_eq!(slot.get(), None);
        slot.put(&frame(7, 100));
        slot.put(&frame(8, 133));
        let seen = slot.get().unwrap();
        assert_eq!(seen.id, FrameRef { camera: CameraRole::Wrist, sequence: 8 });
        assert_eq!((seen.t_capture, seen.t_receive), (Timestamp(133_000_000), Timestamp(138_000_000)));
    }

    #[test]
    fn a_reader_never_sees_half_a_frame() {
        // Every frame written has t_capture = sequence ms and t_receive 5 ms later; a torn read
        // would mix two frames' fields.
        let slot = Arc::new(LatestFrame::new(CameraRole::Scene));
        let writer = {
            let slot = slot.clone();
            std::thread::spawn(move || {
                for s in 1..200_000u32 {
                    slot.put(&frame(s, u64::from(s)));
                }
            })
        };
        let mut reads = 0;
        while !writer.is_finished() {
            if let Some(seen) = slot.get() {
                assert_eq!(seen.t_capture.0, u64::from(seen.id.sequence) * 1_000_000);
                assert_eq!(seen.t_receive.0, seen.t_capture.0 + 5_000_000);
                reads += 1;
            }
        }
        writer.join().unwrap();
        assert!(reads > 0);
    }
}
