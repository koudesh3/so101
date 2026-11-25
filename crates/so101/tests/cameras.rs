//! The camera conformance suite (contract 3): every `Camera` (synthetic, replay, V4L2) must
//! deliver frames whose times, sequence numbers and pictures mean what the contract says.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use so101::camera::{stream, Camera, CameraRole, Continuity, LatestFrame, SyntheticCamera};
use so101::clock::{Clock, ReplayClock, ReplaySpeed, Timestamp};

/// Pulls `n` frames and checks each against contract 3; returns what it counted.
pub fn conforms(camera: &mut dyn Camera, n: usize) -> Continuity {
    let role = camera.role();
    let description = camera.describe();
    assert!(!description.device.is_empty(), "a camera says which device it is (contract 7)");
    let mut continuity = Continuity::default();
    let mut last: Option<so101::camera::Frame> = None;
    for _ in 0..n {
        let frame = camera.next_frame().expect("a frame");
        assert_eq!(frame.camera, role);
        assert!(frame.t_receive >= frame.t_capture, "received before captured: {frame:?}");
        assert_eq!(&frame.jpeg[..2], &[0xFF, 0xD8], "a JPEG starts with SOI");
        assert_eq!(&frame.jpeg[frame.jpeg.len() - 2..], &[0xFF, 0xD9], "and ends with EOI");
        if let Some(prev) = &last {
            assert!(frame.t_capture > prev.t_capture, "capture times only go forward");
            assert!(frame.sequence > prev.sequence, "sequence numbers only go forward");
        }
        continuity.add(&frame, camera.nominal_period());
        last = Some(frame);
    }
    continuity
}

fn stepped() -> Arc<ReplayClock> {
    Arc::new(ReplayClock::new(Timestamp(0), ReplaySpeed::AsFastAsPossible))
}

#[test]
fn the_synthetic_camera_conforms() {
    let mut camera = SyntheticCamera::new(CameraRole::Wrist, stepped());
    let c = conforms(&mut camera, 120);
    assert_eq!((c.frames, c.missing, c.long_intervals), (120, 0, 0));
}

#[test]
fn a_lost_frame_shows_as_a_gap_and_a_long_interval() {
    let mut camera = SyntheticCamera::new(CameraRole::Wrist, stepped()).losing(&[5, 6]);
    let c = conforms(&mut camera, 20);
    assert_eq!(c.missing, 2);
    assert_eq!(c.long_intervals, 1, "three periods between frames 4 and 7");
}

#[test]
fn synthetic_frames_arrive_at_exact_times_on_the_test_clock() {
    let clock = stepped();
    let mut camera = SyntheticCamera::new(CameraRole::Scene, clock.clone())
        .with_period(Duration::from_millis(40))
        .with_latency(Duration::from_millis(10));
    let first = camera.next_frame().unwrap();
    assert_eq!((first.t_capture, first.t_receive), (Timestamp(40_000_000), Timestamp(50_000_000)));
    assert_eq!(clock.now(), first.t_receive, "it waited for the frame to arrive");
    let second = camera.next_frame().unwrap();
    assert_eq!(second.t_capture, Timestamp(80_000_000));
}

#[test]
fn a_streamed_camera_feeds_its_latest_frame_slot_until_stopped() {
    let slot = Arc::new(LatestFrame::new(CameraRole::Wrist));
    let seen = Arc::new(Mutex::new(0u64));
    let camera = SyntheticCamera::new(CameraRole::Wrist, stepped());
    let streaming = {
        let (slot, seen) = (slot.clone(), seen.clone());
        stream(Box::new(camera), move |frame| {
            slot.put(&frame);
            *seen.lock().unwrap() += 1;
        })
    };
    while *seen.lock().unwrap() < 50 {
        std::thread::yield_now();
    }
    let stats = streaming.stop();
    assert!(stats.continuity.frames >= 50 && stats.error.is_none(), "{stats:?}");
    assert_eq!(stats.continuity.missing, 0);
    let latest = slot.get().unwrap();
    assert_eq!(u64::from(latest.id.sequence) + 1, stats.continuity.frames, "the slot holds the last frame");
}
