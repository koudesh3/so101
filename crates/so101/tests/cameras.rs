//! The camera conformance suite (contract 3): every `Camera` (simulated, replay, V4L2) must
//! deliver frames whose times, sequence numbers and pictures mean what the contract says.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use std::io::Cursor;

use so101::camera::{stream, Camera, CameraRole, Continuity, LatestFrame, SimulatedCamera};
use so101::clock::{Clock, ReplayClock, ReplaySpeed, Timestamp};
use so101::control::{Event, EventBus};
use so101::session::{CameraInfo, Header, Recorder, RecorderOptions, ReplayCamera, Session};

/// Pulls `n` frames and checks each against contract 3; returns what it counted.
pub fn conforms(camera: &mut dyn Camera, n: usize) -> Continuity {
    let role = camera.role();
    let description = camera.describe();
    assert!(!description.device.is_empty(), "a camera says which device it is (contract 7)");
    let mut continuity = Continuity::default();
    let mut last: Option<so101::camera::Frame> = None;
    for _ in 0..n {
        // A timeout means none yet (a USB camera takes a moment to start): retry, for up to 3 s.
        let mut waits = 0;
        let frame = loop {
            match camera.next_frame() {
                Err(so101::camera::Error::Timeout) if waits < 15 => waits += 1,
                other => break other.expect("a frame"),
            }
        };
        assert_eq!(frame.camera, role);
        assert!(frame.t_receive >= frame.t_capture, "received before captured: {frame:?}");
        assert_eq!(&frame.jpeg[..2], &[0xFF, 0xD8], "a JPEG starts with SOI");
        assert_eq!(&frame.jpeg[frame.jpeg.len() - 2..], &[0xFF, 0xD9], "and ends with EOI");
        if let Some(prev) = &last {
            if frame.sequence != prev.sequence + 1 {
                println!(
                    "gap: frame {} after {} ({:.1} ms later, frame {} of {n})",
                    frame.sequence,
                    prev.sequence,
                    frame.t_capture.since(prev.t_capture).as_secs_f64() * 1e3,
                    continuity.frames + 1
                );
            }
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
fn the_simulated_camera_conforms() {
    let mut camera = SimulatedCamera::new(CameraRole::Wrist, stepped());
    let c = conforms(&mut camera, 120);
    assert_eq!((c.frames, c.missing, c.long_intervals), (120, 0, 0));
}

#[test]
fn a_lost_frame_shows_as_a_gap_and_a_long_interval() {
    let mut camera = SimulatedCamera::new(CameraRole::Wrist, stepped()).losing(&[5, 6]);
    let c = conforms(&mut camera, 20);
    assert_eq!(c.missing, 2);
    assert_eq!(c.long_intervals, 1, "three periods between frames 4 and 7");
}

#[test]
fn simulated_frames_arrive_at_exact_times_on_the_test_clock() {
    let clock = stepped();
    let mut camera = SimulatedCamera::new(CameraRole::Scene, clock.clone())
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
    let camera = SimulatedCamera::new(CameraRole::Wrist, stepped());
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

/// A session holding `n` simulated wrist frames (one of them lost), written and read back.
fn recorded_wrist(n: usize) -> (Session, Vec<so101::camera::Frame>) {
    let clock = stepped();
    let mut header = Header::now();
    header.cameras.insert(
        "wrist".into(),
        CameraInfo { device: "/dev/v4l/by-id/wrist".into(), settings: "exposure_time_absolute=120".into() },
    );
    let recorder =
        Recorder::start(Cursor::new(Vec::new()), &header, &[], clock.clone(), RecorderOptions::default()).unwrap();
    let bus = EventBus::new();
    bus.attach(recorder.sink());
    let mut camera = SimulatedCamera::new(CameraRole::Wrist, clock).losing(&[3]);
    let frames: Vec<_> = (0..n).map(|_| camera.next_frame().unwrap()).collect();
    for f in &frames {
        bus.publish(Event::frame(f.clone()));
    }
    let (out, _) = recorder.finish().unwrap();
    (Session::from_bytes(&out.into_inner()).unwrap(), frames)
}

#[test]
fn the_replay_camera_conforms_and_keeps_what_was_recorded() {
    let (session, recorded) = recorded_wrist(40);
    let clock = stepped();
    let mut camera = ReplayCamera::new(&session, CameraRole::Wrist, clock.clone()).unwrap();
    assert_eq!(camera.describe().device, "replay of /dev/v4l/by-id/wrist");
    assert_eq!(camera.describe().settings, "exposure_time_absolute=120", "the recorded settings travel along");
    let c = conforms(&mut camera, 40);
    assert_eq!((c.missing, c.long_intervals), (1, 1), "the recording's loss replays as a loss");
    let mut again = ReplayCamera::new(&session, CameraRole::Wrist, stepped()).unwrap();
    for original in &recorded {
        let replayed = again.next_frame().unwrap();
        assert_eq!((replayed.sequence, &replayed.jpeg), (original.sequence, &original.jpeg));
        assert_eq!(
            replayed.t_receive.0 - replayed.t_capture.0,
            original.t_receive.0 - original.t_capture.0,
            "staleness is kept"
        );
    }
    assert!(matches!(again.next_frame(), Err(so101::camera::Error::Ended)));
}

#[test]
fn a_role_the_session_never_recorded_has_no_replay_camera() {
    let (session, _) = recorded_wrist(5);
    assert!(ReplayCamera::new(&session, CameraRole::Scene, stepped()).is_none());
}

/// The real thing, on the Spark: `SO101_TEST_CAMERA=/dev/v4l/by-id/... cargo test --test cameras
/// -- --ignored` (nothing else may have the camera open).
#[cfg(target_os = "linux")]
#[test]
#[ignore]
fn a_real_camera_conforms() {
    use so101::camera::V4l2Camera;
    use so101::clock::MonotonicClock;
    let path = std::env::var("SO101_TEST_CAMERA").expect("SO101_TEST_CAMERA=<device>");
    let controls: Vec<String> =
        std::env::var("SO101_TEST_CONTROLS").unwrap_or_default().split_whitespace().map(str::to_string).collect();
    let clock = Arc::new(MonotonicClock);
    let mut camera = V4l2Camera::open(CameraRole::Wrist, &path, (640, 480), &controls, clock.clone()).unwrap();
    println!("{:?}, period {:?}", camera.describe(), camera.nominal_period());
    let c = conforms(&mut camera, 300);
    println!("{c:?}; skipped as corrupt or truncated: {}", camera.skipped());
    assert_eq!(c.missing, 0, "no frames lost in 10 s");
    assert!(c.long_intervals <= 1, "the camera kept its rate");
}
