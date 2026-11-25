//! The session's tests (docs/phase0.md). The schema snapshot is in `schema_snapshot.rs`.

use std::io::{Cursor, Seek, SeekFrom, Write};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

use sample::joints;
use so101::arm::JointState;
use so101::camera::CameraRole;
use so101::clock::{Clock, MonotonicClock, ReplayClock, ReplaySpeed, Timestamp};
use so101::control::EventBus;
use so101::control::{ArmRole, Event, Message};
use so101::session::{recover, replay, Attachment, CameraInfo, DropKind, Header, Recorder, RecorderOptions, Session};

const MS: u64 = 1_000_000;
const CALIBRATION: &[u8] = br#"{"shoulder_pan": {"homing_offset": 12, "range_min": 800, "range_max": 3300}}"#;

/// A writer that blocks every write while its gate is closed: a stalled disk.
#[derive(Clone)]
struct StalledDisk {
    data: Arc<Mutex<Cursor<Vec<u8>>>>,
    open: Arc<(Mutex<bool>, Condvar)>,
}

impl StalledDisk {
    fn new() -> Self {
        Self { data: Arc::default(), open: Arc::new((Mutex::new(true), Condvar::new())) }
    }
    fn set_open(&self, open: bool) {
        *self.open.0.lock().unwrap() = open;
        self.open.1.notify_all();
    }
}

impl Write for StalledDisk {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let mut open = self.open.0.lock().unwrap();
        while !*open {
            open = self.open.1.wait(open).unwrap();
        }
        drop(open);
        self.data.lock().unwrap().write(buf)
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

impl Seek for StalledDisk {
    fn seek(&mut self, pos: SeekFrom) -> std::io::Result<u64> {
        self.data.lock().unwrap().seek(pos)
    }
}

fn header_fields() -> Header {
    let mut h = Header::now();
    // Fixed, so the header written and the header compared are the same.
    h.start_monotonic = Timestamp(4_000_000_000);
    h.start_wall_unix_ns = 1_795_000_000_000_000_000;
    h.git_commit = "0123abc".into();
    h.git_dirty = true;
    h.config.insert("servo_hz".into(), "120".into());
    h.cameras.insert(
        "wrist".into(),
        CameraInfo {
            device: "/dev/v4l/by-id/usb-icSpring_icspring_camera_20240307110322-video-index0".into(),
            settings: "auto_exposure=1 exposure_time_absolute=300 gain=0".into(),
        },
    );
    h.servos.insert(
        "gripper".into(),
        "P_Coefficient=32 I_Coefficient=0 D_Coefficient=32 Max_Torque=300 Torque_Limit=300".into(),
    );
    h
}

fn header_and_attachments() -> (Header, Vec<Attachment>) {
    let mut h = header_fields();
    let calibration = h.add_calibration("follower", CALIBRATION);
    let config =
        Attachment { name: "robot.toml".into(), media_type: "application/toml".into(), data: b"hz = 120\n".to_vec() };
    (h, vec![calibration, config])
}

fn header() -> Header {
    header_and_attachments().0
}

fn write(events: &[Event], options: RecorderOptions) -> (Vec<u8>, so101::session::RecordSummary) {
    let (header, attachments) = header_and_attachments();
    let recorder =
        Recorder::start(Cursor::new(Vec::new()), &header, &attachments, Arc::new(MonotonicClock), options).unwrap();
    let bus = EventBus::new();
    bus.attach(recorder.sink());
    for e in events {
        bus.publish(e.clone());
    }
    let (out, summary) = recorder.finish().unwrap();
    (out.into_inner(), summary)
}

fn state(cycle: u32, t: u64) -> Event {
    Event::new(
        Message::JointState(
            ArmRole::Follower,
            JointState { t: Timestamp(t), pose: joints(cycle as f32), counts: Some([2048; 6]) },
        ),
        cycle,
        Timestamp(t),
    )
}

/// 1. Every message type, written and read back, gives identical records: payloads (f32 bits),
///    sequence numbers and both timestamps.
#[test]
fn round_trip_is_identical() {
    let events = sample::session(Timestamp(5_000_000_000));
    let (bytes, summary) = write(&events, RecorderOptions::default());
    assert_eq!(summary.messages_written as usize, events.len());
    assert!(summary.complete);

    let log = Session::from_bytes(&bytes).unwrap();
    assert_eq!(log.events, events);
    assert_eq!(log.header, header());
    let end = log.end.clone().unwrap();
    assert!(end.complete);
    assert_eq!(end.messages_written as usize, events.len());
    assert_eq!(log.attachments.len(), 2);
    assert_eq!(log.calibration("follower"), Some(CALIBRATION), "the calibration file itself is in the log");
    assert_eq!(log.header.cameras["wrist"].settings, "auto_exposure=1 exposure_time_absolute=300 gain=0");

    // Contract 3: frame records carry receive time in log_time and capture time in publish_time.
    let frame = log.topic("/cam/wrist").next().unwrap();
    let Message::Frame(f) = &frame.message else { panic!("not a frame") };
    assert_eq!(frame.log_time, f.t_receive);
    assert_eq!(frame.publish_time, f.t_capture);
    assert_eq!(frame.sequence, f.sequence);
    assert!(log.drops().is_empty(), "{:?}", log.drops());
}

/// 2. A sequence gap and a long capture interval with no gap are both reported, as different kinds.
#[test]
fn drops_are_reported_by_kind() {
    let mut events = Vec::new();
    for seq in (0..12).filter(|s| *s != 5 && *s != 6) {
        let t = Timestamp(seq as u64 * 33 * MS);
        events.push(Event::frame(sample::frame(CameraRole::Wrist, seq, t, 500)));
    }
    let mut t = 0;
    for seq in 0..12u32 {
        t += if seq == 7 { 70 } else { 33 } * MS; // the camera itself skipped: no gap in its counter
        events.push(Event::frame(sample::frame(CameraRole::Scene, seq, Timestamp(t), 500)));
    }
    for cycle in (0..10).filter(|s| *s != 4) {
        events.push(state(cycle, cycle as u64 * 10 * MS)); // cycle 4 overran: skipped (contract 5)
    }
    let (bytes, _) = write(&events, RecorderOptions::default());
    let drops = Session::from_bytes(&bytes).unwrap().drops();

    let kinds: Vec<(&str, &DropKind)> = drops.iter().map(|d| (d.topic.as_str(), &d.kind)).collect();
    assert_eq!(kinds.len(), 3, "{drops:?}");
    assert!(kinds.contains(&("/cam/wrist", &DropKind::SequenceGap { after: 4, missing: 2 })), "{drops:?}");
    assert!(kinds.contains(&("/follower/state", &DropKind::SequenceGap { after: 3, missing: 1 })), "{drops:?}");
    assert!(
        kinds.iter().any(|(t, k)| *t == "/cam/scene"
            && matches!(k, DropKind::LongCaptureInterval { after: 6, interval, .. } if *interval == Duration::from_millis(70))),
        "{drops:?}"
    );
}

/// 3. With the disk stalled, publishing never blocks, frames are dropped first, every drop is on
///    `/log/drops`, and no small message is lost.
#[test]
fn a_stalled_disk_drops_frames_not_the_session() {
    let disk = StalledDisk::new();
    let options = RecorderOptions { frame_queue: 8, queue: 10_000, chunk_size: 32 * 1024 };
    let recorder = Recorder::start(
        disk.clone(),
        &header_and_attachments().0,
        &header_and_attachments().1,
        Arc::new(MonotonicClock),
        options,
    )
    .unwrap();
    let bus = EventBus::new();
    bus.attach(recorder.sink());

    disk.set_open(false);
    let mut slowest = Duration::ZERO;
    for i in 0..300u32 {
        let t = Timestamp(i as u64 * 33 * MS);
        let started = Instant::now();
        bus.publish(Event::frame(sample::frame(CameraRole::Wrist, i, t, 40_000)));
        bus.publish(state(i, t.0));
        slowest = slowest.max(started.elapsed());
    }
    assert!(slowest < Duration::from_millis(20), "a publish blocked for {slowest:?}");
    disk.set_open(true);
    let (_, summary) = recorder.finish().unwrap();
    assert!(summary.complete);
    assert!(summary.frames_dropped > 200, "only {} frames dropped", summary.frames_dropped);
    assert_eq!(summary.messages_lost, 0);

    let bytes = disk.data.lock().unwrap().get_ref().clone();
    let log = Session::from_bytes(&bytes).unwrap();
    assert_eq!(log.topic("/follower/state").count(), 300, "joint states were lost");
    let frames = log.topic("/cam/wrist").count() as u64;
    let recorded_drops: u64 = log
        .events
        .iter()
        .filter_map(|e| match &e.message {
            Message::RecorderDrop(d) => Some(d.count as u64),
            _ => None,
        })
        .sum();
    assert_eq!(recorded_drops, summary.frames_dropped);
    assert_eq!(frames + recorded_drops, 300);
    // The recorder's own drops explain every frame gap: nothing is reported twice.
    assert!(log.drops().iter().all(|d| matches!(d.kind, DropKind::Recorder { .. })), "{:?}", log.drops());
}

/// 3b. If even the main queue fills (the disk is effectively dead), the recorder stops and the
///     log says it's incomplete.
#[test]
fn a_dead_disk_marks_the_log_incomplete() {
    let disk = StalledDisk::new();
    let options = RecorderOptions { frame_queue: 4, queue: 16, chunk_size: 1024 };
    let recorder = Recorder::start(
        disk.clone(),
        &header_and_attachments().0,
        &header_and_attachments().1,
        Arc::new(MonotonicClock),
        options,
    )
    .unwrap();
    let bus = EventBus::new();
    bus.attach(recorder.sink());
    disk.set_open(false);
    for i in 0..500u32 {
        bus.publish(state(i, i as u64 * 10 * MS));
    }
    disk.set_open(true);
    let (_, summary) = recorder.finish().unwrap();
    assert!(!summary.complete);
    assert!(summary.messages_lost > 0);
    let log = Session::from_bytes(&disk.data.lock().unwrap().get_ref().clone()).unwrap();
    assert!(!log.end.unwrap().complete);
}

/// 4. A recorded run replayed on `ReplayClock` reproduces its timing.
#[test]
fn replay_reproduces_timing() {
    let offsets_ms = [0u64, 5, 7, 20, 21, 40, 41, 60];
    let t0 = 7_000_000_000;
    let events: Vec<Event> = offsets_ms.iter().enumerate().map(|(i, ms)| state(i as u32, t0 + ms * MS)).collect();
    let (bytes, _) = write(&events, RecorderOptions::default());
    let log = Session::from_bytes(&bytes).unwrap();
    let ordered = log.by_time();

    // As fast as possible: the clock reports each event's exact log time.
    let fast = ReplayClock::new(ordered[0].log_time, ReplaySpeed::AsFastAsPossible);
    let mut seen = Vec::new();
    replay(ordered.iter().copied(), &fast, |e| seen.push((fast.now(), e.log_time)));
    assert!(seen.iter().all(|(now, t)| now == t));

    // Real time: every event is delivered at its recorded offset, never early. Each wait is
    // scheduled from the replay's start, so lateness can't accumulate; what lateness there is
    // comes from the OS waking the thread, so the bound is loose enough for a busy test run.
    let start = MonotonicClock.now();
    let real = ReplayClock::new(ordered[0].log_time, ReplaySpeed::Factor(1.0));
    let mut delivered = Vec::new();
    replay(ordered.iter().copied(), &real, |_| delivered.push(MonotonicClock.now()));
    for (i, ms) in offsets_ms.iter().enumerate() {
        let got = delivered[i].since(start).as_secs_f64() * 1000.0;
        let late = got - *ms as f64;
        assert!(late > -0.5, "event {i}: replayed early, at {got:.2} ms instead of {ms} ms");
        assert!(late < 15.0, "event {i}: replayed at {got:.2} ms, recorded at {ms} ms");
    }
}

/// 6. A file cut short mid-write recovers every complete chunk, and the result opens normally.
#[test]
fn a_truncated_log_recovers_its_complete_chunks() {
    let mut events = Vec::new();
    for i in 0..120u32 {
        let t = Timestamp(i as u64 * 33 * MS);
        events.push(Event::frame(sample::frame(CameraRole::Wrist, i, t, 40_000)));
        events.push(state(i, t.0));
    }
    // A frame queue as long as the run, so a busy machine can't make the recorder drop frames:
    // this test is about recovering a cut file, and needs every message written.
    let options = RecorderOptions { chunk_size: 256 * 1024, frame_queue: events.len(), ..Default::default() };
    let (bytes, _) = write(&events, options);
    let cut = &bytes[..bytes.len() * 7 / 10];
    assert!(Session::from_bytes(cut).is_err(), "a truncated file must not read as complete");

    let (out, summary) = recover(cut, Cursor::new(Vec::new())).unwrap();
    assert!(summary.truncated.is_some());
    let log = Session::from_bytes(out.get_ref()).unwrap();
    let n = log.events.len();
    assert!(n > events.len() / 2 && n < events.len(), "recovered {n} of {}", events.len());
    assert_eq!(log.events[..], events[..n], "recovered messages must be an exact prefix");
    assert_eq!(log.header, header());
    assert!(log.end.is_none(), "the session-end record was never written");
}

/// Degrees only mean something with their calibration: a log whose calibration file doesn't match
/// the hash in its header is refused, not read.
#[test]
fn a_calibration_that_doesnt_match_its_hash_is_refused() {
    let mut h = header_fields();
    let mut calibration = h.add_calibration("follower", CALIBRATION);
    calibration.data = b"{}".to_vec(); // a different file under the same name
    let recorder = Recorder::start(
        Cursor::new(Vec::new()),
        &h,
        &[calibration],
        Arc::new(MonotonicClock),
        RecorderOptions::default(),
    )
    .unwrap();
    let (out, _) = recorder.finish().unwrap();
    let err = Session::from_bytes(out.get_ref()).unwrap_err();
    assert!(err.to_string().contains("doesn't match its hash"), "{err}");
}

/// Contract 1: Python decodes a log using only the schemas embedded in it (`read_session.py`).
#[test]
fn python_reads_a_session_without_generated_code() {
    let (bytes, _) = write(&sample::session(Timestamp(5_000 * MS)), RecorderOptions::default());
    let path = std::env::temp_dir().join(format!("so101-python-{}.mcap", std::process::id()));
    std::fs::write(&path, bytes).unwrap();
    let script = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/read_session.py");
    let out = std::process::Command::new("uv")
        .args(["run", "-q", script])
        .arg(&path)
        .output()
        .expect("uv not found: `mise install` provides it");
    let _ = std::fs::remove_file(&path);
    let said = format!("{}{}", String::from_utf8_lossy(&out.stdout), String::from_utf8_lossy(&out.stderr));
    assert!(out.status.success(), "{said}");
}

/// A small synthetic session with every message type.
mod sample {
    use bytes::Bytes;
    use so101::arm::Limit;
    use so101::arm::{ActuatorStatus, Fault, Joint, JointState, Pose};
    use so101::camera::{CameraRole, Frame, FrameRef};
    use so101::clock::Timestamp;
    use so101::control::{Clip, Tick};
    use so101::control::{JointTarget, Source};
    use so101::policy::{CheckpointId, Chunk};

    use so101::control::{ArmRole, Event, Message};

    const MS: u64 = 1_000_000;

    pub fn joints(base: f32) -> Pose {
        Pose([base, base + 1.5, -base, 0.25, 90.0, 42.0])
    }

    /// A fake JPEG: the SOI marker, then bytes that identify the camera and frame.
    pub fn jpeg(camera: CameraRole, sequence: u32, len: usize) -> Bytes {
        let mut v = vec![0xFF, 0xD8];
        v.extend((0..len.saturating_sub(2)).map(|i| (i as u32 ^ sequence ^ camera as u32) as u8));
        Bytes::from(v)
    }

    pub fn frame(camera: CameraRole, sequence: u32, t_capture: Timestamp, len: usize) -> Frame {
        Frame {
            camera,
            sequence,
            t_capture,
            t_receive: t_capture + std::time::Duration::from_millis(33),
            jpeg: jpeg(camera, sequence, len),
        }
    }

    /// About a third of a second of every event type, starting at `t0`, in publish order.
    pub fn session(t0: Timestamp) -> Vec<Event> {
        let t = |ms: u64| Timestamp(t0.0 + ms * MS);
        let mut out = Vec::new();
        for cycle in 0..30u32 {
            let ms = cycle as u64 * 10;
            let pos = joints(cycle as f32 * 0.5);
            let counts = Some([2048 + cycle as i32, 1900, 2100, 2048, 1024, 3000]);
            out.push(Event::new(
                Message::JointState(ArmRole::Follower, JointState { t: t(ms), pose: pos, counts }),
                cycle,
                t(ms + 1),
            ));
            out.push(Event::new(
                Message::JointState(ArmRole::Leader, JointState { t: t(ms), pose: pos, counts: None }),
                cycle,
                t(ms + 1),
            ));
            let joint = Joint::ALL[cycle as usize % 6];
            out.push(Event::new(
                Message::Actuator(
                    ArmRole::Follower,
                    ActuatorStatus {
                        t: t(ms + 2),
                        joint,
                        velocity_deg_s: Some(-12.5),
                        load_percent: Some(0.0),
                        voltage_v: Some(12.1),
                        temperature_c: Some(31.0),
                        current_a: Some(0.11),
                        faults: if cycle % 5 == 0 { vec![Fault::Overload] } else { vec![] },
                    },
                ),
                cycle,
                t(ms + 2),
            ));
            if cycle % 3 == 0 {
                let tick = cycle / 3;
                let wrist = frame(CameraRole::Wrist, tick, t(ms), 2000);
                let scene = frame(CameraRole::Scene, tick + 100, t(ms + 1), 3000);
                let wrist_id = wrist.id();
                let scene_id = scene.id();
                out.push(Event::frame(wrist));
                out.push(Event::frame(scene));
                out.push(Event::new(
                    Message::JointTarget(
                        ArmRole::Follower,
                        JointTarget {
                            t: t(ms + 3),
                            pose: joints(1.0),
                            source: Source::Teleop,
                            counts: Some([2060; 6]),
                        },
                    ),
                    tick,
                    t(ms + 3),
                ));
                out.push(Event::new(
                    Message::Tick(Tick {
                        t: t(ms + 3),
                        cycle,
                        state: pos,
                        wrist: Some(wrist_id),
                        scene: Some(FrameRef { camera: CameraRole::Scene, sequence: scene_id.sequence }),
                        action: joints(1.0),
                        source: Source::Policy(CheckpointId("ckpt-0005000".into())),
                        requested: joints(1.4),
                        clipped: if tick % 2 == 0 {
                            vec![Clip { joint: Joint::ElbowFlex, limit: Limit::Step }]
                        } else {
                            vec![]
                        },
                    }),
                    tick,
                    t(ms + 3),
                ));
            }
        }
        out.push(Event::new(
            Message::Chunk(Chunk {
                t_obs: t(5),
                period: std::time::Duration::from_nanos(33_333_333),
                actions: vec![joints(2.0), joints(2.5), joints(3.0)],
                checkpoint_id: CheckpointId("ckpt-0005000".into()),
            }),
            0,
            t(9),
        ));
        out.push(Event::new(
            Message::JointTarget(
                ArmRole::Follower,
                JointTarget { t: t(300), pose: joints(0.0), source: Source::Script("home".into()), counts: None },
            ),
            10,
            t(300),
        ));
        out
    }
}
