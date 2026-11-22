//! The Phase 0 tests from docs/phase0.md (the schema snapshot test lives in `contracts`, the
//! Python readability check in `just check-py`).

use std::io::{Cursor, Seek, SeekFrom, Write};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

use contracts::{Camera, Clock, JointState, MonotonicClock, ReplayClock, ReplaySpeed, Timestamp};
use session_log::sample::{self, joints};
use session_log::{
    recover, replay, Arm, Attachment, Bus, DropKind, Envelope, Event, Header, Log, Recorder, RecorderOptions,
};

const MS: u64 = 1_000_000;

fn header() -> Header {
    let mut h = Header::now();
    // Fixed, so the header written and the header compared are the same.
    h.start_monotonic = Timestamp(4_000_000_000);
    h.start_wall_unix_ns = 1_795_000_000_000_000_000;
    h.git_commit = "0123abc".into();
    h.git_dirty = true;
    h.config.insert("servo_hz".into(), "120".into());
    h
}

fn write(envelopes: &[Envelope], options: RecorderOptions) -> (Vec<u8>, session_log::RecordSummary) {
    let attachments =
        [Attachment { name: "robot.toml".into(), media_type: "application/toml".into(), data: b"hz = 120\n".to_vec() }];
    let recorder =
        Recorder::start(Cursor::new(Vec::new()), &header(), &attachments, Arc::new(MonotonicClock), options).unwrap();
    let bus = Bus::new();
    bus.attach(recorder.sink());
    for e in envelopes {
        bus.publish(e.clone());
    }
    let (out, summary) = recorder.finish().unwrap();
    (out.into_inner(), summary)
}

fn state(slot: u32, t: u64) -> Envelope {
    Envelope::new(
        Event::JointState(Arm::Follower, JointState { t: Timestamp(t), pos: joints(slot as f32) }),
        slot,
        Timestamp(t),
    )
}

/// 1. Every message type, written and read back, gives identical records: payloads (f32 bits),
///    sequence numbers and both timestamps.
#[test]
fn round_trip_is_identical() {
    let envelopes = sample::session(Timestamp(5_000_000_000));
    let (bytes, summary) = write(&envelopes, RecorderOptions::default());
    assert_eq!(summary.messages_written as usize, envelopes.len());
    assert!(summary.complete);

    let log = Log::from_bytes(&bytes).unwrap();
    assert_eq!(log.envelopes, envelopes);
    assert_eq!(log.header, header());
    let end = log.end.clone().unwrap();
    assert!(end.complete);
    assert_eq!(end.messages_written as usize, envelopes.len());
    assert_eq!(log.attachments.len(), 1);
    assert_eq!(log.attachments[0].data, b"hz = 120\n");

    // Contract 3: frame records carry receive time in log_time and capture time in publish_time.
    let frame = log.topic("/cam/wrist").next().unwrap();
    let Event::Frame(f) = &frame.event else { panic!("not a frame") };
    assert_eq!(frame.log_time, f.t_receive);
    assert_eq!(frame.publish_time, f.t_capture);
    assert_eq!(frame.sequence, f.sequence);
    assert!(log.drops().is_empty(), "{:?}", log.drops());
}

/// 2. A sequence gap and a long capture interval with no gap are both reported, as different kinds.
#[test]
fn drops_are_reported_by_kind() {
    let mut envelopes = Vec::new();
    for seq in (0..12).filter(|s| *s != 5 && *s != 6) {
        let t = Timestamp(seq as u64 * 33 * MS);
        envelopes.push(Envelope::frame(sample::frame(Camera::Wrist, seq, t, 500)));
    }
    let mut t = 0;
    for seq in 0..12u32 {
        t += if seq == 7 { 70 } else { 33 } * MS; // the camera itself skipped: no gap in its counter
        envelopes.push(Envelope::frame(sample::frame(Camera::Scene, seq, Timestamp(t), 500)));
    }
    for slot in (0..10).filter(|s| *s != 4) {
        envelopes.push(state(slot, slot as u64 * 10 * MS)); // slot 4 overran: skipped (contract 5)
    }
    let (bytes, _) = write(&envelopes, RecorderOptions::default());
    let drops = Log::from_bytes(&bytes).unwrap().drops();

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

/// 3. With the disk stalled, publishing never blocks, frames are dropped first, every drop is on
///    `/log/drops`, and no small message is lost.
#[test]
fn a_stalled_disk_drops_frames_not_the_session() {
    let disk = StalledDisk::new();
    let options = RecorderOptions { frame_queue: 8, queue: 10_000, chunk_size: 32 * 1024 };
    let recorder = Recorder::start(disk.clone(), &header(), &[], Arc::new(MonotonicClock), options).unwrap();
    let bus = Bus::new();
    bus.attach(recorder.sink());

    disk.set_open(false);
    let mut slowest = Duration::ZERO;
    for i in 0..300u32 {
        let t = Timestamp(i as u64 * 33 * MS);
        let started = Instant::now();
        bus.publish(Envelope::frame(sample::frame(Camera::Wrist, i, t, 40_000)));
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
    let log = Log::from_bytes(&bytes).unwrap();
    assert_eq!(log.topic("/follower/state").count(), 300, "joint states were lost");
    let frames = log.topic("/cam/wrist").count() as u64;
    let recorded_drops: u64 = log
        .envelopes
        .iter()
        .filter_map(|e| match &e.event {
            Event::LogDrop(d) => Some(d.count as u64),
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
    let recorder = Recorder::start(disk.clone(), &header(), &[], Arc::new(MonotonicClock), options).unwrap();
    let bus = Bus::new();
    bus.attach(recorder.sink());
    disk.set_open(false);
    for i in 0..500u32 {
        bus.publish(state(i, i as u64 * 10 * MS));
    }
    disk.set_open(true);
    let (_, summary) = recorder.finish().unwrap();
    assert!(!summary.complete);
    assert!(summary.messages_lost > 0);
    let log = Log::from_bytes(&disk.data.lock().unwrap().get_ref().clone()).unwrap();
    assert!(!log.end.unwrap().complete);
}

/// 4. A recorded run replayed on `ReplayClock` reproduces its timing.
#[test]
fn replay_reproduces_timing() {
    let offsets_ms = [0u64, 5, 7, 20, 21, 40, 41, 60];
    let t0 = 7_000_000_000;
    let envelopes: Vec<Envelope> = offsets_ms.iter().enumerate().map(|(i, ms)| state(i as u32, t0 + ms * MS)).collect();
    let (bytes, _) = write(&envelopes, RecorderOptions::default());
    let log = Log::from_bytes(&bytes).unwrap();
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
    let mut envelopes = Vec::new();
    for i in 0..120u32 {
        let t = Timestamp(i as u64 * 33 * MS);
        envelopes.push(Envelope::frame(sample::frame(Camera::Wrist, i, t, 40_000)));
        envelopes.push(state(i, t.0));
    }
    let (bytes, _) = write(&envelopes, RecorderOptions { chunk_size: 256 * 1024, ..Default::default() });
    let cut = &bytes[..bytes.len() * 7 / 10];
    assert!(Log::from_bytes(cut).is_err(), "a truncated file must not read as complete");

    let (out, summary) = recover(cut, Cursor::new(Vec::new())).unwrap();
    assert!(summary.truncated.is_some());
    let log = Log::from_bytes(out.get_ref()).unwrap();
    let n = log.envelopes.len();
    assert!(n > envelopes.len() / 2 && n < envelopes.len(), "recovered {n} of {}", envelopes.len());
    assert_eq!(log.envelopes[..], envelopes[..n], "recovered messages must be an exact prefix");
    assert_eq!(log.header, header());
    assert!(log.end.is_none(), "the session-end record was never written");
}
