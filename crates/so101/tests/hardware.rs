//! Tests on the real follower, run by hand on the Spark (ignored otherwise):
//! `SO101_FOLLOWER=/dev/serial/by-id/... cargo test --release --test hardware -- --ignored --nocapture`
//! The follower holds its pose (torque on) for about a second per start, then goes limp.

use std::path::Path;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;

use so101::arm::{
    Calibration, ClearanceMap, FollowerGains, Joint, JointState, Limit, Pose, Result, SerialPort, ServoArm, Servos,
    SimulatedServos, JOINT_COUNT,
};
use so101::clock::{Clock, MonotonicClock, ReplayClock, ReplaySpeed, Timestamp};
use so101::control::{run, Event, EventBus, Message, Pilot, RunOptions, Source};
use so101::session::{Attachment, Header, Recorder, RecorderOptions};

/// Commands the follower's pose as it was when torque came on: nothing moves.
struct Hold(Option<Pose>);

impl Pilot for Hold {
    fn source(&self) -> Source {
        Source::Script("hold".into())
    }

    fn target(&mut self, _: Timestamp, follower: &JointState) -> Result<(Pose, Vec<Message>)> {
        Ok((*self.0.get_or_insert(follower.pose), Vec::new()))
    }
}

fn repo(path: &str) -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..").join(path)
}

/// Teleop's start, without the leader: configure, read the gains back, torque on, then 120 cycles
/// at 120 Hz. 20 starts, no first-cycle error (the 2026-11-23 "reply from servo 2, expected 1").
#[test]
#[ignore]
fn twenty_starts_without_a_first_cycle_error() {
    let port = std::env::var("SO101_FOLLOWER").expect("SO101_FOLLOWER=<serial port>");
    let gains = FollowerGains::load(&repo("data/gains.json")).unwrap();
    let calibration = Calibration::load(&repo("data/calibration/follower.json")).unwrap();
    let mut failures = Vec::new();
    for start in 1..=20 {
        let servos = Servos::new(Box::new(SerialPort::open(&port).unwrap()));
        let mut follower = ServoArm::new("follower", servos, calibration.clone());
        follower.configure_readings().unwrap();
        follower.configure_follower_with(gains).unwrap();
        let registers = follower.check_gains(&gains).unwrap();
        if start == 1 {
            println!("gripper as read back: {}", registers[5]);
        }
        let mut errors = Vec::new();
        let options = RunOptions { hz: 120.0, max_ticks: Some(120), read_actuators: true };
        let stats = run(
            &mut follower,
            &mut Hold(None),
            options,
            Arc::new(MonotonicClock),
            &AtomicBool::new(false),
            &EventBus::new(),
            |e| errors.push(e.to_string()),
        )
        .unwrap();
        println!("start {start:2}: {} cycles, {} errors {errors:?}", stats.ticks, stats.errors);
        if stats.errors > 0 {
            failures.push(start);
        }
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    assert!(failures.is_empty(), "starts with errors: {failures:?}");
}

// ---------------------------------------------------------------------------------------------
// System identification: a scripted goal sequence, one joint at a time, from a reaching pose,
// recorded through the control loop. Fitting the simulator uses this run; checking it uses data
// the fit never saw (teleop episodes, the fine-motion test).

/// The 23 Nov tuning runs' reaching pose (upper arm about 45° up, forearm level), centred on pan
/// and roll, gripper 40% open so it can step both ways.
const REACH: [f32; JOINT_COUNT] = [0.0, -33.0, 9.0, 39.0, 0.0, 40.0];
const EASE_S: f64 = 5.0;
const STEP_HOLD_S: f64 = 1.2;

/// Per joint: the steps (out and back, both ways), then ramps, then small reversals.
#[derive(Clone, Copy)]
enum Move {
    Hold { offset: f64, seconds: f64 },
    // A triangle from 0 to +amplitude to -amplitude and back to 0, `periods` times.
    Triangle { amplitude: f64, speed: f64, periods: u32 },
}

struct Segment {
    joint: Joint,
    start_s: f64,
    seconds: f64,
    shape: Move,
}

/// The whole run: ease from where the arm is to REACH, each joint's moves, ease back.
struct Schedule {
    segments: Vec<Segment>,
    moves_end_s: f64,
}

impl Move {
    fn seconds(self) -> f64 {
        match self {
            Move::Hold { seconds, .. } => seconds,
            Move::Triangle { amplitude, speed, periods } => 4.0 * amplitude / speed * f64::from(periods),
        }
    }

    fn offset(self, t: f64) -> f64 {
        match self {
            Move::Hold { offset, .. } => offset,
            Move::Triangle { amplitude, speed, .. } => {
                let quarter = amplitude / speed;
                let phase = (t / quarter).rem_euclid(4.0);
                let up = |x: f64| x * amplitude;
                match phase {
                    p if p < 1.0 => up(p),
                    p if p < 3.0 => up(2.0 - p),
                    p => up(p - 4.0),
                }
            }
        }
    }
}

impl Schedule {
    fn new() -> Self {
        let mut segments = Vec::new();
        let mut t = EASE_S + 2.0;
        for joint in Joint::ALL {
            let big = match joint {
                Joint::ShoulderLift | Joint::ElbowFlex | Joint::WristFlex => 20.0,
                Joint::Gripper => 30.0,
                _ => 30.0,
            };
            let mut moves = vec![Move::Hold { offset: 0.0, seconds: 1.0 }];
            for size in [1.0, 3.0, 10.0, big] {
                for offset in [size, 0.0, -size, 0.0] {
                    moves.push(Move::Hold { offset, seconds: STEP_HOLD_S });
                }
            }
            for (speed, periods) in [(10.0, 1), (50.0, 2), (150.0, 3)] {
                moves.push(Move::Triangle { amplitude: 15.0, speed, periods });
                moves.push(Move::Hold { offset: 0.0, seconds: 0.6 });
            }
            for (amplitude, speed) in [(0.5, 1.0), (1.0, 2.0), (2.0, 4.0)] {
                moves.push(Move::Triangle { amplitude, speed, periods: 2 });
            }
            moves.push(Move::Hold { offset: 0.0, seconds: 1.0 });
            for shape in moves {
                segments.push(Segment { joint, start_s: t, seconds: shape.seconds(), shape });
                t += shape.seconds();
            }
        }
        Self { segments, moves_end_s: t }
    }

    fn end_s(&self) -> f64 {
        self.moves_end_s + EASE_S + 1.0
    }

    /// The target at `t` seconds, for an arm that started at `home`.
    fn at(&self, t: f64, home: Pose) -> Pose {
        let ease = |x: f64| {
            let x = x.clamp(0.0, 1.0);
            x * x * (3.0 - 2.0 * x)
        };
        let blend = |a: &[f32; JOINT_COUNT], b: &[f32; JOINT_COUNT], u: f64| {
            Pose(std::array::from_fn(|i| a[i] + (b[i] - a[i]) * u as f32))
        };
        if t < EASE_S + 2.0 {
            return blend(&home.0, &REACH, ease(t / EASE_S));
        }
        if t >= self.moves_end_s {
            return blend(&REACH, &home.0, ease((t - self.moves_end_s) / EASE_S));
        }
        let mut pose = Pose(REACH);
        if let Some(s) = self.segments.iter().find(|s| t >= s.start_s && t < s.start_s + s.seconds) {
            pose.0[s.joint.index()] += s.shape.offset(t - s.start_s) as f32;
        }
        pose
    }
}

/// Plays the schedule from the follower's pose when it starts.
struct SystemId {
    schedule: Schedule,
    started: Option<(Timestamp, Pose)>,
}

impl Pilot for SystemId {
    fn source(&self) -> Source {
        Source::Script("system-id".into())
    }

    fn target(&mut self, now: Timestamp, follower: &JointState) -> Result<(Pose, Vec<Message>)> {
        let (t0, home) = *self.started.get_or_insert((now, follower.pose));
        Ok((self.schedule.at(now.since(t0).as_secs_f64(), home), Vec::new()))
    }
}

// The folded rest pose the arm usually starts from (read 2026-11-24).
const REST: [f32; JOINT_COUNT] = [-18.9, -104.2, 94.3, 75.1, -5.1, 2.4];

/// Every scripted target is inside the calibrated ranges by 5° (5% on the gripper), the eases to
/// and from where the arm starts (REST, which sits within 1° of lift's end) stay inside them, and
/// the wrist camera stays clear of the arm throughout, every 10 ms; and the run completes on the
/// simulator with nothing clipped but the range and step limits it doesn't use.
#[test]
fn the_system_id_run_is_safe_and_completes_on_the_simulator() {
    let schedule = Schedule::new();
    let calibration = Calibration::load(&repo("data/calibration/follower.json")).unwrap();
    let map = ClearanceMap::load(&repo("data/camera_clearance.json")).unwrap();
    let deg = 360.0 / 4096.0;
    let mut t = 0.0;
    let mut closest = f32::MAX;
    while t < schedule.end_s() {
        let pose = schedule.at(t, Pose(REST));
        for joint in Joint::ALL {
            let c = calibration.joints[joint.index()];
            let (lo, hi, margin) = if joint == Joint::Gripper {
                (0.0, 100.0, 5.0)
            } else {
                let mid = (c.range_min + c.range_max) as f32 / 2.0;
                ((c.range_min as f32 - mid) * deg, (c.range_max as f32 - mid) * deg, 5.0)
            };
            let v = pose.0[joint.index()];
            let margin = if t >= EASE_S && t < schedule.moves_end_s { margin } else { 0.0 };
            assert!(
                v >= lo + margin && v <= hi - margin,
                "{} at {v:.1} (t {t:.2} s), range {lo:.0}..{hi:.0}",
                joint.name()
            );
        }
        closest = closest.min(map.clearance_mm(&pose));
        t += 0.01;
    }
    assert!(closest >= 10.0, "the camera comes within {closest:.1} mm of the arm");
    // SO101_SCHEDULE_CSV=<file> writes the targets, for a look before the arm moves.
    if let Ok(path) = std::env::var("SO101_SCHEDULE_CSV") {
        let mut rows = String::from("t,pan,lift,elbow,wrist_flex,wrist_roll,gripper\n");
        let mut t = 0.0;
        while t < schedule.end_s() {
            let p = schedule.at(t, Pose(REST)).0;
            rows.push_str(&format!("{t:.3},{}\n", p.map(|v| format!("{v:.3}")).join(",")));
            t += 0.01;
        }
        std::fs::write(path, rows).unwrap();
    }
    println!("system id: {:.0} s, camera never closer than {closest:.1} mm", schedule.end_s());

    // The same run on the simulated follower.
    let clock = Arc::new(ReplayClock::new(Timestamp(0), ReplaySpeed::AsFastAsPossible));
    let mut sim = SimulatedServos::arm("follower").with_clock(clock.clone());
    let counts = calibration.to_counts(&Pose(REST));
    for (i, id) in so101::arm::IDS.iter().enumerate() {
        sim.servo_mut(*id).unwrap().actual_counts = f64::from(counts[i]);
    }
    let mut follower = ServoArm::new("follower", Servos::new(Box::new(sim)), calibration).with_clock(clock.clone());
    follower.set_camera_guard(map, 10.0);
    let bus = EventBus::new();
    let events = bus.subscribe();
    let ticks = (schedule.end_s() * 120.0) as u64;
    let mut pilot = SystemId { schedule, started: None };
    let options = RunOptions { hz: 120.0, max_ticks: Some(ticks), read_actuators: true };
    let stats = run(&mut follower, &mut pilot, options, clock, &AtomicBool::new(false), &bus, |_| {}).unwrap();
    drop(bus);
    assert_eq!((stats.errors, stats.skipped), (0, 0), "{stats:?}");
    let clipped: Vec<Limit> = events
        .into_iter()
        .filter_map(|e| match e.message {
            Message::Tick(t) => Some(t.clipped.into_iter().map(|c| c.limit).collect::<Vec<_>>()),
            _ => None,
        })
        .flatten()
        .collect();
    assert!(!clipped.iter().any(|l| matches!(l, Limit::Collision | Limit::Range)), "{clipped:?}");
}

/// On the real follower: `SO101_FOLLOWER=<port> SO101_RECORD=<file.mcap> cargo test --release
/// --test hardware system_identification -- --ignored --nocapture`. About 5 minutes of motion.
#[test]
#[ignore]
fn system_identification() {
    let port = std::env::var("SO101_FOLLOWER").expect("SO101_FOLLOWER=<serial port>");
    let out = std::env::var("SO101_RECORD").expect("SO101_RECORD=<file.mcap>");
    let gains = FollowerGains::load(&repo("data/gains.json")).unwrap();
    let calibration_file = std::fs::read(repo("data/calibration/follower.json")).unwrap();
    let calibration = Calibration::load(&repo("data/calibration/follower.json")).unwrap();
    let mut follower = ServoArm::new("follower", Servos::new(Box::new(SerialPort::open(&port).unwrap())), calibration);
    follower.configure_readings().unwrap();
    follower.configure_follower_with(gains).unwrap();
    let registers = follower.check_gains(&gains).unwrap();
    follower.set_camera_guard(ClearanceMap::load(&repo("data/camera_clearance.json")).unwrap(), 10.0);

    let mut header = Header::now();
    header.config.insert("source".into(), "system-id".into());
    header.config.insert("reach_pose".into(), format!("{REACH:?}"));
    for (joint, r) in Joint::ALL.iter().zip(registers) {
        header.servos.insert(joint.name().into(), r);
    }
    let attachments: Vec<Attachment> = vec![
        header.add_calibration("follower", &calibration_file),
        Attachment {
            name: "gains.json".into(),
            media_type: "application/json".into(),
            data: std::fs::read(repo("data/gains.json")).unwrap(),
        },
    ];
    let clock: Arc<dyn Clock> = Arc::new(MonotonicClock);
    let file = std::io::BufWriter::new(std::fs::File::create_new(&out).unwrap());
    let recorder = Recorder::start(file, &header, &attachments, clock.clone(), RecorderOptions::default()).unwrap();
    let bus = EventBus::new();
    bus.attach(recorder.sink());
    let schedule = Schedule::new();
    let ticks = (schedule.end_s() * 120.0) as u64;
    println!("system id: {:.0} s of motion; Ctrl-C stops it with torque off", schedule.end_s());
    let mut pilot = SystemId { schedule, started: None };
    let options = RunOptions { hz: 120.0, max_ticks: Some(ticks), read_actuators: true };
    let stats =
        run(&mut follower, &mut pilot, options, clock, &AtomicBool::new(false), &bus, |e| println!("ERROR {e}"));
    drop(bus);
    let (_, summary) = recorder.finish().unwrap();
    println!("{stats:?}\nrecorded {out}: {summary:?}");
    stats.unwrap();
    let _: Option<Event> = None;
}
