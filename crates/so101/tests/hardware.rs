//! Tests on the real follower, run by hand on the Spark (ignored otherwise):
//! `SO101_FOLLOWER=/dev/serial/by-id/... cargo test --release --test hardware -- --ignored --nocapture`
//! The follower holds its pose (torque on) for about a second per start, then goes limp.

use std::path::Path;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;

use so101::arm::{Calibration, FollowerGains, JointState, Pose, Result, SerialPort, ServoArm, Servos};
use so101::clock::{MonotonicClock, Timestamp};
use so101::control::{run, EventBus, Message, Pilot, RunOptions, Source};

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
