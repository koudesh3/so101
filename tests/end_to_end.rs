//! Everything above the transport, exercised against the simulator.

use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use std::time::Duration;

use so101::robot::arm::Arm;
use so101::device::bus::Bus;
use so101::control::calibrate;
use so101::robot::calibration::ArmCalibration;
use so101::robot::joint::{Joint, COUNT, IDS};
use so101::wire::registers;
use so101::device::sim::{SimClock, SimulatedBus};
use so101::control::teleop::{self, TeleopOptions};
use so101::wire::transport::{read_log, ByteLog, Recorder};
use so101::report::visualize;
use so101::Error;

fn sim_bus(name: &str) -> Bus {
    Bus::new(Box::new(SimulatedBus::arm(name)))
}

fn sim_arm(sim: SimulatedBus, label: &str) -> Arm {
    Arm::new(label, Bus::new(Box::new(sim)), ArmCalibration::identity(label))
}

#[test]
fn ping_and_model_number() {
    let mut bus = sim_bus("bus");
    bus.ping(1).unwrap();
    let servo = bus.identify(1).unwrap();
    assert_eq!(servo.model, 777);
    assert!(servo.is_sts3215());
    assert!(matches!(bus.ping(9), Err(Error::NoReply { id: 9 })));
}

#[test]
fn scan_finds_exactly_six() {
    let mut bus = sim_bus("bus");
    assert_eq!(bus.scan(), vec![1, 2, 3, 4, 5, 6]);
}

#[test]
fn write_then_read_a_ram_register() {
    let mut bus = sim_bus("bus");
    bus.write(3, registers::ACCELERATION, 254).unwrap();
    assert_eq!(bus.read(3, registers::ACCELERATION).unwrap(), 254);
}

#[test]
fn eeprom_write_needs_unlock() {
    let mut bus = sim_bus("bus");
    bus.write(1, registers::P_COEFFICIENT, 16).unwrap();
    assert_eq!(bus.read(1, registers::P_COEFFICIENT).unwrap(), 32);
    bus.write_eeprom(1, registers::P_COEFFICIENT, 16).unwrap();
    assert_eq!(bus.read(1, registers::P_COEFFICIENT).unwrap(), 16);
    assert_eq!(bus.read(1, registers::LOCK).unwrap(), 1);
}

#[test]
fn read_only_register_is_refused_before_touching_the_wire() {
    let mut bus = sim_bus("bus");
    assert!(matches!(bus.write(1, registers::PRESENT_POSITION, 0), Err(Error::Refused(_))));
}

#[test]
fn servo_tracks_goal_within_speed_limit() {
    let clock = SimClock::manual();
    let sim = SimulatedBus::new("bus", &[1], clock.clone());
    let mut bus = Bus::new(Box::new(sim));
    bus.write(1, registers::TORQUE_ENABLE, 1).unwrap();
    bus.write(1, registers::GOAL_POSITION, 3048).unwrap();

    clock.advance(Duration::from_millis(100));
    let after_100ms = bus.read(1, registers::PRESENT_POSITION).unwrap();
    assert!(after_100ms > 2048 && after_100ms <= 2048 + 310, "{after_100ms}");

    clock.advance(Duration::from_secs(2));
    let settled = bus.read(1, registers::PRESENT_POSITION).unwrap();
    assert!((settled - 3048).abs() <= 2, "{settled}");
    assert_eq!(bus.read(1, registers::MOVING).unwrap(), 0);
}

#[test]
fn sync_read_returns_in_id_order_and_fails_on_a_missing_servo() {
    let mut sim = SimulatedBus::arm("bus");
    sim.servo_mut(4).unwrap().actual_ticks = 1000.0;
    let mut bus = Bus::new(Box::new(sim));
    let v = bus.sync_read(&[1, 4, 6], registers::PRESENT_POSITION).unwrap();
    assert_eq!(v, vec![2048, 1000, 2048]);

    let mut sim = SimulatedBus::arm("bus");
    sim.silenced_ids.push(3);
    let mut bus = Bus::new(Box::new(sim));
    let err = bus.sync_read(&IDS, registers::PRESENT_POSITION).unwrap_err();
    assert!(matches!(err, Error::WrongId { expected: 3, got: 4 }), "{err}");
}

#[test]
fn check_present_rejects_a_wrong_model() {
    let mut sim = SimulatedBus::arm("bus");
    sim.servo_mut(2).unwrap().set(registers::MODEL_NUMBER, 1234);
    let mut arm = sim_arm(sim, "a");
    let err = arm.check_present().unwrap_err();
    assert!(matches!(err, Error::UnexpectedModel { id: 2, model: 1234 }), "{err}");
}

#[test]
fn recentre_makes_every_joint_read_centre_without_moving() {
    let mut sim = SimulatedBus::arm("bus");
    for (i, s) in sim.servos.iter_mut().enumerate() {
        s.actual_ticks = 1500.0 + i as f64 * 100.0;
    }
    let mut arm = sim_arm(sim, "a");
    let offsets = arm.recentre().unwrap();
    assert_eq!(offsets, [-548, -448, -348, -248, -148, -48]);
    assert_eq!(arm.read_ticks().unwrap(), [2048; COUNT]);
}

#[test]
fn sweep_records_the_range_the_hand_covered() {
    let mut arm = sim_arm(SimulatedBus::waving_arm("bus"), "a");
    arm.torque_off().unwrap();
    let mut polls = 0;
    let sweep = calibrate::sweep(&mut arm, 200.0, |_| {
        polls += 1;
        polls < 60
    })
    .unwrap();
    for joint in Joint::ALL {
        let (lo, hi) = (sweep.min[joint.index()], sweep.max[joint.index()]);
        assert!(hi > lo, "{} did not move: {lo}..{hi}", joint.name());
        assert!(lo >= 0 && hi <= 4095);
    }
    let cal = ArmCalibration::from_sweep("a", [0; COUNT], sweep.min, sweep.max);
    assert!(cal.check_travel(5).is_ok());
    assert!(cal.check_travel(100_000).is_err());
}

#[test]
fn teleop_follows_the_leader_and_the_log_replays() {
    let dir = std::env::temp_dir().join(format!("so101-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let log_path = dir.join("bytes.jsonl");
    let log = ByteLog::create(&log_path).unwrap();

    let leader_sim = Recorder::new(SimulatedBus::waving_arm("leader"), "leader", log.clone());
    let follower_sim = Recorder::new(SimulatedBus::arm("follower"), "follower", log.clone());
    let mut leader = Arm::new("leader", Bus::new(Box::new(leader_sim)), ArmCalibration::identity("leader"));
    let mut follower = Arm::new("follower", Bus::new(Box::new(follower_sim)), ArmCalibration::identity("follower"));

    leader.check_present().unwrap();
    follower.check_present().unwrap();
    follower.configure_follower().unwrap();
    assert_eq!(follower.read_register(Joint::ShoulderPan, registers::P_COEFFICIENT).unwrap(), 16);

    let options = TeleopOptions { hz: 100.0, max_step_ticks: Some(200), max_ticks: Some(120) };
    let mut reports = Vec::new();
    let stats = teleop::run(&mut leader, &mut follower, options, Arc::new(AtomicBool::new(false)), |r| reports.push(r.clone())).unwrap();
    log.flush();

    assert_eq!(stats.ticks, 120);
    assert_eq!(stats.errors, 0);
    assert!(stats.overruns <= 2, "overruns {}", stats.overruns);

    let last = reports.last().unwrap();
    for joint in Joint::ALL {
        let i = joint.index();
        let gap = (last.leader_ticks[i] - last.follower_ticks[i]).abs();
        assert!(gap < 60, "{}: leader {} follower {} (gap {gap})", joint.name(), last.leader_ticks[i], last.follower_ticks[i]);
    }
    for pair in reports.windows(2) {
        for i in 0..COUNT {
            assert!((pair[1].goal_ticks_sent[i] - pair[0].follower_ticks[i]).abs() <= 200 + 5);
        }
    }
    assert_eq!(follower.read_register(Joint::ShoulderPan, registers::TORQUE_ENABLE).unwrap(), 0);

    let records = read_log(&log_path).unwrap();
    assert!(records.len() > 120 * 4, "only {} log records", records.len());
    assert!(records.windows(2).all(|w| w[0].t_us <= w[1].t_us), "log is not time-ordered");
    let decoded = visualize::decode(&records);
    assert_eq!(decoded.undecodable_bytes, 0);
    let follower_series = &decoded.series["follower"];
    let leader_series = &decoded.series["leader"];
    assert_eq!(follower_series.goals[&1].len(), 120 + 1, "one seed write plus one per tick");
    assert_eq!(leader_series.present[&1].len(), 120, "one leader read per tick");
    assert!(follower_series.present[&1].len() >= 120);
    assert_eq!(follower_series.goals[&1].last().unwrap().value, last.goal_ticks_sent[0]);

    let html_path = dir.join("report.html");
    visualize::report_from_log(&log_path, &html_path, 400).unwrap();
    let html = std::fs::read_to_string(&html_path).unwrap();
    assert!(html.contains("<svg"));
    assert!(html.contains("SYNC WRITE Goal_Position"));
    assert!(html.contains("SYNC READ Present_Position"));
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn a_silent_follower_servo_fails_teleop_before_the_loop_starts() {
    let mut follower_sim = SimulatedBus::arm("follower");
    follower_sim.silenced_ids.push(2);
    let mut leader = sim_arm(SimulatedBus::waving_arm("leader"), "leader");
    let mut follower = sim_arm(follower_sim, "follower");
    let options = TeleopOptions { hz: 200.0, max_step_ticks: Some(200), max_ticks: Some(10) };
    let mut errors = 0;
    let result = teleop::run(&mut leader, &mut follower, options, Arc::new(AtomicBool::new(false)), |r| {
        if r.error.is_some() {
            errors += 1
        }
    });
    assert!(result.is_err());
    assert_eq!(errors, 0);
}
