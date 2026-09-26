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

    let options = TeleopOptions { hz: 100.0, max_step_ticks: Some(200), max_ticks: Some(120), telemetry: false, lead: [Duration::ZERO; COUNT] };
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
    // The step limit bounds how fast the command moves, starting from where the follower was.
    for i in 0..COUNT {
        assert!((reports[0].goal_ticks_sent[i] - reports[0].follower_ticks[i]).abs() <= 200);
    }
    for pair in reports.windows(2) {
        for i in 0..COUNT {
            assert!((pair[1].goal_ticks_sent[i] - pair[0].goal_ticks_sent[i]).abs() <= 200);
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
    // check_present's identify() reads Present_Position once, then one read per tick.
    assert_eq!(leader_series.present[&1].len(), 1 + 120, "one identify read plus one per tick");
    assert!(follower_series.present[&1].len() >= 120);
    assert_eq!(follower_series.goals[&1].last().unwrap().value, last.goal_ticks_sent[0]);

    let html_path = dir.join("report.html");
    // Configuration writes come first in the log; show enough rows to reach teleop.
    visualize::report_from_log(&log_path, &html_path, 1500).unwrap();
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
    let options = TeleopOptions { hz: 200.0, max_step_ticks: Some(200), max_ticks: Some(10), telemetry: false, lead: [Duration::ZERO; COUNT] };
    let mut errors = 0;
    let result = teleop::run(&mut leader, &mut follower, options, Arc::new(AtomicBool::new(false)), |r| {
        if r.error.is_some() {
            errors += 1
        }
    });
    assert!(result.is_err());
    assert_eq!(errors, 0);
}

#[test]
fn configure_follower_caps_the_gripper_only() {
    let mut arm = sim_arm(SimulatedBus::arm("follower"), "follower");
    arm.configure_follower().unwrap();
    assert_eq!(arm.read_register(Joint::Gripper, registers::MAX_TORQUE).unwrap(), 500);
    assert_eq!(arm.read_register(Joint::Gripper, registers::PROTECTION_CURRENT).unwrap(), 250);
    assert_eq!(arm.read_register(Joint::Gripper, registers::OVERLOAD_TORQUE).unwrap(), 25);
    assert_eq!(arm.read_register(Joint::Gripper, registers::TORQUE_LIMIT).unwrap(), 500);
    assert_ne!(arm.read_register(Joint::ShoulderPan, registers::MAX_TORQUE).unwrap(), 500, "arm joints keep full torque");
}

#[test]
fn telemetry_block_decodes_every_field_in_one_read() {
    let mut arm = sim_arm(SimulatedBus::arm("follower"), "follower");
    let t = arm.read_telemetry_block(Joint::ElbowFlex).unwrap();
    assert_eq!(t.joint, Joint::ElbowFlex);
    assert_eq!(t.position_ticks, 2048);
    assert_eq!(t.temperature_c, 31);
    assert_eq!(t.voltage_tenths, 74);
    assert!(t.status.is_healthy());
}

#[test]
fn a_blocked_gripper_shows_load_and_no_speed() {
    use so101::device::sim::demo;
    let clock = SimClock::manual();
    let sim = SimulatedBus::new("follower", &IDS, clock.clone()).with_obstacle(demo::object());
    let mut arm = sim_arm(sim, "follower");
    arm.configure_follower().unwrap();
    arm.torque_on_holding().unwrap();
    // Into the window where the object sits in the jaws, then squeeze past it.
    clock.advance(Duration::from_secs_f64(6.0));
    arm.write_goal_ticks([2048, 2048, 2048, 2048, 2048, 1600], None, None).unwrap();
    for _ in 0..50 {
        clock.advance(Duration::from_millis(20));
        let _ = arm.read_ticks().unwrap();
    }
    let t = arm.read_telemetry_block(Joint::Gripper).unwrap();
    assert!(t.load.abs() >= 400, "load {}", t.load);
    assert!(t.load.abs() <= 500, "the gripper cap should hold load to 50%, got {}", t.load);
    assert!(t.speed.abs() < 5, "speed {}", t.speed);
    assert!(t.position_ticks > 1900, "jaws stopped at the object, got {}", t.position_ticks);
}

#[test]
fn teleop_with_telemetry_reports_one_servo_per_tick_in_rotation() {
    let mut leader = sim_arm(SimulatedBus::waving_arm("leader"), "leader");
    let mut follower = sim_arm(SimulatedBus::arm("follower"), "follower");
    let options = TeleopOptions { hz: 200.0, max_step_ticks: Some(200), max_ticks: Some(12), telemetry: true, lead: [Duration::ZERO; COUNT] };
    let mut seen = Vec::new();
    teleop::run(&mut leader, &mut follower, options, Arc::new(AtomicBool::new(false)), |r| {
        seen.push(r.telemetry.map(|t| t.joint.id()).unwrap_or(0));
    })
    .unwrap();
    assert_eq!(seen, vec![1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6]);
}

#[test]
fn dashboard_serves_page_model_and_events() {
    use so101::report::dashboard::{serve, Hub};
    use std::io::{Read, Write};
    let hub = Hub::new();
    hub.set_hello(r#"{"k":"hello"}"#.to_string());
    let addr = serve(hub.clone(), "127.0.0.1:0").unwrap();
    let get = |path: &str, limit: usize| {
        let mut s = std::net::TcpStream::connect(addr).unwrap();
        s.set_read_timeout(Some(Duration::from_millis(500))).unwrap();
        write!(s, "GET {path} HTTP/1.1\r\nHost: x\r\n\r\n").unwrap();
        let mut buf = vec![0u8; limit];
        let mut got = 0;
        while got < limit {
            match s.read(&mut buf[got..]) {
                Ok(0) | Err(_) => break,
                Ok(n) => got += n,
            }
        }
        String::from_utf8_lossy(&buf[..got]).to_string()
    };
    assert!(get("/?autostart", 1 << 20).contains("<canvas id=\"gl\">"));
    let model = get("/model.json", 1 << 16);
    assert!(model.contains("\"shoulder_pan\"") && model.contains("sts3215_03a_v1.stl"));
    let events = get("/events", 512);
    assert!(events.contains("text/event-stream") && events.contains(r#"data: {"k":"hello"}"#));
}

#[test]
fn check_centred_rejects_a_middle_pose_held_at_one_end() {
    use so101::control::calibrate::{check_centred, Sweep};
    let good = Sweep { min: [900, 900, 900, 900, 0, 1400], max: [3200, 3200, 3200, 3200, 4095, 2700] };
    assert!(check_centred(&good).is_ok());
    // Elbow and wrist flex centred near their top end, lift wrapped: today's leader sweep.
    let bad = Sweep { min: [804, 1, -78, 10, 173, 1344], max: [3417, 4087, 2124, 2343, 3973, 2544] };
    let msg = check_centred(&bad).unwrap_err().to_string();
    assert!(msg.contains("shoulder_lift wrapped"), "{msg}");
    assert!(msg.contains("elbow_flex held at"), "{msg}");
    assert!(msg.contains("wrist_flex held at"), "{msg}");
    assert!(!msg.contains("shoulder_pan"), "{msg}");
}

#[test]
fn the_step_limit_does_not_throttle_the_follower_below_the_cap() {
    // The leader sits 800 ticks away on the pan joint. With a 40-tick step at
    // 200 Hz the command may move 8000 ticks/s, so the follower should run at
    // its own top speed (3100 ticks/s in the sim), not crawl behind a goal
    // pinned 40 ticks ahead of it.
    let hand: so101::device::sim::Hand = Arc::new(|id: u8, _t: f64| Some(if id == 1 { 2848.0 } else { 2048.0 }));
    let mut leader = sim_arm(SimulatedBus::arm("leader").with_hand(hand), "leader");
    let mut follower = sim_arm(SimulatedBus::arm("follower"), "follower");
    let options = TeleopOptions { hz: 200.0, max_step_ticks: Some(40), max_ticks: Some(60), telemetry: false, lead: [Duration::ZERO; COUNT] };
    let mut last = None;
    teleop::run(&mut leader, &mut follower, options, Arc::new(AtomicBool::new(false)), |r| last = Some(r.follower_ticks)).unwrap();
    let covered = last.unwrap()[0] - 2048;
    assert!(covered >= 600, "follower covered only {covered} of 800 ticks in 0.3 s");
}

#[test]
fn an_overloaded_gripper_is_released_and_teleop_keeps_running() {
    // The follower's jaws meet an object at tick 1900 while the leader asks for
    // 1600; after 0.7 s the leader opens to 2600. The capped gripper overloads.
    // Teleop must keep ticking, hold the gripper where it stopped, then follow
    // the leader open again.
    let hand: so101::device::sim::Hand =
        Arc::new(|id: u8, t: f64| Some(if id == 6 { if t < 0.7 { 1600.0 } else { 2600.0 } } else { 2048.0 }));
    let object: so101::device::sim::Obstacle = Arc::new(|id: u8, _t: f64| (id == 6).then_some(1900.0));
    let mut leader = sim_arm(SimulatedBus::arm("leader").with_hand(hand), "leader");
    let mut follower = sim_arm(SimulatedBus::arm("follower").with_obstacle(object), "follower");
    follower.configure_follower().unwrap();
    let options = TeleopOptions { hz: 200.0, max_step_ticks: Some(40), max_ticks: Some(260), telemetry: false, lead: [Duration::ZERO; COUNT] };
    let mut reports = Vec::new();
    let stats = teleop::run(&mut leader, &mut follower, options, Arc::new(AtomicBool::new(false)), |r| reports.push(r.clone())).unwrap();
    let g = Joint::Gripper.index();
    assert_eq!(stats.errors, 0, "an overload flag must not fail ticks");
    let held: Vec<_> = reports.iter().filter(|r| r.released[g]).collect();
    assert!(!held.is_empty(), "the gripper never tripped");
    // Held at the contact point with a small fixed squeeze (30 ticks), not driven at the leader's 1600.
    assert!(held.iter().skip(10).all(|r| (1860..=1900).contains(&r.goal_ticks_sent[g])), "held goal must stop pushing into the object");
    let last = reports.last().unwrap();
    assert!(!last.released[g], "never resumed after the leader opened");
    assert!(last.follower_ticks[g] > 2300, "did not follow the leader open: {}", last.follower_ticks[g]);
}

#[test]
fn configure_writes_gains_max_acceleration_and_clears_the_wrap_bit() {
    use so101::robot::arm::FollowerGains;
    let mut arm = sim_arm(SimulatedBus::arm("follower"), "follower");
    arm.configure_follower_with(FollowerGains { p: 28, d: 32, grip_torque: 800 }).unwrap();
    assert_eq!(arm.read_register(Joint::Gripper, registers::MAX_TORQUE).unwrap(), 800);
    assert_eq!(arm.read_register(Joint::Gripper, registers::TORQUE_LIMIT).unwrap(), 800);
    assert_eq!(arm.read_register(Joint::Gripper, registers::PROTECTION_CURRENT).unwrap(), 250, "stall protection stays on");
    assert_eq!(arm.read_register(Joint::ElbowFlex, registers::P_COEFFICIENT).unwrap(), 28);
    assert_eq!(arm.read_register(Joint::ElbowFlex, registers::MAXIMUM_ACCELERATION).unwrap(), 254);
    assert_eq!(arm.read_register(Joint::ElbowFlex, registers::LOCK).unwrap(), 1);
    assert_eq!(arm.configure_readings().unwrap(), 0, "factory Phase 12 has bit 4 clear");
}

#[test]
fn lead_closes_the_gap_to_a_moving_leader() {
    // Leader pan sweeps at a constant 1200 ticks/s. Commanding 60 ms ahead
    // should leave the follower closer to the leader than no lead does.
    let gap = |lead_ms: u64| {
        let hand: so101::device::sim::Hand = Arc::new(|id: u8, t: f64| Some(if id == 1 { 1200.0 + 1200.0 * t.min(1.5) } else { 2048.0 }));
        let mut leader = sim_arm(SimulatedBus::arm("leader").with_hand(hand), "leader");
        let mut follower = sim_arm(SimulatedBus::arm("follower"), "follower");
        let options = TeleopOptions { hz: 100.0, max_step_ticks: Some(80), max_ticks: Some(80), telemetry: false, lead: [Duration::from_millis(lead_ms); COUNT] };
        let mut errs = Vec::new();
        teleop::run(&mut leader, &mut follower, options, Arc::new(AtomicBool::new(false)), |r| errs.push((r.leader_ticks[0] - r.follower_ticks[0]).abs())).unwrap();
        errs[40..].iter().sum::<i32>() as f64 / 40.0
    };
    let (none, led) = (gap(0), gap(60));
    assert!(led < none * 0.6, "lead did not help: {none:.0} -> {led:.0} ticks");
}

// A transport with the timing measured on hardware: a write that commits to
// EEPROM or the factory area leaves that servo busy for ~40 ms, and anything it
// says meanwhile (the ack included) arrives only when it's done. Other replies
// arrive 0.3 ms after the request. A reply that misses its window stays in
// flight and lands in whatever exchange is open then, as on a real port.
struct SlowEepromAcks {
    inner: SimulatedBus,
    busy_until: std::collections::HashMap<u8, std::time::Instant>,
    in_flight: Vec<(std::time::Instant, Vec<u8>)>,
    arrived: std::collections::VecDeque<u8>,
}

impl SlowEepromAcks {
    const COMMIT: Duration = Duration::from_millis(40);
    const TURNAROUND: Duration = Duration::from_micros(300);

    fn new() -> Self {
        Self { inner: SimulatedBus::arm("bus"), busy_until: Default::default(), in_flight: Vec::new(), arrived: Default::default() }
    }

    fn land_due(&mut self) {
        let now = std::time::Instant::now();
        self.in_flight.sort_by_key(|(t, _)| *t);
        while self.in_flight.first().is_some_and(|(t, _)| *t <= now) {
            let (_, bytes) = self.in_flight.remove(0);
            self.arrived.extend(bytes);
        }
    }
}

impl so101::wire::transport::Transport for SlowEepromAcks {
    fn send(&mut self, bytes: &[u8]) -> Result<(), so101::wire::transport::TransportError> {
        let now = std::time::Instant::now();
        self.inner.send(bytes)?;
        let (id, instr, addr) = (bytes[2], bytes[4], bytes[5]);
        let mut out = Vec::new();
        let mut b = [0u8; 1];
        while self.inner.receive_exact(&mut b, Duration::ZERO).is_ok() {
            out.push(b[0]);
        }
        let mut rest = &out[..];
        while let Ok((frame, used)) = so101::wire::protocol::parse_frame(rest) {
            let busy = self.busy_until.get(&frame.id).copied().filter(|t| *t > now);
            self.in_flight.push((busy.unwrap_or(now + Self::TURNAROUND), rest[..used].to_vec()));
            rest = &rest[used..];
        }
        if instr == 0x03 && (addr < 40 || addr >= 80) {
            let done = now + Self::COMMIT;
            self.busy_until.insert(id, done);
            for (t, bytes) in &mut self.in_flight {
                if bytes[2] == id {
                    *t = done;
                }
            }
        }
        Ok(())
    }

    fn receive_exact(&mut self, buf: &mut [u8], timeout: Duration) -> Result<(), so101::wire::transport::TransportError> {
        let deadline = std::time::Instant::now() + timeout;
        let mut got = 0;
        while got < buf.len() {
            self.land_due();
            match self.arrived.pop_front() {
                Some(b) => {
                    buf[got] = b;
                    got += 1;
                }
                None if std::time::Instant::now() >= deadline => {
                    return Err(so101::wire::transport::TransportError::Timeout { wanted: buf.len(), got })
                }
                None => std::thread::sleep(Duration::from_micros(100)),
            }
        }
        Ok(())
    }

    // Like a real port: only bytes that have already arrived are thrown away.
    fn discard_input(&mut self) -> Result<(), so101::wire::transport::TransportError> {
        self.land_due();
        self.arrived.clear();
        Ok(())
    }

    fn name(&self) -> &str {
        "slow-eeprom"
    }
}

#[test]
fn a_late_eeprom_ack_does_not_leak_into_the_next_transaction() {
    // The sequence that failed on hardware: clear the wrap bit on servo 3, then
    // on servo 4. Servo 3's late ack landed in servo 4's exchange.
    let mut bus = Bus::new(Box::new(SlowEepromAcks::new()));
    bus.write_unlocked(3, registers::PHASE, 12).unwrap();
    bus.write_unlocked(4, registers::PHASE, 12).unwrap();
    assert_eq!(bus.read(5, registers::MODEL_NUMBER).unwrap(), 777);
    for id in [3, 4] {
        assert_eq!(bus.read(id, registers::PHASE).unwrap(), 12);
        assert_eq!(bus.read(id, registers::LOCK).unwrap(), 1, "servo {id} relocked");
    }
}

#[test]
fn lead_ignores_tremor_so_a_still_hand_gives_a_still_follower() {
    // The leader's pan trembles 4 ticks (0.35 deg) at 8 Hz: a hand held still.
    // With 70 ms of lead the commanded goal must not shake more than without.
    let spread = |lead_ms: u64| {
        let hand: so101::device::sim::Hand =
            Arc::new(|id: u8, t: f64| Some(if id == 1 { 2048.0 + 4.0 * (t * 8.0 * std::f64::consts::TAU).sin() } else { 2048.0 }));
        let mut leader = sim_arm(SimulatedBus::arm("leader").with_hand(hand), "leader");
        let mut follower = sim_arm(SimulatedBus::arm("follower"), "follower");
        let options = TeleopOptions { hz: 100.0, max_step_ticks: Some(40), max_ticks: Some(100), telemetry: false, lead: [Duration::from_millis(lead_ms); COUNT] };
        let mut goals = Vec::new();
        teleop::run(&mut leader, &mut follower, options, Arc::new(AtomicBool::new(false)), |r| goals.push(r.goal_ticks_sent[0])).unwrap();
        let g = &goals[20..];
        (*g.iter().max().unwrap() - *g.iter().min().unwrap()) as f64
    };
    let (still, led) = (spread(0), spread(70));
    assert!(led <= still + 1.0, "lead amplified tremor: goal spread {still} -> {led} ticks");
}

#[test]
fn a_blocked_joint_is_held_before_any_servo_protection_trips() {
    // Pan is blocked at 2000 while the leader asks for 1900: a 100-tick gap,
    // well under saturation, so the servo never raises a flag. Teleop must
    // notice the stall by itself and hold at the contact point.
    let hand: so101::device::sim::Hand = Arc::new(|id: u8, _t: f64| Some(if id == 1 { 1900.0 } else { 2048.0 }));
    let wall: so101::device::sim::Obstacle = Arc::new(|id: u8, _t: f64| (id == 1).then_some(2000.0));
    let mut leader = sim_arm(SimulatedBus::arm("leader").with_hand(hand), "leader");
    let mut follower = sim_arm(SimulatedBus::arm("follower").with_obstacle(wall), "follower");
    let options = TeleopOptions { hz: 200.0, max_step_ticks: Some(40), max_ticks: Some(120), telemetry: false, lead: [Duration::ZERO; COUNT] };
    let mut reports = Vec::new();
    teleop::run(&mut leader, &mut follower, options, Arc::new(AtomicBool::new(false)), |r| reports.push(r.clone())).unwrap();
    assert!(reports.iter().all(|r| r.flags[0].is_healthy()), "the servo should never have flagged");
    let last = reports.last().unwrap();
    assert!(last.released[0], "stall not detected");
    assert!((last.goal_ticks_sent[0] - 2000).abs() <= 2, "held goal {} should sit at the contact point", last.goal_ticks_sent[0]);
}

// A transport that goes silent after a number of sends: a follower bus dying mid-run.
struct DiesAfter {
    inner: SimulatedBus,
    sends_left: usize,
}

impl so101::wire::transport::Transport for DiesAfter {
    fn send(&mut self, bytes: &[u8]) -> Result<(), so101::wire::transport::TransportError> {
        if self.sends_left == 0 {
            return Ok(());
        }
        self.sends_left -= 1;
        self.inner.send(bytes)
    }
    fn receive_exact(&mut self, buf: &mut [u8], timeout: Duration) -> Result<(), so101::wire::transport::TransportError> {
        if self.sends_left == 0 {
            return Err(so101::wire::transport::TransportError::Timeout { wanted: buf.len(), got: 0 });
        }
        self.inner.receive_exact(buf, timeout)
    }
    fn discard_input(&mut self) -> Result<(), so101::wire::transport::TransportError> {
        self.inner.discard_input()
    }
    fn name(&self) -> &str {
        "dies-after"
    }
}

#[test]
fn a_follower_that_keeps_failing_stops_teleop_instead_of_freezing() {
    let mut leader = sim_arm(SimulatedBus::waving_arm("leader"), "leader");
    let dying = DiesAfter { inner: SimulatedBus::arm("follower"), sends_left: 60 };
    let mut follower = Arm::new("follower", Bus::new(Box::new(dying)), ArmCalibration::identity("follower"));
    let options = TeleopOptions { hz: 200.0, max_step_ticks: Some(40), max_ticks: Some(2000), telemetry: false, lead: [Duration::ZERO; COUNT] };
    let mut errors = 0;
    let result = teleop::run(&mut leader, &mut follower, options, Arc::new(AtomicBool::new(false)), |r| {
        if r.error.is_some() {
            errors += 1
        }
    });
    let msg = result.unwrap_err().to_string();
    assert!(msg.contains("stopping with torque off"), "{msg}");
    assert!(errors <= 100, "ran {errors} failed ticks; the fail-safe is 0.5 s = 100 ticks at 200 Hz");
}

// Flags the gripper's first few replies with OVERCURRENT, as a real servo does
// for a moment after torque comes back on following an earlier trip.
struct StaleGripperFlags {
    inner: SimulatedBus,
    flagged_left: usize,
}

impl so101::wire::transport::Transport for StaleGripperFlags {
    fn send(&mut self, bytes: &[u8]) -> Result<(), so101::wire::transport::TransportError> {
        self.inner.send(bytes)
    }
    fn receive_exact(&mut self, buf: &mut [u8], timeout: Duration) -> Result<(), so101::wire::transport::TransportError> {
        self.inner.receive_exact(buf, timeout)?;
        if self.flagged_left > 0 && buf.len() >= 6 && buf[2] == 6 {
            self.flagged_left -= 1;
            buf[4] |= 0x08;
            let n = buf.len();
            buf[n - 1] = !buf[2..n - 1].iter().fold(0u8, |a, b| a.wrapping_add(*b));
        }
        Ok(())
    }
    fn discard_input(&mut self) -> Result<(), so101::wire::transport::TransportError> {
        self.inner.discard_input()
    }
    fn name(&self) -> &str {
        "stale-gripper-flags"
    }
}

#[test]
fn stale_flags_at_startup_do_not_lock_the_gripper() {
    // Hardware, grab1.jsonl: overcurrent left over from the previous run showed
    // in the first 0.09 s, the first tick latched a hold, and the gripper stayed
    // shut for the whole run. A free gripper must follow the leader open.
    let hand: so101::device::sim::Hand = Arc::new(|id: u8, _t: f64| Some(if id == 6 { 2600.0 } else { 2048.0 }));
    let mut leader = sim_arm(SimulatedBus::arm("leader").with_hand(hand), "leader");
    let stale = StaleGripperFlags { inner: SimulatedBus::arm("follower"), flagged_left: 8 };
    let mut follower = Arm::new("follower", Bus::new(Box::new(stale)), ArmCalibration::identity("follower"));
    let options = TeleopOptions { hz: 200.0, max_step_ticks: Some(40), max_ticks: Some(200), telemetry: false, lead: [Duration::ZERO; COUNT] };
    let mut reports = Vec::new();
    teleop::run(&mut leader, &mut follower, options, Arc::new(AtomicBool::new(false)), |r| reports.push(r.clone())).unwrap();
    let g = Joint::Gripper.index();
    let last = reports.last().unwrap();
    assert!(!last.released[g], "gripper latched in a hold it never earned");
    assert!(last.follower_ticks[g] > 2500, "gripper did not open: {}", last.follower_ticks[g]);
}

// Flags the gripper's replies with OVERLOAD between two reply counts: the
// servo's protection staying on across the moment the leader opens.
struct GripperFlagWindow {
    inner: SimulatedBus,
    seen: usize,
    from: usize,
    to: usize,
}

impl so101::wire::transport::Transport for GripperFlagWindow {
    fn send(&mut self, bytes: &[u8]) -> Result<(), so101::wire::transport::TransportError> {
        self.inner.send(bytes)
    }
    fn receive_exact(&mut self, buf: &mut [u8], timeout: Duration) -> Result<(), so101::wire::transport::TransportError> {
        self.inner.receive_exact(buf, timeout)?;
        if buf.len() >= 6 && buf[2] == 6 {
            self.seen += 1;
            if (self.from..self.to).contains(&self.seen) {
                buf[4] |= 0x20;
                let n = buf.len();
                buf[n - 1] = !buf[2..n - 1].iter().fold(0u8, |a, b| a.wrapping_add(*b));
            }
        }
        Ok(())
    }
    fn discard_input(&mut self) -> Result<(), so101::wire::transport::TransportError> {
        self.inner.discard_input()
    }
    fn name(&self) -> &str {
        "gripper-flag-window"
    }
}

#[test]
fn an_overload_flag_left_over_from_a_hold_does_not_trap_the_gripper_open() {
    // Hardware, grab1.jsonl (second run): holding an object keeps the servo's
    // overload flag on; opening the leader released the hold, the flag was still
    // set for a few ticks while the jaws had not started moving, and a second
    // hold latched ~30 ticks open. It stayed until the leader closed again.
    let hand: so101::device::sim::Hand =
        Arc::new(|id: u8, t: f64| Some(if id == 6 { if t < 0.6 { 1600.0 } else { 2600.0 } } else { 2048.0 }));
    let object: so101::device::sim::Obstacle = Arc::new(|id: u8, _t: f64| (id == 6).then_some(1900.0));
    let mut leader = sim_arm(SimulatedBus::arm("leader").with_hand(hand), "leader");
    // Flag on from the grip until ~0.15 s after the leader opens.
    let window = GripperFlagWindow { inner: SimulatedBus::arm("follower").with_obstacle(object), seen: 0, from: 60, to: 150 };
    let mut follower = Arm::new("follower", Bus::new(Box::new(window)), ArmCalibration::identity("follower"));
    let options = TeleopOptions { hz: 200.0, max_step_ticks: Some(40), max_ticks: Some(260), telemetry: false, lead: [Duration::ZERO; COUNT] };
    let mut reports = Vec::new();
    teleop::run(&mut leader, &mut follower, options, Arc::new(AtomicBool::new(false)), |r| reports.push(r.clone())).unwrap();
    let g = Joint::Gripper.index();
    assert!(reports.iter().any(|r| r.released[g]), "the grip never held");
    let last = reports.last().unwrap();
    assert!(!last.released[g], "gripper trapped in a hold after the leader opened");
    assert!(last.follower_ticks[g] > 2300, "gripper did not open: {}", last.follower_ticks[g]);
}
