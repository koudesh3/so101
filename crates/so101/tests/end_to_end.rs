//! Everything above the transport, exercised against the simulator.

use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use std::time::Duration;

use so101::arm::servos;
use so101::arm::{
    read_trace, sweep, ActuatorStatus, ByteTrace, Calibration, Counts, Error, Fault, Joint, Limit, ServoArm, Servos,
    SimulatedServos, Traced, IDS, JOINT_COUNT,
};
use so101::clock::{Clock, ReplayClock, ReplaySpeed, Timestamp};
use so101::control::{
    run, ArmRole, Event, EventBus, Message, Pilot, RunOptions, RunStats, Source, Teleop, Tick, LATE_AFTER,
};

/// One teleop step's events, by what they are.
struct Step {
    leader: Counts,
    follower: Counts,
    sent: Counts,
    tick: Tick,
    actuators: Vec<ActuatorStatus>,
}

// A transport with the timing measured on hardware: a write that commits to
// EEPROM or the factory area leaves that servo busy for ~40 ms, and anything it
// says meanwhile (the ack included) arrives only when it's done. Other replies
// arrive 0.3 ms after the request. A reply that misses its window stays in
// flight and lands in whatever exchange is open then, as on a real port.
struct SlowEepromAcks {
    inner: SimulatedServos,
    busy_until: std::collections::HashMap<u8, std::time::Instant>,
    in_flight: Vec<(std::time::Instant, Vec<u8>)>,
    arrived: std::collections::VecDeque<u8>,
}

// A transport that goes silent after a number of sends: a follower line dying mid-run.
struct DiesAfter {
    inner: SimulatedServos,
    sends_left: usize,
}

// Flags the gripper's first few replies with OVERCURRENT, as a real servo does
// for a moment after torque comes back on following an earlier trip.
struct StaleGripperFlags {
    inner: SimulatedServos,
    flagged_left: usize,
}

// Flags the gripper's replies with OVERLOAD between two reply counts: the
// servo's protection staying on across the moment the leader opens.
struct GripperFlagWindow {
    inner: SimulatedServos,
    seen: usize,
    from: usize,
    to: usize,
}

impl Step {
    fn of(events: &[Event]) -> Step {
        let (mut leader, mut follower, mut sent, mut tick, mut actuators) = (None, None, None, None, Vec::new());
        for event in events {
            match &event.message {
                Message::JointState(ArmRole::Leader, s) => leader = s.counts,
                Message::JointState(ArmRole::Follower, s) => follower = s.counts,
                Message::JointTarget(ArmRole::Follower, t) => sent = t.counts,
                Message::Tick(t) => tick = Some(t.clone()),
                Message::Actuator(_, a) => actuators.push(a.clone()),
                _ => {}
            }
        }
        Step {
            leader: leader.unwrap(),
            follower: follower.unwrap(),
            sent: sent.unwrap(),
            tick: tick.unwrap(),
            actuators,
        }
    }

    fn held(&self, joint: Joint) -> bool {
        self.tick.clipped.iter().any(|c| c.joint == joint && c.limit == Limit::Stall)
    }
}

impl SlowEepromAcks {
    const COMMIT: Duration = Duration::from_millis(40);
    const TURNAROUND: Duration = Duration::from_micros(300);

    fn new() -> Self {
        Self {
            inner: SimulatedServos::arm("servos"),
            busy_until: Default::default(),
            in_flight: Vec::new(),
            arrived: Default::default(),
        }
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

impl so101::arm::Transport for SlowEepromAcks {
    fn send(&mut self, bytes: &[u8]) -> so101::arm::Result<()> {
        let now = std::time::Instant::now();
        self.inner.send(bytes)?;
        let (id, instr, addr) = (bytes[2], bytes[4], bytes[5]);
        let mut out = Vec::new();
        let mut b = [0u8; 1];
        while self.inner.receive_exact(&mut b, Duration::ZERO).is_ok() {
            out.push(b[0]);
        }
        let mut rest = &out[..];
        while let Ok((frame, used)) = so101::arm::servos::parse_packet(rest) {
            let busy = self.busy_until.get(&frame.id).copied().filter(|t| *t > now);
            self.in_flight.push((busy.unwrap_or(now + Self::TURNAROUND), rest[..used].to_vec()));
            rest = &rest[used..];
        }
        if instr == 0x03 && !(40..80).contains(&addr) {
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

    fn receive_exact(&mut self, buf: &mut [u8], timeout: Duration) -> so101::arm::Result<()> {
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
                    return Err(so101::arm::Error::Timeout { wanted: buf.len(), got })
                }
                None => std::thread::sleep(Duration::from_micros(100)),
            }
        }
        Ok(())
    }

    // Like a real port: only bytes that have already arrived are thrown away.
    fn discard_input(&mut self) -> so101::arm::Result<()> {
        self.land_due();
        self.arrived.clear();
        Ok(())
    }

    fn name(&self) -> &str {
        "slow-eeprom"
    }
}

impl so101::arm::Transport for DiesAfter {
    fn send(&mut self, bytes: &[u8]) -> so101::arm::Result<()> {
        if self.sends_left == 0 {
            return Ok(());
        }
        self.sends_left -= 1;
        self.inner.send(bytes)
    }
    fn receive_exact(&mut self, buf: &mut [u8], timeout: Duration) -> so101::arm::Result<()> {
        if self.sends_left == 0 {
            return Err(so101::arm::Error::Timeout { wanted: buf.len(), got: 0 });
        }
        self.inner.receive_exact(buf, timeout)
    }
    fn discard_input(&mut self) -> so101::arm::Result<()> {
        self.inner.discard_input()
    }
    fn name(&self) -> &str {
        "dies-after"
    }
}

impl so101::arm::Transport for StaleGripperFlags {
    fn send(&mut self, bytes: &[u8]) -> so101::arm::Result<()> {
        self.inner.send(bytes)
    }
    fn receive_exact(&mut self, buf: &mut [u8], timeout: Duration) -> so101::arm::Result<()> {
        self.inner.receive_exact(buf, timeout)?;
        if self.flagged_left > 0 && buf.len() >= 6 && buf[2] == 6 {
            self.flagged_left -= 1;
            buf[4] |= 0x08;
            let n = buf.len();
            buf[n - 1] = !buf[2..n - 1].iter().fold(0u8, |a, b| a.wrapping_add(*b));
        }
        Ok(())
    }
    fn discard_input(&mut self) -> so101::arm::Result<()> {
        self.inner.discard_input()
    }
    fn name(&self) -> &str {
        "stale-gripper-flags"
    }
}

impl so101::arm::Transport for GripperFlagWindow {
    fn send(&mut self, bytes: &[u8]) -> so101::arm::Result<()> {
        self.inner.send(bytes)
    }
    fn receive_exact(&mut self, buf: &mut [u8], timeout: Duration) -> so101::arm::Result<()> {
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
    fn discard_input(&mut self) -> so101::arm::Result<()> {
        self.inner.discard_input()
    }
    fn name(&self) -> &str {
        "gripper-flag-window"
    }
}

fn sim_bus(name: &str) -> Servos {
    Servos::new(Box::new(SimulatedServos::arm(name)))
}

struct Teleopped {
    result: so101::arm::Result<RunStats>,
    steps: Vec<Step>,
    errors: u64,
}

/// Runs a `Teleop` pilot on `follower` and groups what the loop published into steps.
fn run_teleop(
    leader: &mut ServoArm,
    follower: &mut ServoArm,
    options: RunOptions,
    lead: [Duration; JOINT_COUNT],
    clock: &Arc<ReplayClock>,
) -> Teleopped {
    let bus = EventBus::new();
    let events = bus.subscribe();
    let mut errors = 0;
    let result = Teleop::new(leader, lead).and_then(|mut pilot| {
        run(follower, &mut pilot, options, clock.clone(), &AtomicBool::new(false), &bus, |_| errors += 1)
    });
    drop(bus);
    let events: Vec<Event> = events.into_iter().collect();
    // Each cycle starts with the follower's state.
    let mut steps = Vec::new();
    let mut start = 0;
    for i in 1..=events.len() {
        if i == events.len() || matches!(events[i].message, Message::JointState(ArmRole::Follower, _)) {
            steps.push(Step::of(&events[start..i]));
            start = i;
        }
    }
    Teleopped { result, steps, errors }
}

/// A clock that only moves when something waits on it: the grid, or a test stepping time.
fn stepped() -> Arc<ReplayClock> {
    Arc::new(ReplayClock::new(Timestamp(0), ReplaySpeed::AsFastAsPossible))
}

fn advance(clock: &ReplayClock, by: Duration) {
    clock.sleep_until(clock.now() + by);
}

fn sim_arm(sim: SimulatedServos, label: &str, clock: &Arc<ReplayClock>) -> ServoArm {
    ServoArm::new(label, Servos::new(Box::new(sim.with_clock(clock.clone()))), Calibration::identity(label))
        .with_clock(clock.clone())
}

#[test]
fn ping_and_model_number() {
    let mut servos = sim_bus("servos");
    servos.ping(1).unwrap();
    let servo = servos.identify(1).unwrap();
    assert_eq!(servo.model, 777);
    assert!(servo.is_sts3215());
    assert!(matches!(servos.ping(9), Err(Error::NoReply { id: 9 })));
}

#[test]
fn scan_finds_exactly_six() {
    let mut servos = sim_bus("servos");
    assert_eq!(servos.scan(), vec![1, 2, 3, 4, 5, 6]);
}

#[test]
fn write_then_read_a_ram_register() {
    let mut servos = sim_bus("servos");
    servos.write(3, servos::ACCELERATION, 254).unwrap();
    assert_eq!(servos.read(3, servos::ACCELERATION).unwrap(), 254);
}

#[test]
fn eeprom_write_needs_unlock() {
    let mut servos = sim_bus("servos");
    servos.write(1, servos::P_COEFFICIENT, 16).unwrap();
    assert_eq!(servos.read(1, servos::P_COEFFICIENT).unwrap(), 32);
    servos.write_eeprom(1, servos::P_COEFFICIENT, 16).unwrap();
    assert_eq!(servos.read(1, servos::P_COEFFICIENT).unwrap(), 16);
    assert_eq!(servos.read(1, servos::LOCK).unwrap(), 1);
}

#[test]
fn read_only_register_is_refused_before_touching_the_wire() {
    let mut servos = sim_bus("servos");
    assert!(matches!(servos.write(1, servos::PRESENT_POSITION, 0), Err(Error::Refused(_))));
}

#[test]
fn servo_tracks_goal_within_speed_limit() {
    let clock = stepped();
    let sim = SimulatedServos::new("servos", &[1], clock.clone());
    let mut servos = Servos::new(Box::new(sim));
    servos.write(1, servos::TORQUE_ENABLE, 1).unwrap();
    servos.write(1, servos::GOAL_POSITION, 3048).unwrap();

    advance(&clock, Duration::from_millis(100));
    let after_100ms = servos.read(1, servos::PRESENT_POSITION).unwrap();
    assert!(after_100ms > 2048 && after_100ms <= 2048 + 310, "{after_100ms}");

    advance(&clock, Duration::from_secs(2));
    let settled = servos.read(1, servos::PRESENT_POSITION).unwrap();
    assert!((settled - 3048).abs() <= 2, "{settled}");
    assert_eq!(servos.read(1, servos::MOVING).unwrap(), 0);
}

#[test]
fn sync_read_returns_in_id_order_and_fails_on_a_missing_servo() {
    let mut sim = SimulatedServos::arm("servos");
    sim.servo_mut(4).unwrap().actual_counts = 1000.0;
    let mut servos = Servos::new(Box::new(sim));
    let v = servos.sync_read(&[1, 4, 6], servos::PRESENT_POSITION).unwrap();
    assert_eq!(v, vec![2048, 1000, 2048]);

    let mut sim = SimulatedServos::arm("servos");
    sim.silenced_ids.push(3);
    let mut servos = Servos::new(Box::new(sim));
    let err = servos.sync_read(&IDS, servos::PRESENT_POSITION).unwrap_err();
    assert!(matches!(err, Error::WrongId { expected: 3, got: 4 }), "{err}");
}

#[test]
fn check_present_rejects_a_wrong_model() {
    let clock = stepped();
    let mut sim = SimulatedServos::arm("servos").with_clock(clock.clone());
    sim.servo_mut(2).unwrap().set(servos::MODEL_NUMBER, 1234);
    let mut arm = sim_arm(sim, "a", &clock);
    let err = arm.check_present().unwrap_err();
    assert!(matches!(err, Error::UnexpectedModel { id: 2, model: 1234 }), "{err}");
}

#[test]
fn recentre_makes_every_joint_read_centre_without_moving() {
    let clock = stepped();
    let mut sim = SimulatedServos::arm("servos").with_clock(clock.clone());
    for (i, s) in sim.servos.iter_mut().enumerate() {
        s.actual_counts = 1500.0 + i as f64 * 100.0;
    }
    let mut arm = sim_arm(sim, "a", &clock);
    let offsets = arm.recentre().unwrap();
    assert_eq!(offsets, [-548, -448, -348, -248, -148, -48]);
    assert_eq!(arm.read_counts().unwrap(), [2048; JOINT_COUNT]);
}

#[test]
fn sweep_records_the_range_the_hand_covered() {
    let clock = stepped();
    let mut arm = sim_arm(SimulatedServos::waving_arm("servos").with_clock(clock.clone()), "a", &clock);
    arm.torque_off().unwrap();
    let mut polls = 0;
    let sweep = sweep(&mut arm, 200.0, |_| {
        polls += 1;
        polls < 60
    })
    .unwrap();
    for joint in Joint::ALL {
        let (lo, hi) = (sweep.min[joint.index()], sweep.max[joint.index()]);
        assert!(hi > lo, "{} did not move: {lo}..{hi}", joint.name());
        assert!(lo >= 0 && hi <= 4095);
    }
    let cal = Calibration::from_sweep("a", [0; JOINT_COUNT], sweep.min, sweep.max);
    assert!(cal.check_travel(5).is_ok());
    assert!(cal.check_travel(100_000).is_err());
}

#[test]
fn teleop_follows_the_leader_and_logs_every_byte() {
    let clock = stepped();
    let dir = std::env::temp_dir().join(format!("so101-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let trace_path = dir.join("bytes.jsonl");
    let log = ByteTrace::create(&trace_path).unwrap();

    let leader_sim =
        Traced::new(SimulatedServos::waving_arm("leader").with_clock(clock.clone()), "leader", log.clone());
    let follower_sim = Traced::new(SimulatedServos::arm("follower").with_clock(clock.clone()), "follower", log.clone());
    let mut leader = ServoArm::new("leader", Servos::new(Box::new(leader_sim)), Calibration::identity("leader"))
        .with_clock(clock.clone());
    let mut follower =
        ServoArm::new("follower", Servos::new(Box::new(follower_sim)), Calibration::identity("follower"))
            .with_clock(clock.clone());

    leader.check_present().unwrap();
    follower.check_present().unwrap();
    follower.configure_follower().unwrap();
    assert_eq!(follower.read_register(Joint::ShoulderPan, servos::P_COEFFICIENT).unwrap(), 16);

    let options = RunOptions { hz: 100.0, max_ticks: Some(120), read_actuators: false };

    let lead = [Duration::ZERO; JOINT_COUNT];
    follower.set_step_limit(Some(200));
    let run = run_teleop(&mut leader, &mut follower, options, lead, &clock);
    let stats = run.result.unwrap();
    let reports = run.steps;
    log.flush();

    assert_eq!(stats.ticks, 120);
    assert_eq!(stats.errors, 0);
    assert_eq!((stats.skipped, stats.late), (0, 0), "on a stepped clock every cycle starts on the grid");

    let last = reports.last().unwrap();
    for joint in Joint::ALL {
        let i = joint.index();
        let gap = (last.leader[i] - last.follower[i]).abs();
        assert!(gap < 60, "{}: leader {} follower {} (gap {gap})", joint.name(), last.leader[i], last.follower[i]);
    }
    // The step limit bounds how fast the command moves, starting from where the follower was.
    for i in 0..JOINT_COUNT {
        assert!((reports[0].sent[i] - reports[0].follower[i]).abs() <= 200);
    }
    for pair in reports.windows(2) {
        for i in 0..JOINT_COUNT {
            assert!((pair[1].sent[i] - pair[0].sent[i]).abs() <= 200);
        }
    }
    assert_eq!(follower.read_register(Joint::ShoulderPan, servos::TORQUE_ENABLE).unwrap(), 0);

    let records = read_trace(&trace_path).unwrap();
    assert!(records.len() > 120 * 4, "only {} log records", records.len());
    assert!(records.windows(2).all(|w| w[0].t_us <= w[1].t_us), "log is not time-ordered");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn a_silent_follower_servo_fails_teleop_before_the_loop_starts() {
    let clock = stepped();
    let mut follower_sim = SimulatedServos::arm("follower").with_clock(clock.clone());
    follower_sim.silenced_ids.push(2);
    let mut leader = sim_arm(SimulatedServos::waving_arm("leader").with_clock(clock.clone()), "leader", &clock);
    let mut follower = sim_arm(follower_sim, "follower", &clock);
    let options = RunOptions { hz: 200.0, max_ticks: Some(10), read_actuators: false };
    let lead = [Duration::ZERO; JOINT_COUNT];
    follower.set_step_limit(Some(200));
    let run = run_teleop(&mut leader, &mut follower, options, lead, &clock);
    let result = run.result;
    let errors = run.errors;
    assert!(result.is_err());
    assert_eq!(errors, 0);
}

#[test]
fn configure_follower_caps_the_gripper_only() {
    let clock = stepped();
    let mut arm = sim_arm(SimulatedServos::arm("follower").with_clock(clock.clone()), "follower", &clock);
    arm.configure_follower().unwrap();
    assert_eq!(arm.read_register(Joint::Gripper, servos::MAX_TORQUE).unwrap(), 500);
    assert_eq!(arm.read_register(Joint::Gripper, servos::PROTECTION_CURRENT).unwrap(), 250);
    assert_eq!(arm.read_register(Joint::Gripper, servos::OVERLOAD_TORQUE).unwrap(), 25);
    assert_eq!(arm.read_register(Joint::Gripper, servos::TORQUE_LIMIT).unwrap(), 500);
    assert_ne!(arm.read_register(Joint::ShoulderPan, servos::MAX_TORQUE).unwrap(), 500, "arm joints keep full torque");
}

#[test]
fn actuator_status_decodes_every_field_in_one_read() {
    let clock = stepped();
    let mut arm = sim_arm(SimulatedServos::arm("follower").with_clock(clock.clone()), "follower", &clock);
    let a = arm.read_actuator(Joint::ElbowFlex).unwrap();
    assert_eq!(a.joint, Joint::ElbowFlex);
    assert_eq!(a.temperature_c, Some(31.0));
    assert!((a.voltage_v.unwrap() - 7.4).abs() < 1e-4);
    assert_eq!(a.velocity_deg_s, Some(0.0));
    assert!(a.faults.is_empty());
}

#[test]
fn a_blocked_gripper_shows_load_and_no_speed() {
    use so101::arm::demo;
    let clock = stepped();
    let sim = SimulatedServos::new("follower", &IDS, clock.clone()).with_obstacle(demo::object());
    let mut arm = sim_arm(sim, "follower", &clock);
    arm.configure_follower().unwrap();
    arm.torque_on_holding().unwrap();
    // Into the window where the object sits in the jaws, then squeeze past it.
    advance(&clock, Duration::from_secs_f64(6.0));
    arm.write_goal(&[2048, 2048, 2048, 2048, 2048, 1600]).unwrap();
    for _ in 0..50 {
        advance(&clock, Duration::from_millis(20));
        let _ = arm.read_counts().unwrap();
    }
    let a = arm.read_actuator(Joint::Gripper).unwrap();
    let load = a.load_percent.unwrap().abs();
    assert!(load >= 40.0, "load {load}%");
    assert!(load <= 50.0, "the gripper cap should hold load to 50%, got {load}%");
    assert!(a.velocity_deg_s.unwrap().abs() < 0.5, "speed {:?}", a.velocity_deg_s);
    let at = arm.read_counts().unwrap()[Joint::Gripper.index()];
    assert!(at > 1900, "jaws stopped at the object, got {at}");
}

#[test]
fn teleop_reads_every_actuator_every_cycle() {
    let clock = stepped();
    let mut leader = sim_arm(SimulatedServos::waving_arm("leader").with_clock(clock.clone()), "leader", &clock);
    let mut follower = sim_arm(SimulatedServos::arm("follower").with_clock(clock.clone()), "follower", &clock);
    let options = RunOptions { hz: 200.0, max_ticks: Some(12), read_actuators: true };
    let lead = [Duration::ZERO; JOINT_COUNT];
    follower.set_step_limit(Some(200));
    let run = run_teleop(&mut leader, &mut follower, options, lead, &clock);
    run.result.unwrap();
    assert_eq!(run.steps.len(), 12);
    for step in &run.steps {
        let ids: Vec<u8> = step.actuators.iter().map(|a| a.joint.id()).collect();
        assert_eq!(ids, vec![1, 2, 3, 4, 5, 6], "all six, every cycle");
        assert!(step.actuators.iter().all(|a| a.voltage_v.is_some_and(|v| v > 0.0) && a.current_a.is_some()));
    }
}

#[test]
fn the_current_guard_slows_a_follower_drawing_too_much() {
    // The same fast moves, with and without current readings (the guard needs them).
    let follow = |read_actuators: bool| {
        let clock = stepped();
        // Shoulder pan jumps 800 counts every quarter second: fast moves that draw hard.
        let hand: so101::arm::Hand =
            Arc::new(|id: u8, t: f64| Some(if id == 1 && (t * 4.0) as u64 % 2 == 1 { 2448.0 } else { 1648.0 }));
        let mut leader =
            sim_arm(SimulatedServos::arm("leader").with_clock(clock.clone()).with_hand(hand), "leader", &clock);
        let mut follower = sim_arm(SimulatedServos::arm("follower").with_clock(clock.clone()), "follower", &clock);
        follower.set_step_limit(Some(200));
        let options = RunOptions { hz: 200.0, max_ticks: Some(400), read_actuators };
        let run = run_teleop(&mut leader, &mut follower, options, [Duration::ZERO; JOINT_COUNT], &clock);
        run.result.unwrap();
        let moves: Vec<i32> = run
            .steps
            .windows(2)
            .map(|w| (0..JOINT_COUNT).map(|i| (w[1].sent[i] - w[0].sent[i]).abs()).max().unwrap())
            .collect();
        (run.steps, moves)
    };
    let (steps, guarded) = follow(true);
    let (_, free) = follow(false);
    // The simulated pan servo draws several amps on each jump, well over budget.
    let total = |s: &Step| s.actuators.iter().filter_map(|a| a.current_a).sum::<f32>();
    assert!(steps.iter().any(|s| total(s) > 1.5), "the simulation should exceed the budget");
    let fastest = |m: &[i32]| m.iter().copied().max().unwrap();
    assert!(fastest(&free) > 100, "unguarded, a jump moves the goal up to the step limit");
    // It takes 50 ms over budget (10 writes at 200 Hz) to engage, so the first jump starts fast.
    assert!(fastest(&guarded[..60]) > 10, "the first jump starts before the guard engages");
    assert!(fastest(&guarded[60..]) <= 10, "once the guard is on, at most 10 counts a write");
}

#[test]
fn the_camera_guard_stops_the_wrist_short_of_the_arm_and_lets_it_back_out() {
    use so101::arm::{Axis, ClearanceMap};
    let clock = stepped();
    // The leader rolls its wrist from 0° to 90° and back over 2 s. In this map the camera's
    // clearance is 60 mm minus the roll in degrees, so 10 mm is reached at a roll of 50°.
    let hand: so101::arm::Hand = Arc::new(|id: u8, t: f64| {
        let roll_deg = 90.0 * (1.0 - (t - 1.0).abs()).max(0.0);
        Some(if id == 5 { 2048.0 + roll_deg * 4096.0 / 360.0 } else { 2048.0 })
    });
    let mut leader =
        sim_arm(SimulatedServos::arm("leader").with_clock(clock.clone()).with_hand(hand), "leader", &clock);
    let mut follower = sim_arm(SimulatedServos::arm("follower").with_clock(clock.clone()), "follower", &clock);
    let axes = [
        Axis { min: -90.0, step: 90.0, count: 3 },
        Axis { min: -90.0, step: 90.0, count: 3 },
        Axis { min: -180.0, step: 30.0, count: 13 },
    ];
    let mm = (0..3 * 3 * 13).map(|n| 60.0 - (-180.0 + 30.0 * (n % 13) as f32)).collect();
    let map = ClearanceMap::new([Joint::ElbowFlex, Joint::WristFlex, Joint::WristRoll], axes, mm).unwrap();
    follower.set_camera_guard(map, 10.0);
    follower.set_step_limit(Some(40));
    let options = RunOptions { hz: 100.0, max_ticks: Some(220), read_actuators: false };
    let run = run_teleop(&mut leader, &mut follower, options, [Duration::ZERO; JOINT_COUNT], &clock);
    run.result.unwrap();

    let roll = |counts: i32| (counts - 2048) as f32 * 360.0 / 4096.0;
    let r = Joint::WristRoll.index();
    let highest = run.steps.iter().map(|s| roll(s.sent[r])).fold(f32::MIN, f32::max);
    assert!(highest <= 50.5 && highest > 45.0, "stops at the 10 mm line (50°), got {highest:.1}°");
    let kept = run.steps.iter().filter(|s| s.tick.clipped.iter().any(|c| c.limit == Limit::Collision)).count();
    assert!(kept > 20, "the guard held while the leader was past the line ({kept} ticks)");
    let last = run.steps.last().unwrap();
    assert!(roll(last.sent[r]).abs() < 3.0, "followed the leader back out to 0°");
}

#[test]
fn the_committed_clearance_map_agrees_with_the_arm() {
    use so101::arm::{ClearanceMap, Pose};
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../data/camera_clearance.json");
    let map = ClearanceMap::load(&path).unwrap();
    // Follower poses read on 2026-11-23 (pan, lift, elbow, wrist flex, wrist roll, gripper).
    let at = |p: [f32; 6]| map.clearance_mm(&Pose(p));
    let bump1 = at([-21.4, -92.4, 74.2, 68.1, 45.9, 0.5]);
    let bump2 = at([-11.5, -92.5, 60.8, 93.1, 80.0, 0.5]);
    let rest = at([-21.4, -104.5, 96.1, -102.0, 17.5, 31.9]);
    let straight = at([0.0; 6]);
    assert!(bump1 < 10.0 && bump2 < 10.0, "the camera touched the arm at both: {bump1:.1}, {bump2:.1} mm");
    assert!(rest > 10.0 && straight > 10.0, "and cleared it here: {rest:.1}, {straight:.1} mm");
}

/// A pilot that takes extra time on chosen calls, as an OS hiccup would.
struct Slow<P: Pilot> {
    inner: P,
    clock: Arc<ReplayClock>,
    delays: Vec<(u64, Duration)>,
    calls: u64,
}

impl<P: Pilot> Pilot for Slow<P> {
    fn source(&self) -> Source {
        self.inner.source()
    }
    fn target(
        &mut self,
        now: Timestamp,
        follower: &so101::arm::JointState,
    ) -> so101::arm::Result<(so101::arm::Pose, Vec<Message>)> {
        if let Some(&(_, delay)) = self.delays.iter().find(|(call, _)| *call == self.calls) {
            advance(&self.clock, delay);
        }
        self.calls += 1;
        self.inner.target(now, follower)
    }
}

#[test]
fn a_late_cycle_and_a_skipped_one_are_each_counted() {
    let clock = stepped();
    let mut leader = sim_arm(SimulatedServos::waving_arm("leader").with_clock(clock.clone()), "leader", &clock);
    let mut follower = sim_arm(SimulatedServos::arm("follower").with_clock(clock.clone()), "follower", &clock);
    follower.set_step_limit(Some(200));
    let options = RunOptions { hz: 100.0, max_ticks: Some(10), read_actuators: false };
    // 10 ms cycles. Cycle 3's pilot takes 13 ms more, so cycle 4 starts a few ms late; cycle 6's
    // takes 25 ms more, so the grid skips one point and the next cycle is late too.
    let teleop = Teleop::new(&mut leader, [Duration::ZERO; JOINT_COUNT]).unwrap();
    let delays = vec![(3, Duration::from_millis(13)), (6, Duration::from_millis(25))];
    let mut pilot = Slow { inner: teleop, clock: clock.clone(), delays, calls: 0 };
    let stats =
        run(&mut follower, &mut pilot, options, clock.clone(), &AtomicBool::new(false), &EventBus::new(), |_| {})
            .unwrap();
    assert_eq!((stats.ticks, stats.errors), (10, 0));
    assert_eq!(stats.skipped, 1);
    assert_eq!(stats.late, 2);
    assert!(
        stats.worst_lateness > LATE_AFTER && stats.worst_lateness < Duration::from_millis(10),
        "{:?}",
        stats.worst_lateness
    );
}

#[test]
fn check_centred_rejects_a_middle_pose_held_at_one_end() {
    use so101::arm::{check_centred, Sweep};
    let good = Sweep { min: [900, 900, 900, 900, 0, 1400], max: [3200, 3200, 3200, 3200, 4095, 2700] };
    assert!(check_centred(&good).is_ok());
    // Elbow and wrist flex centred near their top end, lift wrapped: a real leader sweep.
    let bad = Sweep { min: [804, 1, -78, 10, 173, 1344], max: [3417, 4087, 2124, 2343, 3973, 2544] };
    let msg = check_centred(&bad).unwrap_err().to_string();
    assert!(msg.contains("shoulder_lift wrapped"), "{msg}");
    assert!(msg.contains("elbow_flex held at"), "{msg}");
    assert!(msg.contains("wrist_flex held at"), "{msg}");
    assert!(!msg.contains("shoulder_pan"), "{msg}");
}

#[test]
fn the_step_limit_does_not_throttle_the_follower_below_the_cap() {
    let clock = stepped();
    // The leader sits 800 counts away on the pan joint. With a 40-count step at
    // 200 Hz the command may move 8000 counts/s, so the follower should run at
    // its own top speed (3100 counts/s in the sim), not crawl behind a goal
    // pinned 40 counts ahead of it.
    let hand: so101::arm::Hand = Arc::new(|id: u8, _t: f64| Some(if id == 1 { 2848.0 } else { 2048.0 }));
    let mut leader =
        sim_arm(SimulatedServos::arm("leader").with_clock(clock.clone()).with_hand(hand), "leader", &clock);
    let mut follower = sim_arm(SimulatedServos::arm("follower").with_clock(clock.clone()), "follower", &clock);
    let options = RunOptions { hz: 200.0, max_ticks: Some(60), read_actuators: false };
    let lead = [Duration::ZERO; JOINT_COUNT];
    follower.set_step_limit(Some(40));
    let run = run_teleop(&mut leader, &mut follower, options, lead, &clock);
    run.result.unwrap();
    let last = run.steps.last().map(|s| s.follower);
    let covered = last.unwrap()[0] - 2048;
    assert!(covered >= 600, "follower covered only {covered} of 800 counts in 0.3 s");
}

#[test]
fn an_overloaded_gripper_is_released_and_teleop_keeps_running() {
    let clock = stepped();
    // The follower's jaws meet an object at tick 1900 while the leader asks for
    // 1600; after 0.7 s the leader opens to 2600. The capped gripper overloads.
    // Teleop must keep ticking, hold the gripper where it stopped, then follow
    // the leader open again.
    let hand: so101::arm::Hand = Arc::new(|id: u8, t: f64| {
        Some(if id == 6 {
            if t < 0.7 {
                1600.0
            } else {
                2600.0
            }
        } else {
            2048.0
        })
    });
    let object: so101::arm::Obstacle = Arc::new(|id: u8, _t: f64| (id == 6).then_some(1900.0));
    let mut leader =
        sim_arm(SimulatedServos::arm("leader").with_clock(clock.clone()).with_hand(hand), "leader", &clock);
    let mut follower =
        sim_arm(SimulatedServos::arm("follower").with_clock(clock.clone()).with_obstacle(object), "follower", &clock);
    follower.configure_follower().unwrap();
    let options = RunOptions { hz: 200.0, max_ticks: Some(260), read_actuators: false };
    let lead = [Duration::ZERO; JOINT_COUNT];
    follower.set_step_limit(Some(40));
    let run = run_teleop(&mut leader, &mut follower, options, lead, &clock);
    let stats = run.result.unwrap();
    let reports = run.steps;
    let g = Joint::Gripper.index();
    assert_eq!(stats.errors, 0, "an overload flag must not fail ticks");
    let held: Vec<_> = reports.iter().filter(|r| r.held(Joint::Gripper)).collect();
    assert!(!held.is_empty(), "the gripper never tripped");
    // Held at the contact point with a small fixed squeeze (30 counts), not driven at the leader's 1600.
    assert!(
        held.iter().skip(10).all(|r| (1860..=1900).contains(&r.sent[g])),
        "held goal must stop pushing into the object"
    );
    let last = reports.last().unwrap();
    assert!(!last.held(Joint::Gripper), "never resumed after the leader opened");
    assert!(last.follower[g] > 2300, "did not follow the leader open: {}", last.follower[g]);
}

#[test]
fn configure_writes_gains_max_acceleration_and_clears_the_wrap_bit() {
    let clock = stepped();
    use so101::arm::FollowerGains;
    let mut arm = sim_arm(SimulatedServos::arm("follower").with_clock(clock.clone()), "follower", &clock);
    arm.configure_follower_with(FollowerGains { p: 28, d: 32, max_torque: [900, 800, 700, 800, 700, 800] }).unwrap();
    assert_eq!(arm.read_register(Joint::Gripper, servos::MAX_TORQUE).unwrap(), 800);
    assert_eq!(arm.read_register(Joint::Gripper, servos::TORQUE_LIMIT).unwrap(), 800);
    assert_eq!(arm.read_register(Joint::ElbowFlex, servos::MAX_TORQUE).unwrap(), 700);
    assert_eq!(arm.read_register(Joint::ShoulderPan, servos::TORQUE_LIMIT).unwrap(), 900);
    assert_eq!(
        arm.read_register(Joint::Gripper, servos::PROTECTION_CURRENT).unwrap(),
        250,
        "stall protection stays on"
    );
    assert_eq!(arm.read_register(Joint::ElbowFlex, servos::P_COEFFICIENT).unwrap(), 28);
    assert_eq!(arm.read_register(Joint::ElbowFlex, servos::MAXIMUM_ACCELERATION).unwrap(), 254);
    assert_eq!(arm.read_register(Joint::ElbowFlex, servos::LOCK).unwrap(), 1);
    assert_eq!(arm.configure_readings().unwrap(), 0, "factory Phase 12 has bit 4 clear");
}

#[test]
fn lead_closes_the_gap_to_a_moving_leader() {
    let clock = stepped();
    // Leader pan sweeps at a constant 1200 counts/s. Commanding 60 ms ahead
    // should leave the follower closer to the leader than no lead does.
    let gap = |lead_ms: u64| {
        let hand: so101::arm::Hand =
            Arc::new(|id: u8, t: f64| Some(if id == 1 { 1200.0 + 1200.0 * t.min(1.5) } else { 2048.0 }));
        let mut leader =
            sim_arm(SimulatedServos::arm("leader").with_clock(clock.clone()).with_hand(hand), "leader", &clock);
        let mut follower = sim_arm(SimulatedServos::arm("follower").with_clock(clock.clone()), "follower", &clock);
        let options = RunOptions { hz: 100.0, max_ticks: Some(80), read_actuators: false };
        let lead = [Duration::from_millis(lead_ms); JOINT_COUNT];
        follower.set_step_limit(Some(80));
        let run = run_teleop(&mut leader, &mut follower, options, lead, &clock);
        run.result.unwrap();
        let errs: Vec<i32> = run.steps.iter().map(|r| (r.leader[0] - r.follower[0]).abs()).collect();
        errs[40..].iter().sum::<i32>() as f64 / 40.0
    };
    let (none, led) = (gap(0), gap(60));
    assert!(led < none * 0.6, "lead did not help: {none:.0} -> {led:.0} counts");
}

#[test]
fn a_late_eeprom_ack_does_not_leak_into_the_next_transaction() {
    // The sequence that failed on hardware: clear the wrap bit on servo 3, then
    // on servo 4. Servo 3's late ack landed in servo 4's exchange.
    let mut servos = Servos::new(Box::new(SlowEepromAcks::new()));
    servos.write_unlocked(3, servos::PHASE, 12).unwrap();
    servos.write_unlocked(4, servos::PHASE, 12).unwrap();
    assert_eq!(servos.read(5, servos::MODEL_NUMBER).unwrap(), 777);
    for id in [3, 4] {
        assert_eq!(servos.read(id, servos::PHASE).unwrap(), 12);
        assert_eq!(servos.read(id, servos::LOCK).unwrap(), 1, "servo {id} relocked");
    }
}

#[test]
fn lead_ignores_tremor_so_a_still_hand_gives_a_still_follower() {
    let clock = stepped();
    // The leader's pan trembles 4 counts (0.35 deg) at 8 Hz: a hand held still.
    // With 70 ms of lead the commanded goal must not shake more than without.
    let spread = |lead_ms: u64| {
        let hand: so101::arm::Hand = Arc::new(|id: u8, t: f64| {
            Some(if id == 1 { 2048.0 + 4.0 * (t * 8.0 * std::f64::consts::TAU).sin() } else { 2048.0 })
        });
        let mut leader =
            sim_arm(SimulatedServos::arm("leader").with_clock(clock.clone()).with_hand(hand), "leader", &clock);
        let mut follower = sim_arm(SimulatedServos::arm("follower").with_clock(clock.clone()), "follower", &clock);
        let options = RunOptions { hz: 100.0, max_ticks: Some(100), read_actuators: false };
        let lead = [Duration::from_millis(lead_ms); JOINT_COUNT];
        follower.set_step_limit(Some(40));
        let run = run_teleop(&mut leader, &mut follower, options, lead, &clock);
        run.result.unwrap();
        let goals: Vec<i32> = run.steps.iter().map(|s| s.sent[0]).collect();
        let g = &goals[20..];
        (*g.iter().max().unwrap() - *g.iter().min().unwrap()) as f64
    };
    let (still, led) = (spread(0), spread(70));
    assert!(led <= still + 1.0, "lead amplified tremor: goal spread {still} -> {led} counts");
}

#[test]
fn a_blocked_joint_is_held_before_any_servo_protection_trips() {
    let clock = stepped();
    // Pan is blocked at 2000 while the leader asks for 1900: a 100-tick gap,
    // well under saturation, so the servo never raises a flag. Teleop must
    // notice the stall by itself and hold at the contact point.
    let hand: so101::arm::Hand = Arc::new(|id: u8, _t: f64| Some(if id == 1 { 1900.0 } else { 2048.0 }));
    let wall: so101::arm::Obstacle = Arc::new(|id: u8, _t: f64| (id == 1).then_some(2000.0));
    let mut leader =
        sim_arm(SimulatedServos::arm("leader").with_clock(clock.clone()).with_hand(hand), "leader", &clock);
    let mut follower =
        sim_arm(SimulatedServos::arm("follower").with_clock(clock.clone()).with_obstacle(wall), "follower", &clock);
    let options = RunOptions { hz: 200.0, max_ticks: Some(120), read_actuators: true };
    let lead = [Duration::ZERO; JOINT_COUNT];
    follower.set_step_limit(Some(40));
    let run = run_teleop(&mut leader, &mut follower, options, lead, &clock);
    run.result.unwrap();
    let reports = run.steps;
    let pan_faults = reports.iter().flat_map(|r| r.actuators.iter()).filter(|a| a.joint == Joint::ShoulderPan);
    assert!(
        pan_faults.flat_map(|a| a.faults.iter()).all(|f| !matches!(f, Fault::Overload | Fault::Overcurrent)),
        "the servo should never have flagged"
    );
    let last = reports.last().unwrap();
    assert!(last.held(Joint::ShoulderPan), "stall not detected");
    assert!((last.sent[0] - 2000).abs() <= 2, "held goal {} should sit at the contact point", last.sent[0]);
}

#[test]
fn a_follower_that_keeps_failing_stops_teleop_instead_of_freezing() {
    let clock = stepped();
    let mut leader = sim_arm(SimulatedServos::waving_arm("leader").with_clock(clock.clone()), "leader", &clock);
    let dying = DiesAfter { inner: SimulatedServos::arm("follower").with_clock(clock.clone()), sends_left: 60 };
    let mut follower = ServoArm::new("follower", Servos::new(Box::new(dying)), Calibration::identity("follower"))
        .with_clock(clock.clone());
    let options = RunOptions { hz: 200.0, max_ticks: Some(2000), read_actuators: false };
    let lead = [Duration::ZERO; JOINT_COUNT];
    follower.set_step_limit(Some(40));
    let run = run_teleop(&mut leader, &mut follower, options, lead, &clock);
    let result = run.result;
    let errors = run.errors;
    let msg = result.unwrap_err().to_string();
    assert!(msg.contains("stopping with torque off"), "{msg}");
    assert!(errors <= 100, "ran {errors} failed ticks; the fail-safe is 0.5 s = 100 ticks at 200 Hz");
}

#[test]
fn stale_flags_at_startup_do_not_lock_the_gripper() {
    let clock = stepped();
    // Hardware, grab1.jsonl: overcurrent left over from the previous run showed
    // in the first 0.09 s, the first tick latched a hold, and the gripper stayed
    // shut for the whole run. A free gripper must follow the leader open.
    let hand: so101::arm::Hand = Arc::new(|id: u8, _t: f64| Some(if id == 6 { 2600.0 } else { 2048.0 }));
    let mut leader =
        sim_arm(SimulatedServos::arm("leader").with_clock(clock.clone()).with_hand(hand), "leader", &clock);
    let stale =
        StaleGripperFlags { inner: SimulatedServos::arm("follower").with_clock(clock.clone()), flagged_left: 8 };
    let mut follower = ServoArm::new("follower", Servos::new(Box::new(stale)), Calibration::identity("follower"))
        .with_clock(clock.clone());
    let options = RunOptions { hz: 200.0, max_ticks: Some(200), read_actuators: false };
    let lead = [Duration::ZERO; JOINT_COUNT];
    follower.set_step_limit(Some(40));
    let run = run_teleop(&mut leader, &mut follower, options, lead, &clock);
    run.result.unwrap();
    let reports = run.steps;
    let g = Joint::Gripper.index();
    let last = reports.last().unwrap();
    assert!(!last.held(Joint::Gripper), "gripper latched in a hold it never earned");
    assert!(last.follower[g] > 2500, "gripper did not open: {}", last.follower[g]);
}

#[test]
fn an_overload_flag_left_over_from_a_hold_does_not_trap_the_gripper_open() {
    let clock = stepped();
    // Hardware, grab1.jsonl (second run): holding an object keeps the servo's
    // overload flag on; opening the leader released the hold, the flag was still
    // set for a few ticks while the jaws had not started moving, and a second
    // hold latched ~30 counts open. It stayed until the leader closed again.
    let hand: so101::arm::Hand = Arc::new(|id: u8, t: f64| {
        Some(if id == 6 {
            if t < 0.6 {
                1600.0
            } else {
                2600.0
            }
        } else {
            2048.0
        })
    });
    let object: so101::arm::Obstacle = Arc::new(|id: u8, _t: f64| (id == 6).then_some(1900.0));
    let mut leader =
        sim_arm(SimulatedServos::arm("leader").with_clock(clock.clone()).with_hand(hand), "leader", &clock);
    // Flag on from the grip until ~0.15 s after the leader opens.
    let window = GripperFlagWindow {
        inner: SimulatedServos::arm("follower").with_clock(clock.clone()).with_obstacle(object),
        seen: 0,
        from: 60,
        to: 150,
    };
    let mut follower = ServoArm::new("follower", Servos::new(Box::new(window)), Calibration::identity("follower"))
        .with_clock(clock.clone());
    let options = RunOptions { hz: 200.0, max_ticks: Some(260), read_actuators: false };
    let lead = [Duration::ZERO; JOINT_COUNT];
    follower.set_step_limit(Some(40));
    let run = run_teleop(&mut leader, &mut follower, options, lead, &clock);
    run.result.unwrap();
    let reports = run.steps;
    let g = Joint::Gripper.index();
    assert!(reports.iter().any(|r| r.held(Joint::Gripper)), "the grip never held");
    let last = reports.last().unwrap();
    assert!(!last.held(Joint::Gripper), "gripper trapped in a hold after the leader opened");
    assert!(last.follower[g] > 2300, "gripper did not open: {}", last.follower[g]);
}
