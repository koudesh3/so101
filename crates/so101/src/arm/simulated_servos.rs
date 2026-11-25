//! Simulated STS3215 servos behind the `Transport` seam. They parse real
//! instruction bytes and answer with real status bytes.

use std::collections::VecDeque;
use std::sync::Arc;
use std::time::Duration;

use crate::arm::servos::{self, Register, CENTER_COUNT, COUNTS_PER_TURN};
use crate::arm::servos::{encode_status, parse_packet, Instruction, ParseError, ServoError, BROADCAST_ID};
use crate::arm::Error;
use crate::arm::Transport;
use crate::arm::IDS;
use crate::clock::{Clock, MonotonicClock, Timestamp};
use crate::rng::Rng;

// Counts per degree of a servo's own angle.
const COUNTS_PER_DEG: f64 = COUNTS_PER_TURN as f64 / 360.0;
// The physics' longest time step.
const MAX_STEP_S: f64 = 0.001;
// One byte on the wire at 1 Mbaud: 10 bits.
const BYTE_TIME_S: f64 = 10e-6;
const AMBIENT_C: f64 = 31.0;

// Load (0..1000) per tick of position error while torque is on.
const LOAD_PER_COUNT_ERROR: f64 = 7.0;

// Static load from the arm's own weight, per joint id 1..6, at the worst angle.
const GRAVITY_LOAD: [f64; 6] = [0.0, 280.0, 170.0, 60.0, 0.0, 0.0];

// note: the STS3215 raises its overload bit only after the output has been
// saturated for Protection_Time (register 35, default 20 x 10 ms).
const PROTECTION_TIME_S: f64 = 0.2;

const FACTORY_DEFAULTS: [(Register, i32); 13] = [
    (servos::MODEL_NUMBER, servos::STS3215_MODEL_NUMBER),
    (servos::RESPONSE_LEVEL, 1),
    (servos::MAX_ANGLE_LIMIT, 4095),
    (servos::MAX_TEMPERATURE, 70),
    (servos::MAX_TORQUE, 1000),
    (servos::PHASE, 12),
    (servos::P_COEFFICIENT, 32),
    (servos::D_COEFFICIENT, 32),
    (servos::LOCK, 1),
    (servos::TORQUE_LIMIT, 1000),
    (servos::PRESENT_VOLTAGE, 74),
    (servos::PRESENT_TEMPERATURE, 31),
    (servos::RETURN_DELAY, 0),
];

/// How our follower's servos behave beyond the protocol, fitted to the rig (2026-11-24,
/// /projects/spikes/sysid-20261124 on the Spark): the motion from the system-ID run
/// (`tests/hardware.rs`: steps of 1-30°, ramps to 150°/s and small reversals on each joint), the
/// static part from the gravity sweeps (each joint settling from above and below at 15 poses).
/// Checked against recordings the fit never saw: the fine-motion test and both teleop episodes.
/// Per joint, in LeRobot's order (shoulder pan .. gripper); the gripper's numbers are in its raw
/// counts' degrees (4096 to a turn), like the others.
#[derive(Clone, Copy, Debug)]
pub struct ServoProfile {
    pub motor: [Motor; 6],
    /// A goal moving back by less than this doesn't move the joint, which settles half of it short
    /// of its goal: the difference between settling from above and from below (sweeps).
    pub play_deg: [f64; 6],
    pub gravity: [Gravity; 6],
    /// One temperature reading in this many is junk (45-150 C in a steady band): 2026-11-24, both
    /// episodes, about 1 in 500 on every servo. 0 for none.
    pub junk_temperature_one_in: u32,
    /// One voltage reading in this many is junk (10.2-11.2 V on 11.9 V), per servo: only wrist
    /// roll (21 in ~24,000) and the gripper (37 in ~24,000) did, 2026-11-24.
    pub junk_voltage_one_in: [u32; 6],
    /// Supply, in volts: the follower's 12 V adapter (11.8-11.9 V at the servos).
    pub supply_v: f64,
    /// Bus time per transaction: the USB adapter's turnaround plus 10 µs per byte at 1 Mbaud. A
    /// position read of all six took 1.0 ms (62 bytes), a status read about 1.6 ms more (2026-11-23).
    pub bus_turnaround: Duration,
}

/// How a servo moves toward its goal: after a dead time, its speed is a pull proportional to the
/// distance left, limited in acceleration and capped. Peak speed then grows with step size as
/// measured (about 22°/s for 1°, 55 for 3°, 125 for 10°, 240 for 30°), which neither a pure
/// first-order servo nor a fixed speed cap does.
#[derive(Clone, Copy, Debug)]
pub struct Motor {
    pub dead_time: Duration,
    pub gain_per_s: f64,
    pub accel_deg_s2: f64,
    pub max_speed_deg_s: f64,
}

/// How far above its goal a joint settles under gravity, in degrees:
/// `constant + cos_term * cos(angle) + sin_term * sin(angle)` of the joint's own angle from the
/// servo's centre. Only shoulder lift's varies, 0.0° tucked to 1.0° stretched out (one cosine fits
/// the 15 poses to 0.08°); the others are flat to within an encoder count (0.09°).
#[derive(Clone, Copy, Debug)]
pub struct Gravity {
    pub constant: f64,
    pub cos_term: f64,
    pub sin_term: f64,
}

impl Gravity {
    pub const fn flat(deg: f64) -> Self {
        Self { constant: deg, cos_term: 0.0, sin_term: 0.0 }
    }

    pub fn above_goal_deg(&self, angle_rad: f64) -> f64 {
        self.constant + self.cos_term * angle_rad.cos() + self.sin_term * angle_rad.sin()
    }
}

impl ServoProfile {
    /// The follower, as fitted.
    pub const FOLLOWER: ServoProfile = ServoProfile {
        motor: [
            Motor::new(50.3, 21.3, 2330.0, 342.0),
            Motor::new(52.2, 26.6, 2110.0, 226.0),
            Motor::new(54.7, 24.8, 1990.0, 327.0),
            Motor::new(52.7, 23.7, 1870.0, 350.0),
            Motor::new(37.9, 17.3, 2100.0, 454.0),
            // The gripper's top speed is its 30% torque ceiling (gains.json).
            Motor::new(42.3, 40.2, 1460.0, 93.0),
        ],
        play_deg: [0.31, 0.46, 0.38, 0.19, 0.33, 0.37],
        gravity: [
            Gravity::flat(0.05),
            Gravity { constant: 0.635, cos_term: 0.096, sin_term: 0.568 },
            // Unresolved: teleop recordings earlier the same day settled 0.1-1.3° lower, at the
            // same poses (docs/phase1.md).
            Gravity::flat(0.56),
            Gravity::flat(0.22),
            Gravity::flat(0.02),
            Gravity::flat(-0.04),
        ],
        junk_temperature_one_in: 500,
        junk_voltage_one_in: [0, 0, 0, 0, 1100, 650],
        supply_v: 11.9,
        bus_turnaround: Duration::from_micros(380),
    };
}

impl Motor {
    const fn new(dead_ms: f64, gain_per_s: f64, accel_deg_s2: f64, max_speed_deg_s: f64) -> Self {
        Self { dead_time: Duration::from_micros((dead_ms * 1000.0) as u64), gain_per_s, accel_deg_s2, max_speed_deg_s }
    }
}

// Moves a torque-off servo: given (id, time in seconds), returns its shaft angle in counts.
pub type Hand = Arc<dyn Fn(u8, f64) -> Option<f64> + Send + Sync>;

// Something in the way of a torque-on servo: given (id, time in seconds), the
// lowest shaft angle it can reach (an object in the gripper, say).
pub type Obstacle = Arc<dyn Fn(u8, f64) -> Option<f64> + Send + Sync>;

pub struct SimServo {
    pub id: u8,
    pub table: [u8; 128],
    // Shaft angle in counts before the homing offset is applied.
    pub actual_counts: f64,
    pub last_update_s: f64,
    // Winding temperature, integrated from load.
    pub temperature_c: f64,
    pub saturated_s: f64,
    // Goals as written, (time s, register value), oldest first: the dead time picks from these.
    goals: VecDeque<(f64, i32)>,
    // Where the play lets the motor aim: moves only when the goal pushes past half the play.
    aim: f64,
    // The motor's speed, in counts/s: it changes no faster than the acceleration limit.
    speed: f64,
    rng: Rng,
    // Supply in decivolts, as the voltage register counts.
    supply_dv: f64,
}

pub struct SimulatedServos {
    pub servos: Vec<SimServo>,
    clock: Arc<dyn Clock>,
    start: Timestamp,
    hand: Option<Hand>,
    obstacle: Option<Obstacle>,
    reply_queue: VecDeque<u8>,
    pending_input: Vec<u8>,
    name: String,
    // Replies from these IDs are dropped, to exercise timeouts.
    pub silenced_ids: Vec<u8>,
    // These IDs miss the first read after their torque comes on, as servo 1 did on hardware.
    pub deaf_after_torque_on: Vec<u8>,
    deaf: Vec<u8>,
    pub profile: ServoProfile,
    // Faults: (id, every Nth read, extra time): that servo answers late now and then (one took
    // 13.5 ms, 2026-11-23).
    pub slow_replies: Vec<(u8, u32, Duration)>,
    reads: u64,
}

impl SimServo {
    pub fn new(id: u8, start_counts: f64) -> Self {
        let mut s = Self {
            id,
            table: [0u8; 128],
            actual_counts: start_counts,
            last_update_s: 0.0,
            temperature_c: AMBIENT_C,
            saturated_s: 0.0,
            goals: VecDeque::new(),
            aim: start_counts,
            speed: 0.0,
            rng: Rng::new(0x5E4F0 + id as u64),
            supply_dv: 119.0,
        };
        for (reg, value) in FACTORY_DEFAULTS {
            s.set(reg, value);
        }
        s.set(servos::ID, id as i32);
        s
    }

    pub fn get(&self, reg: Register) -> i32 {
        reg.decode(&self.table[reg.address as usize..])
    }

    pub fn set(&mut self, reg: Register, value: i32) {
        let bytes = reg.encode(value);
        let a = reg.address as usize;
        self.table[a..a + bytes.len()].copy_from_slice(&bytes);
    }

    pub fn torque_on(&self) -> bool {
        self.get(servos::TORQUE_ENABLE) == 1
    }

    pub fn homing_offset(&self) -> i32 {
        self.get(servos::HOMING_OFFSET)
    }

    pub fn present_counts(&self) -> i32 {
        self.actual_counts.round() as i32 - self.homing_offset()
    }

    // Advances the servo to `now` in steps of at most `MAX_STEP_S`, so a long wait (a test jumping
    // the clock) integrates the same as many short ones.
    fn step(&mut self, now: f64, profile: &ServoProfile, hand: Option<&Hand>, obstacle: Option<&Obstacle>) {
        let start = self.last_update_s;
        let steps = ((now - start) / MAX_STEP_S).ceil().max(1.0);
        for k in 1..=steps as u64 {
            self.integrate(start + (now - start) * k as f64 / steps, profile, hand, obstacle);
        }
        let joint = (self.id as usize).saturating_sub(1).min(5);
        self.junk_readings(profile, joint);
    }

    fn integrate(&mut self, now: f64, profile: &ServoProfile, hand: Option<&Hand>, obstacle: Option<&Obstacle>) {
        let dt = (now - self.last_update_s).max(0.0);
        self.last_update_s = now;
        let joint = (self.id as usize).saturating_sub(1).min(5);
        self.supply_dv = profile.supply_v * 10.0;
        let angle = (self.actual_counts - CENTER_COUNT as f64) / COUNTS_PER_TURN as f64 * std::f64::consts::TAU;
        let motor = profile.motor[joint];
        let max_speed = motor.max_speed_deg_s * COUNTS_PER_DEG;
        if self.torque_on() {
            // The stages, in order: dead time, gravity, play, then the motor.
            let goal = self.goal_in_effect(now, motor.dead_time);
            let above = profile.gravity[joint].above_goal_deg(angle) * COUNTS_PER_DEG;
            let goal_actual = (goal + self.homing_offset()) as f64 + above;
            let half_play = profile.play_deg[joint] * COUNTS_PER_DEG / 2.0;
            self.aim = self.aim.clamp(goal_actual - half_play, goal_actual + half_play);
            let error = self.aim - self.actual_counts;
            let wanted = (motor.gain_per_s * error).clamp(-max_speed, max_speed);
            let max_change = motor.accel_deg_s2 * COUNTS_PER_DEG * dt;
            self.speed += (wanted - self.speed).clamp(-max_change, max_change);
            let before = self.actual_counts;
            let delta = self.speed * dt;
            if error * (error - delta) < 0.0 {
                // It would pass the aim: it stops there.
                self.actual_counts = self.aim;
                self.speed = 0.0;
            } else {
                self.actual_counts += delta;
            }
            if let Some(floor) = obstacle.and_then(|o| o(self.id, now)) {
                self.actual_counts = self.actual_counts.max(floor);
            }
            let residual = self.aim - self.actual_counts;
            if dt > 0.0 {
                self.set(servos::PRESENT_SPEED, ((self.actual_counts - before) / dt).round() as i32);
            }
            self.set(servos::MOVING, if residual.abs() > 2.0 { 1 } else { 0 });
            // Load: what the loop pushes against (error) plus the arm's weight,
            // capped by the torque limit, like the real output stage.
            let gravity = GRAVITY_LOAD[joint] * angle.cos();
            let cap = self.get(servos::TORQUE_LIMIT).min(self.get(servos::MAX_TORQUE)).max(0) as f64;
            let moving_fast = dt > 0.0 && ((self.actual_counts - before) / dt).abs() > 0.1 * max_speed;
            self.set_effort((residual * LOAD_PER_COUNT_ERROR + gravity).clamp(-cap, cap), true, dt, moving_fast);
        } else {
            if let Some(counts) = hand.and_then(|h| h(self.id, now)) {
                self.actual_counts = counts;
            }
            self.aim = self.actual_counts;
            self.speed = 0.0;
            self.goals.clear();
            self.set(servos::PRESENT_SPEED, 0);
            self.set(servos::MOVING, 0);
            self.set_effort(0.0, false, dt, false);
        }
        self.actual_counts = self.actual_counts.clamp(0.0, (COUNTS_PER_TURN - 1) as f64);
        self.set(servos::PRESENT_POSITION, self.present_counts());
    }

    // Notes a goal as written at `now`, for the dead time.
    fn note_goal(&mut self, now: f64, dead_time: Duration) {
        let goal = self.get(servos::GOAL_POSITION);
        match self.goals.back() {
            Some(&(_, last)) if last == goal => {}
            // The first goal (at torque-on, the present position) holds from the start.
            None => self.goals.push_back((now - dead_time.as_secs_f64(), goal)),
            Some(_) => self.goals.push_back((now, goal)),
        }
    }

    // The newest goal written at least `dead_time` before `now`; older ones are dropped.
    fn goal_in_effect(&mut self, now: f64, dead_time: Duration) -> i32 {
        let due = now - dead_time.as_secs_f64();
        while self.goals.len() > 1 && self.goals[1].0 <= due {
            self.goals.pop_front();
        }
        self.goals.front().map_or_else(|| self.get(servos::GOAL_POSITION), |&(_, g)| g)
    }

    // Single junk readings, as the real servos send now and then (with a correct checksum).
    fn junk_readings(&mut self, profile: &ServoProfile, joint: usize) {
        let one_in = |n: u32, rng: &mut Rng| n > 0 && rng.chance(1.0 / f64::from(n));
        if one_in(profile.junk_temperature_one_in, &mut self.rng) {
            let junk = 45.0 + self.rng.unit() * 105.0;
            self.set(servos::PRESENT_TEMPERATURE, junk as i32);
        }
        if one_in(profile.junk_voltage_one_in[joint], &mut self.rng) {
            let junk = 102.0 + self.rng.unit() * 10.0;
            self.set(servos::PRESENT_VOLTAGE, junk as i32);
        }
    }

    // `moving_fast`: a spinning motor's back-EMF limits its current, so only a
    // saturated output on a (nearly) stalled shaft counts toward an overload trip.
    fn set_effort(&mut self, load: f64, torque_on: bool, dt: f64, moving_fast: bool) {
        let magnitude = load.abs();
        // Heating goes with the square of current; cooling is Newtonian.
        let heat = (magnitude / 1000.0).powi(2) * 2.2;
        self.temperature_c += (heat - (self.temperature_c - AMBIENT_C) * 0.012) * dt;
        let current = if torque_on { 6.0 + magnitude * 0.9 } else { 0.0 };
        self.set(servos::PRESENT_LOAD, load.round() as i32);
        self.set(servos::PRESENT_CURRENT, current.round() as i32);
        self.set(servos::PRESENT_TEMPERATURE, self.temperature_c.round() as i32);
        self.set(servos::PRESENT_VOLTAGE, (self.supply_dv - current / 120.0).round() as i32);
        let cap = self.get(servos::TORQUE_LIMIT).min(self.get(servos::MAX_TORQUE)).max(1) as f64;
        self.saturated_s =
            if torque_on && magnitude >= cap * 0.98 && !moving_fast { self.saturated_s + dt } else { 0.0 };
        let overloaded = self.saturated_s >= PROTECTION_TIME_S;
        self.set(servos::STATUS, if overloaded { ServoError::OVERLOAD as i32 } else { 0 });
    }

    fn write_bytes(&mut self, address: u8, bytes: &[u8]) -> ServoError {
        let a = address as usize;
        if a + bytes.len() > self.table.len() {
            return ServoError(ServoError::ANGLE_LIMIT);
        }
        if servos::is_eeprom_address(address) && self.get(servos::LOCK) == 1 {
            return ServoError(0);
        }
        let is_torque_enable = address == servos::TORQUE_ENABLE.address;
        if is_torque_enable && bytes[0] as i32 == servos::TORQUE_RECENTER {
            let offset = self.actual_counts.round() as i32 - CENTER_COUNT;
            self.set(servos::HOMING_OFFSET, offset.clamp(-2047, 2047));
            self.set(servos::TORQUE_ENABLE, 0);
            return ServoError(0);
        }
        self.table[a..a + bytes.len()].copy_from_slice(bytes);
        if is_torque_enable && bytes[0] == 1 && self.get(servos::GOAL_POSITION) == 0 {
            self.set(servos::GOAL_POSITION, self.present_counts());
        }
        ServoError(0)
    }

    fn read_bytes(&self, address: u8, count: u8) -> Vec<u8> {
        let a = address as usize;
        let end = (a + count as usize).min(self.table.len());
        self.table[a..end].to_vec()
    }
}

impl SimulatedServos {
    pub fn new(name: &str, ids: &[u8], clock: Arc<dyn Clock>) -> Self {
        let servos = ids.iter().map(|&id| SimServo::new(id, CENTER_COUNT as f64)).collect();
        let start = clock.now();
        Self {
            servos,
            clock,
            start,
            hand: None,
            obstacle: None,
            reply_queue: VecDeque::new(),
            pending_input: Vec::new(),
            name: name.to_string(),
            silenced_ids: Vec::new(),
            deaf_after_torque_on: Vec::new(),
            deaf: Vec::new(),
            profile: ServoProfile::FOLLOWER,
            slow_replies: Vec::new(),
            reads: 0,
        }
    }

    /// A full arm on the real clock that only tracks goals.
    pub fn arm(name: &str) -> Self {
        Self::new(name, &IDS, Arc::new(MonotonicClock))
    }

    /// Runs the physics on `clock` from now on, starting at its present time.
    pub fn with_clock(mut self, clock: Arc<dyn Clock>) -> Self {
        self.start = clock.now();
        self.clock = clock;
        self
    }

    fn now_s(&self) -> f64 {
        self.clock.now().since(self.start).as_secs_f64()
    }

    pub fn waving_arm(name: &str) -> Self {
        Self::arm(name).with_hand(waving_hand())
    }

    pub fn with_hand(mut self, hand: Hand) -> Self {
        self.hand = Some(hand);
        self
    }

    pub fn with_obstacle(mut self, obstacle: Obstacle) -> Self {
        self.obstacle = Some(obstacle);
        self
    }

    // The demo pair: a leader running a pick-and-place, a follower whose
    // gripper meets an object while it is held.
    pub fn demo_leader(name: &str) -> Self {
        Self::arm(name).with_hand(demo::hand())
    }

    pub fn demo_follower(name: &str) -> Self {
        Self::arm(name).with_obstacle(demo::object())
    }

    pub fn servo(&self, id: u8) -> Option<&SimServo> {
        self.servos.iter().find(|s| s.id == id)
    }

    pub fn servo_mut(&mut self, id: u8) -> Option<&mut SimServo> {
        self.servos.iter_mut().find(|s| s.id == id)
    }

    pub fn with_profile(mut self, profile: ServoProfile) -> Self {
        self.profile = profile;
        self
    }

    pub fn settle(&mut self) {
        let now = self.now_s();
        let hand = self.hand.as_ref();
        let obstacle = self.obstacle.as_ref();
        for s in &mut self.servos {
            s.step(now, &self.profile, hand, obstacle);
        }
    }

    fn reply(&mut self, id: u8, error: ServoError, params: &[u8]) {
        if self.silenced_ids.contains(&id) {
            return;
        }
        if let Some(i) = self.deaf.iter().position(|&d| d == id) {
            self.deaf.remove(i);
            return;
        }
        self.reply_queue.extend(encode_status(id, error, params));
    }

    fn handle_instruction(&mut self, id: u8, instruction: Instruction, params: &[u8]) {
        self.settle();
        let torque_before: Vec<(u8, bool)> = self.servos.iter().map(|s| (s.id, s.torque_on())).collect();
        let replied_before = self.reply_queue.len();
        self.dispatch(id, instruction, params);
        let now = self.now_s();
        for s in self.servos.iter_mut().filter(|s| s.torque_on()) {
            let joint = (s.id as usize).saturating_sub(1).min(5);
            s.note_goal(now, self.profile.motor[joint].dead_time);
        }
        self.charge_bus_time(id, instruction, params, self.reply_queue.len() - replied_before);
        for (id, was_on) in torque_before {
            let now_on = self.servo(id).is_some_and(|s| s.torque_on());
            if !was_on && now_on && self.deaf_after_torque_on.contains(&id) {
                self.deaf.push(id);
            }
        }
    }

    // The clock moves by what the transaction takes on the wire, so the loop sees the bus's real
    // cost per cycle; and a servo set to answer slowly now and then does.
    fn charge_bus_time(&mut self, id: u8, instruction: Instruction, params: &[u8], reply_bytes: usize) {
        if self.profile.bus_turnaround.is_zero() {
            return;
        }
        let bytes = params.len() + 6 + reply_bytes;
        let mut cost = self.profile.bus_turnaround.as_secs_f64() + bytes as f64 * BYTE_TIME_S;
        if matches!(instruction, Instruction::Read | Instruction::SyncRead) {
            self.reads += 1;
            let asked: Vec<u8> = match instruction {
                Instruction::SyncRead => params.get(2..).unwrap_or_default().to_vec(),
                _ => vec![id],
            };
            for &(slow_id, every, extra) in &self.slow_replies {
                if asked.contains(&slow_id) && every > 0 && self.reads.is_multiple_of(u64::from(every)) {
                    cost += extra.as_secs_f64();
                }
            }
        }
        self.clock.sleep_until(self.clock.now() + Duration::from_secs_f64(cost));
    }

    fn dispatch(&mut self, id: u8, instruction: Instruction, params: &[u8]) {
        let targets: Vec<u8> = if id == BROADCAST_ID { self.servos.iter().map(|s| s.id).collect() } else { vec![id] };
        match instruction {
            Instruction::Ping => {
                for t in targets {
                    if self.servo(t).is_some() {
                        self.reply(t, ServoError(0), &[]);
                    }
                }
            }
            Instruction::Read => {
                let [address, count, ..] = params else { return };
                for t in targets {
                    if let Some(s) = self.servo(t) {
                        let bytes = s.read_bytes(*address, *count);
                        let status = ServoError(s.get(servos::STATUS) as u8);
                        self.reply(t, status, &bytes);
                    }
                }
            }
            Instruction::Write | Instruction::RegWrite => {
                let [address, data @ ..] = params else { return };
                let broadcast = id == BROADCAST_ID;
                for t in targets {
                    let Some(s) = self.servo_mut(t) else { continue };
                    let err = s.write_bytes(*address, data);
                    let answers = s.get(servos::RESPONSE_LEVEL) == 1;
                    if !broadcast && answers {
                        self.reply(t, err, &[]);
                    }
                }
            }
            Instruction::Action => {}
            Instruction::SyncRead => {
                let [address, count, ids @ ..] = params else { return };
                for &t in ids {
                    if let Some(s) = self.servo(t) {
                        let bytes = s.read_bytes(*address, *count);
                        let status = ServoError(s.get(servos::STATUS) as u8);
                        self.reply(t, status, &bytes);
                    }
                }
            }
            Instruction::SyncWrite => {
                let [address, width, blocks @ ..] = params else { return };
                for block in blocks.chunks_exact(1 + *width as usize) {
                    if let Some(s) = self.servo_mut(block[0]) {
                        s.write_bytes(*address, &block[1..]);
                    }
                }
            }
        }
    }
}

impl Transport for SimulatedServos {
    fn send(&mut self, bytes: &[u8]) -> crate::arm::Result<()> {
        self.pending_input.extend_from_slice(bytes);
        loop {
            match parse_packet(&self.pending_input) {
                Ok((frame, used)) => {
                    self.pending_input.drain(..used);
                    if let Some(instr) = frame.instruction() {
                        self.handle_instruction(frame.id, instr, &frame.params);
                    }
                }
                Err(ParseError::NoHeader { skip }) => {
                    self.pending_input.drain(..skip);
                }
                Err(ParseError::BadChecksum { .. }) => {
                    self.pending_input.drain(..2);
                }
                Err(ParseError::Incomplete(_)) => break,
            }
        }
        Ok(())
    }

    fn receive_exact(&mut self, buf: &mut [u8], _timeout: Duration) -> crate::arm::Result<()> {
        let wanted = buf.len();
        for (got, slot) in buf.iter_mut().enumerate() {
            match self.reply_queue.pop_front() {
                Some(b) => *slot = b,
                None => return Err(Error::Timeout { wanted, got }),
            }
        }
        Ok(())
    }

    fn discard_input(&mut self) -> crate::arm::Result<()> {
        self.reply_queue.clear();
        Ok(())
    }

    fn name(&self) -> &str {
        &self.name
    }
}

pub fn waving_hand() -> Hand {
    Arc::new(|id: u8, t: f64| {
        let period_s = 3.0 + id as f64 * 0.7;
        let amplitude = 300.0 + id as f64 * 60.0;
        Some(CENTER_COUNT as f64 + amplitude * (2.0 * std::f64::consts::PI * t / period_s).sin())
    })
}

// A scripted pick-and-place, so the simulated leader moves like a person
// rather than six metronomes: reach, descend, squeeze, lift, carry, release,
// return, on a 14 s loop, with a little tremor.
pub mod demo {
    use std::sync::Arc;

    use super::{Hand, Obstacle};
    use crate::arm::calibration::{Calibration, JointCalibration};
    use crate::arm::{Joint, JOINT_COUNT};

    pub const LOOP_S: f64 = 14.0;
    // Joint half-range in counts for the demo calibration (about +-97 degrees).
    const HALF_RANGE: i32 = 1100;
    // Gripper range in raw counts: closed at MIN, open at MAX.
    const GRIP_MIN: i32 = 1600;
    const GRIP_MAX: i32 = 2900;
    const COUNTS_PER_DEG: f64 = 4096.0 / 360.0;
    // The object's width, as the gripper percentage where the jaws meet it.
    const OBJECT_PCT: f64 = 34.0;
    // When the object sits between the follower's jaws, in loop seconds.
    const OBJECT_HELD: (f64, f64) = (5.2, 10.9);

    // (time s, [pan, lift, elbow, wrist_flex, wrist_roll] degrees, gripper %)
    const KEYS: [(f64, [f64; 5], f64); 9] = [
        (0.0, [0.0, -55.0, 65.0, 35.0, 0.0], 30.0),
        (1.2, [0.0, -55.0, 65.0, 35.0, 0.0], 30.0),
        (3.0, [32.0, -5.0, 25.0, 50.0, 15.0], 95.0),
        (4.4, [32.0, 18.0, 5.0, 62.0, 25.0], 95.0),
        (5.6, [32.0, 18.0, 5.0, 62.0, 25.0], 0.0),
        (7.4, [30.0, -18.0, 30.0, 45.0, 25.0], 0.0),
        (9.4, [-38.0, 8.0, 12.0, 55.0, -35.0], 0.0),
        (11.0, [-38.0, 8.0, 12.0, 55.0, -35.0], 95.0),
        (12.8, [0.0, -55.0, 65.0, 35.0, 0.0], 30.0),
    ];

    fn ease(x: f64) -> f64 {
        let x = x.clamp(0.0, 1.0);
        x * x * x * (x * (x * 6.0 - 15.0) + 10.0)
    }

    // Pose at time t: five joint degrees and the gripper percent.
    pub fn pose(t: f64) -> ([f64; 5], f64) {
        let t = t.rem_euclid(LOOP_S);
        let i = KEYS.iter().rposition(|k| k.0 <= t).unwrap_or(0);
        let (t0, a, g0) = KEYS[i];
        let (t1, b, g1) = KEYS.get(i + 1).copied().unwrap_or((LOOP_S, KEYS[0].1, KEYS[0].2));
        let u = ease((t - t0) / (t1 - t0).max(1e-6));
        (std::array::from_fn(|k| a[k] + (b[k] - a[k]) * u), g0 + (g1 - g0) * u)
    }

    pub fn hand() -> Hand {
        Arc::new(|id: u8, t: f64| {
            let (joints, grip) = pose(t);
            // A person's hand is never still: two slow incommensurate wobbles.
            let tremor = 0.7 * (t * 1.9 + id as f64).sin() + 0.4 * (t * 3.7 + 2.0 * id as f64).sin();
            Some(match id {
                1..=5 => 2048.0 + (joints[id as usize - 1] + tremor) * COUNTS_PER_DEG,
                6 => GRIP_MIN as f64 + grip.clamp(0.0, 100.0) / 100.0 * (GRIP_MAX - GRIP_MIN) as f64,
                _ => 2048.0,
            })
        })
    }

    // The object in the follower's jaws while held: they stop at its width
    // while the leader squeezes further. That is a stall.
    pub fn object() -> Obstacle {
        Arc::new(|id: u8, t: f64| {
            let t = t.rem_euclid(LOOP_S);
            (id == 6 && (OBJECT_HELD.0..=OBJECT_HELD.1).contains(&t))
                .then(|| GRIP_MIN as f64 + OBJECT_PCT / 100.0 * (GRIP_MAX - GRIP_MIN) as f64)
        })
    }

    pub fn calibration(arm_id: &str) -> Calibration {
        let joints: [JointCalibration; JOINT_COUNT] = std::array::from_fn(|i| {
            let (range_min, range_max) = if Joint::ALL[i] == Joint::Gripper {
                (GRIP_MIN, GRIP_MAX)
            } else {
                (2048 - HALF_RANGE, 2048 + HALF_RANGE)
            };
            JointCalibration { homing_offset: 0, range_min, range_max }
        });
        Calibration { arm_id: arm_id.to_string(), joints }
    }
}
