//! Simulated STS3215 servos behind the `Transport` seam. They parse real
//! instruction bytes and answer with real status bytes.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::robot::joint::IDS;
use crate::wire::protocol::{encode_status, parse_frame, Instruction, ParseError, ServoError, BROADCAST_ID};
use crate::wire::registers::{self, Register, CENTER_TICK, TICKS_PER_TURN};
use crate::wire::transport::{Transport, TransportError};

// note: unloaded STS3215 is rated 0.22 s per 60 degrees, about 3100 ticks/s.
pub const MAX_SPEED_TICKS_PER_S: f64 = 3100.0;
const LOOP_GAIN_PER_S: f64 = 25.0;

// Moves a torque-off servo: given (id, time in seconds), returns its shaft angle in ticks.
pub type Hand = Arc<dyn Fn(u8, f64) -> Option<f64> + Send + Sync>;

// Something in the way of a torque-on servo: given (id, time in seconds), the
// lowest shaft angle it can reach (an object in the gripper, say).
pub type Obstacle = Arc<dyn Fn(u8, f64) -> Option<f64> + Send + Sync>;

const AMBIENT_C: f64 = 31.0;
// Load (0..1000) per tick of position error while torque is on.
const LOAD_PER_TICK_ERROR: f64 = 7.0;
// Static load from the arm's own weight, per joint id 1..6, at the worst angle.
const GRAVITY_LOAD: [f64; 6] = [0.0, 280.0, 170.0, 60.0, 0.0, 0.0];
// note: the STS3215 raises its overload bit only after the output has been
// saturated for Protection_Time (register 35, default 20 x 10 ms).
const PROTECTION_TIME_S: f64 = 0.2;

#[derive(Clone)]
pub enum SimClock {
    Real(Instant),
    Manual(Arc<Mutex<f64>>),
}

impl SimClock {
    pub fn manual() -> Self {
        SimClock::Manual(Arc::new(Mutex::new(0.0)))
    }
    pub fn now(&self) -> f64 {
        match self {
            SimClock::Real(epoch) => epoch.elapsed().as_secs_f64(),
            SimClock::Manual(t) => *t.lock().unwrap(),
        }
    }
    pub fn advance(&self, dt: Duration) {
        if let SimClock::Manual(t) = self {
            *t.lock().unwrap() += dt.as_secs_f64();
        }
    }
}

pub struct SimServo {
    pub id: u8,
    pub table: [u8; 128],
    // Shaft angle in ticks before the homing offset is applied.
    pub actual_ticks: f64,
    pub last_update_s: f64,
    // Winding temperature, integrated from load.
    pub temperature_c: f64,
    // How long the output has been saturated without a break.
    pub saturated_s: f64,
}

impl SimServo {
    pub fn new(id: u8, start_ticks: f64) -> Self {
        let mut s = Self { id, table: [0u8; 128], actual_ticks: start_ticks, last_update_s: 0.0, temperature_c: AMBIENT_C, saturated_s: 0.0 };
        for (reg, value) in FACTORY_DEFAULTS {
            s.set(reg, value);
        }
        s.set(registers::ID, id as i32);
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
        self.get(registers::TORQUE_ENABLE) == 1
    }

    pub fn homing_offset(&self) -> i32 {
        self.get(registers::HOMING_OFFSET)
    }

    pub fn present_ticks(&self) -> i32 {
        self.actual_ticks.round() as i32 - self.homing_offset()
    }

    fn step(&mut self, now: f64, hand: Option<&Hand>, obstacle: Option<&Obstacle>) {
        let dt = (now - self.last_update_s).max(0.0);
        self.last_update_s = now;
        if self.torque_on() {
            let goal_actual = (self.get(registers::GOAL_POSITION) + self.homing_offset()) as f64;
            let error = goal_actual - self.actual_ticks;
            let wanted = error * LOOP_GAIN_PER_S * dt;
            let cap = MAX_SPEED_TICKS_PER_S * dt;
            let delta = wanted.clamp(-cap, cap).clamp(-error.abs(), error.abs());
            let before = self.actual_ticks;
            self.actual_ticks += delta;
            if let Some(floor) = obstacle.and_then(|o| o(self.id, now)) {
                self.actual_ticks = self.actual_ticks.max(floor);
            }
            let residual = goal_actual - self.actual_ticks;
            if dt > 0.0 {
                self.set(registers::PRESENT_SPEED, ((self.actual_ticks - before) / dt).round() as i32);
            }
            self.set(registers::MOVING, if residual.abs() > 2.0 { 1 } else { 0 });
            // Load: what the loop pushes against (error) plus the arm's weight,
            // capped by the torque limit, like the real output stage.
            let angle = (self.actual_ticks - CENTER_TICK as f64) / TICKS_PER_TURN as f64 * std::f64::consts::TAU;
            let gravity = GRAVITY_LOAD[(self.id as usize).saturating_sub(1).min(5)] * angle.cos();
            let cap = self.get(registers::TORQUE_LIMIT).min(self.get(registers::MAX_TORQUE)).max(0) as f64;
            let moving_fast = dt > 0.0 && ((self.actual_ticks - before) / dt).abs() > 0.1 * MAX_SPEED_TICKS_PER_S;
            self.set_effort((residual * LOAD_PER_TICK_ERROR + gravity).clamp(-cap, cap), true, dt, moving_fast);
        } else {
            if let Some(ticks) = hand.and_then(|h| h(self.id, now)) {
                self.actual_ticks = ticks;
            }
            self.set(registers::PRESENT_SPEED, 0);
            self.set(registers::MOVING, 0);
            self.set_effort(0.0, false, dt, false);
        }
        self.actual_ticks = self.actual_ticks.clamp(0.0, (TICKS_PER_TURN - 1) as f64);
        self.set(registers::PRESENT_POSITION, self.present_ticks());
    }

    // Publish load, current, temperature, voltage sag and the overload bit.
    // `moving_fast`: a spinning motor's back-EMF limits its current, so only a
    // saturated output on a (nearly) stalled shaft counts toward an overload trip.
    fn set_effort(&mut self, load: f64, torque_on: bool, dt: f64, moving_fast: bool) {
        let magnitude = load.abs();
        // Heating goes with the square of current; cooling is Newtonian.
        let heat = (magnitude / 1000.0).powi(2) * 2.2;
        self.temperature_c += (heat - (self.temperature_c - AMBIENT_C) * 0.012) * dt;
        let current = if torque_on { 6.0 + magnitude * 0.9 } else { 0.0 };
        self.set(registers::PRESENT_LOAD, load.round() as i32);
        self.set(registers::PRESENT_CURRENT, current.round() as i32);
        self.set(registers::PRESENT_TEMPERATURE, self.temperature_c.round() as i32);
        self.set(registers::PRESENT_VOLTAGE, (74.0 - current / 120.0).round() as i32);
        let cap = self.get(registers::TORQUE_LIMIT).min(self.get(registers::MAX_TORQUE)).max(1) as f64;
        self.saturated_s = if torque_on && magnitude >= cap * 0.98 && !moving_fast { self.saturated_s + dt } else { 0.0 };
        let overloaded = self.saturated_s >= PROTECTION_TIME_S;
        self.set(registers::STATUS, if overloaded { ServoError::OVERLOAD as i32 } else { 0 });
    }

    fn write_bytes(&mut self, address: u8, bytes: &[u8]) -> ServoError {
        let a = address as usize;
        if a + bytes.len() > self.table.len() {
            return ServoError(ServoError::ANGLE_LIMIT);
        }
        if registers::is_eeprom_address(address) && self.get(registers::LOCK) == 1 {
            return ServoError(0);
        }
        let is_torque_enable = address == registers::TORQUE_ENABLE.address;
        if is_torque_enable && bytes[0] as i32 == registers::TORQUE_RECENTER {
            let offset = self.actual_ticks.round() as i32 - CENTER_TICK;
            self.set(registers::HOMING_OFFSET, offset.clamp(-2047, 2047));
            self.set(registers::TORQUE_ENABLE, 0);
            return ServoError(0);
        }
        self.table[a..a + bytes.len()].copy_from_slice(bytes);
        if is_torque_enable && bytes[0] == 1 && self.get(registers::GOAL_POSITION) == 0 {
            self.set(registers::GOAL_POSITION, self.present_ticks());
        }
        ServoError(0)
    }

    fn read_bytes(&self, address: u8, count: u8) -> Vec<u8> {
        let a = address as usize;
        let end = (a + count as usize).min(self.table.len());
        self.table[a..end].to_vec()
    }
}

const FACTORY_DEFAULTS: [(Register, i32); 13] = [
    (registers::MODEL_NUMBER, registers::STS3215_MODEL_NUMBER),
    (registers::RESPONSE_LEVEL, 1),
    (registers::MAX_ANGLE_LIMIT, 4095),
    (registers::MAX_TEMPERATURE, 70),
    (registers::MAX_TORQUE, 1000),
    (registers::PHASE, 12),
    (registers::P_COEFFICIENT, 32),
    (registers::D_COEFFICIENT, 32),
    (registers::LOCK, 1),
    (registers::TORQUE_LIMIT, 1000),
    (registers::PRESENT_VOLTAGE, 74),
    (registers::PRESENT_TEMPERATURE, 31),
    (registers::RETURN_DELAY, 0),
];

pub struct SimulatedBus {
    pub servos: Vec<SimServo>,
    pub clock: SimClock,
    hand: Option<Hand>,
    obstacle: Option<Obstacle>,
    reply_queue: VecDeque<u8>,
    pending_input: Vec<u8>,
    name: String,
    // Replies from these IDs are dropped, to exercise timeouts.
    pub silenced_ids: Vec<u8>,
}

impl SimulatedBus {
    pub fn new(name: &str, ids: &[u8], clock: SimClock) -> Self {
        let servos = ids.iter().map(|&id| SimServo::new(id, CENTER_TICK as f64)).collect();
        Self {
            servos,
            clock,
            hand: None,
            obstacle: None,
            reply_queue: VecDeque::new(),
            pending_input: Vec::new(),
            name: name.to_string(),
            silenced_ids: Vec::new(),
        }
    }

    // A full arm on the real clock that only tracks goals.
    pub fn arm(name: &str) -> Self {
        Self::new(name, &IDS, SimClock::Real(Instant::now()))
    }

    // A full arm on the real clock whose joints are waved by an invisible hand.
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

    pub fn settle(&mut self) {
        let now = self.clock.now();
        let hand = self.hand.as_ref();
        let obstacle = self.obstacle.as_ref();
        for s in &mut self.servos {
            s.step(now, hand, obstacle);
        }
    }

    fn reply(&mut self, id: u8, error: ServoError, params: &[u8]) {
        if self.silenced_ids.contains(&id) {
            return;
        }
        self.reply_queue.extend(encode_status(id, error, params));
    }

    fn handle_instruction(&mut self, id: u8, instruction: Instruction, params: &[u8]) {
        self.settle();
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
                        let status = ServoError(s.get(registers::STATUS) as u8);
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
                    let answers = s.get(registers::RESPONSE_LEVEL) == 1;
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
                        let status = ServoError(s.get(registers::STATUS) as u8);
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

// Every joint on its own slow sinusoid around centre.
pub fn waving_hand() -> Hand {
    Arc::new(|id: u8, t: f64| {
        let period_s = 3.0 + id as f64 * 0.7;
        let amplitude = 300.0 + id as f64 * 60.0;
        Some(CENTER_TICK as f64 + amplitude * (2.0 * std::f64::consts::PI * t / period_s).sin())
    })
}

impl Transport for SimulatedBus {
    fn send(&mut self, bytes: &[u8]) -> Result<(), TransportError> {
        self.pending_input.extend_from_slice(bytes);
        loop {
            match parse_frame(&self.pending_input) {
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

    fn receive_exact(&mut self, buf: &mut [u8], _timeout: Duration) -> Result<(), TransportError> {
        let wanted = buf.len();
        for (got, slot) in buf.iter_mut().enumerate() {
            match self.reply_queue.pop_front() {
                Some(b) => *slot = b,
                None => return Err(TransportError::Timeout { wanted, got }),
            }
        }
        Ok(())
    }

    fn discard_input(&mut self) -> Result<(), TransportError> {
        self.reply_queue.clear();
        Ok(())
    }

    fn name(&self) -> &str {
        &self.name
    }
}

// A scripted pick-and-place, so the simulated leader moves like a person
// rather than six metronomes: reach, descend, squeeze, lift, carry, release,
// return, on a 14 s loop, with a little tremor.
pub mod demo {
    use std::sync::Arc;

    use super::{Hand, Obstacle};
    use crate::robot::calibration::{ArmCalibration, JointCalibration};
    use crate::robot::joint::{Joint, COUNT};

    pub const LOOP_S: f64 = 14.0;
    // Joint half-range in ticks for the demo calibration (about +-97 degrees).
    const HALF_RANGE: i32 = 1100;
    // Gripper range in raw ticks: closed at MIN, open at MAX.
    const GRIP_MIN: i32 = 1600;
    const GRIP_MAX: i32 = 2900;
    const TICKS_PER_DEG: f64 = 4096.0 / 360.0;
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
                1..=5 => 2048.0 + (joints[id as usize - 1] + tremor) * TICKS_PER_DEG,
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

    // A calibration matching the demo's ranges, for either arm.
    pub fn calibration(arm_id: &str) -> ArmCalibration {
        let joints: [JointCalibration; COUNT] = std::array::from_fn(|i| {
            let (range_min, range_max) =
                if Joint::ALL[i] == Joint::Gripper { (GRIP_MIN, GRIP_MAX) } else { (2048 - HALF_RANGE, 2048 + HALF_RANGE) };
            JointCalibration { homing_offset: 0, range_min, range_max }
        });
        ArmCalibration { arm_id: arm_id.to_string(), joints }
    }
}
