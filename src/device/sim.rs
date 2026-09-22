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
}

impl SimServo {
    pub fn new(id: u8, start_ticks: f64) -> Self {
        let mut s = Self { id, table: [0u8; 128], actual_ticks: start_ticks, last_update_s: 0.0 };
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

    fn step(&mut self, now: f64, hand: Option<&Hand>) {
        let dt = (now - self.last_update_s).max(0.0);
        self.last_update_s = now;
        if self.torque_on() {
            let goal_actual = (self.get(registers::GOAL_POSITION) + self.homing_offset()) as f64;
            let error = goal_actual - self.actual_ticks;
            let wanted = error * LOOP_GAIN_PER_S * dt;
            let cap = MAX_SPEED_TICKS_PER_S * dt;
            let delta = wanted.clamp(-cap, cap).clamp(-error.abs(), error.abs());
            self.actual_ticks += delta;
            if dt > 0.0 {
                self.set(registers::PRESENT_SPEED, (delta / dt).round() as i32);
            }
            self.set(registers::MOVING, if error.abs() > 2.0 { 1 } else { 0 });
        } else if let Some(hand) = hand {
            if let Some(ticks) = hand(self.id, now) {
                self.actual_ticks = ticks;
            }
            self.set(registers::PRESENT_SPEED, 0);
            self.set(registers::MOVING, 0);
        }
        self.actual_ticks = self.actual_ticks.clamp(0.0, (TICKS_PER_TURN - 1) as f64);
        self.set(registers::PRESENT_POSITION, self.present_ticks());
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

    pub fn servo(&self, id: u8) -> Option<&SimServo> {
        self.servos.iter().find(|s| s.id == id)
    }

    pub fn servo_mut(&mut self, id: u8) -> Option<&mut SimServo> {
        self.servos.iter_mut().find(|s| s.id == id)
    }

    pub fn settle(&mut self) {
        let now = self.clock.now();
        let hand = self.hand.as_ref();
        for s in &mut self.servos {
            s.step(now, hand);
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
                        self.reply(t, ServoError(0), &bytes);
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
                        self.reply(t, ServoError(0), &bytes);
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
