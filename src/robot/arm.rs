//! One arm: six joints on one bus, addressed by `Joint`, with positions in
//! the calibrated coordinate. The only module above `bus` that names registers.

use std::time::Duration;

use crate::device::bus::Bus;
use crate::robot::calibration::ArmCalibration;
use crate::error::{Error, Result};
use crate::robot::joint::{Joint, Ticks, Values, COUNT, IDS};
use crate::wire::protocol::ServoError;
use crate::wire::registers;

// The follower's position-loop gains. LeRobot halves P from the factory 32 to
// 16 to avoid shake; higher P is stiffer and quicker to close a gap.
#[derive(Clone, Copy, Debug)]
pub struct FollowerGains {
    pub p: i32,
    pub d: i32,
    // Gripper drive ceiling, 0..1000 = 0..100%. It caps grip force and, on
    // these servos, top speed with it: 500 measured ~1600 ticks/s against
    // ~3100 at full drive. Stall protection does not depend on it (current
    // limit, overload trip, and teleop's release all stay on).
    pub grip_torque: i32,
}

impl FollowerGains {
    pub const LEROBOT: FollowerGains = FollowerGains { p: 16, d: 32, grip_torque: 500 };
}

pub struct Arm {
    label: String,
    bus: Bus,
    calibration: ArmCalibration,
}

#[derive(Clone, Copy, Debug)]
pub struct Telemetry {
    pub joint: Joint,
    pub position_ticks: i32,
    pub speed: i32,
    pub load: i32,
    pub voltage_tenths: i32,
    pub temperature_c: i32,
    pub status: ServoError,
    pub moving: bool,
    // Raw Present_Current (0 when not read).
    pub current: i32,
}

// Present_Position (56) through Present_Current (69..70): one 15-byte read.
pub const TELEMETRY_START: u8 = 56;
pub const TELEMETRY_LEN: u8 = 15;

impl Telemetry {
    // Decode the block that starts at Present_Position.
    pub fn from_block(joint: Joint, b: &[u8]) -> Option<Self> {
        if b.len() < TELEMETRY_LEN as usize {
            return None;
        }
        let at = |r: registers::Register| r.decode(&b[(r.address - TELEMETRY_START) as usize..]);
        Some(Self {
            joint,
            position_ticks: at(registers::PRESENT_POSITION),
            speed: at(registers::PRESENT_SPEED),
            load: at(registers::PRESENT_LOAD),
            voltage_tenths: at(registers::PRESENT_VOLTAGE),
            temperature_c: at(registers::PRESENT_TEMPERATURE),
            status: ServoError(at(registers::STATUS) as u8),
            moving: at(registers::MOVING) != 0,
            current: at(registers::PRESENT_CURRENT),
        })
    }
}

impl Arm {
    pub fn new(label: &str, bus: Bus, calibration: ArmCalibration) -> Self {
        Self { label: label.to_string(), bus, calibration }
    }

    pub fn label(&self) -> &str {
        &self.label
    }

    pub fn bus_name(&self) -> &str {
        self.bus.name()
    }

    pub fn calibration(&self) -> &ArmCalibration {
        &self.calibration
    }

    // Every joint answers and is an STS3215.
    pub fn check_present(&mut self) -> Result<()> {
        for joint in Joint::ALL {
            let identity = self.bus.identify(joint.id())?;
            if !identity.is_sts3215() {
                return Err(Error::UnexpectedModel { id: joint.id(), model: identity.model });
            }
        }
        Ok(())
    }

    // The register set LeRobot writes to a follower at connect time.
    // What LeRobot writes to every arm at connect: position readings that never
    // wrap. Returns how many servos had the wrap bit set and were changed.
    pub fn configure_readings(&mut self) -> Result<usize> {
        let mut changed = 0;
        for id in IDS {
            let phase = self.bus.read(id, registers::PHASE)?;
            if phase & registers::PHASE_ANGLE_FEEDBACK != 0 {
                self.bus.write_eeprom(id, registers::PHASE, phase & !registers::PHASE_ANGLE_FEEDBACK)?;
                changed += 1;
            }
        }
        Ok(changed)
    }

    pub fn configure_follower(&mut self) -> Result<()> {
        self.configure_follower_with(FollowerGains::LEROBOT)
    }

    pub fn configure_follower_with(&mut self, gains: FollowerGains) -> Result<()> {
        for id in IDS {
            self.bus.write(id, registers::TORQUE_ENABLE, 0)?;
            self.bus.write_eeprom(id, registers::RETURN_DELAY, 0)?;
            self.bus.write_eeprom(id, registers::OPERATING_MODE, 0)?;
            self.bus.write_eeprom(id, registers::P_COEFFICIENT, gains.p)?;
            self.bus.write_eeprom(id, registers::I_COEFFICIENT, 0)?;
            self.bus.write_eeprom(id, registers::D_COEFFICIENT, gains.d)?;
            self.bus.write_unlocked(id, registers::MAXIMUM_ACCELERATION, 254)?;
            self.bus.write(id, registers::ACCELERATION, 254)?;
        }
        // Gripper burnout guard, LeRobot's values. Closing the leader trigger
        // drives the claw into itself; without these it stalls at full torque.
        let grip = Joint::Gripper.id();
        self.bus.write_eeprom(grip, registers::MAX_TORQUE, gains.grip_torque)?; // drive ceiling
        self.bus.write_eeprom(grip, registers::PROTECTION_CURRENT, 250)?; // factory default 0 = off
        self.bus.write_eeprom(grip, registers::OVERLOAD_TORQUE, 25)?; // overload trips at 25%
        self.bus.write(grip, registers::TORQUE_LIMIT, gains.grip_torque)?; // RAM copy, in case 16 only loads at power-on
        Ok(())
    }

    pub fn torque_off(&mut self) -> Result<()> {
        self.bus.sync_write(registers::TORQUE_ENABLE, &IDS.map(|id| (id, 0)))
    }

    // Seeds every goal with the present position first so nothing jumps.
    pub fn torque_on_holding(&mut self) -> Result<()> {
        let present = self.read_ticks()?;
        self.bus.sync_write(registers::GOAL_POSITION, &pairs(present))?;
        self.bus.sync_write(registers::TORQUE_ENABLE, &IDS.map(|id| (id, 1)))
    }

    // Declare the current pose to be centre. Returns the Homing_Offset each servo chose.
    pub fn recentre(&mut self) -> Result<Ticks> {
        self.torque_off()?;
        let mut offsets = [0; COUNT];
        for (i, id) in IDS.iter().enumerate() {
            self.bus.write(*id, registers::TORQUE_ENABLE, registers::TORQUE_RECENTER)?;
            offsets[i] = self.read_after_eeprom_write(*id, registers::HOMING_OFFSET)?;
        }
        Ok(offsets)
    }

    // note: after Torque_Enable = 128 the servo stores its new offset in EEPROM
    // and can stay silent for several ms. Retry a silent read for up to ~200 ms;
    // any other error returns at once.
    fn read_after_eeprom_write(&mut self, id: u8, reg: registers::Register) -> Result<i32> {
        let mut last = Error::NoReply { id };
        for _ in 0..20 {
            std::thread::sleep(Duration::from_millis(10));
            match self.bus.read(id, reg) {
                Ok(v) => return Ok(v),
                Err(e @ Error::NoReply { .. }) => last = e,
                Err(e) => return Err(e),
            }
        }
        Err(last)
    }

    pub fn read_ticks(&mut self) -> Result<Ticks> {
        let v = self.bus.sync_read(&IDS, registers::PRESENT_POSITION)?;
        Ok(std::array::from_fn(|i| v[i]))
    }

    pub fn read_values(&mut self) -> Result<Values> {
        let ticks = self.read_ticks()?;
        Ok(self.calibration.ticks_to_values(&ticks))
    }

    pub fn ticks_to_values(&self, ticks: &Ticks) -> Values {
        self.calibration.ticks_to_values(ticks)
    }

    pub fn values_to_ticks(&self, values: &Values) -> Ticks {
        self.calibration.values_to_ticks(values)
    }

    // Send goals, each pulled to within `max_step` of `from` when both are
    // given (teleop passes the previous goal). Returns what was actually sent.
    pub fn write_goal_ticks(&mut self, goals: Ticks, from: Option<Ticks>, max_step: Option<i32>) -> Result<Ticks> {
        let sent: Ticks = match (from, max_step) {
            (Some(from), Some(max_step)) => {
                std::array::from_fn(|i| from[i] + (goals[i] - from[i]).clamp(-max_step, max_step))
            }
            _ => goals,
        };
        self.bus.sync_write(registers::GOAL_POSITION, &pairs(sent))?;
        Ok(sent)
    }

    pub fn read_telemetry(&mut self) -> Result<Vec<Telemetry>> {
        let mut out = Vec::with_capacity(COUNT);
        for joint in Joint::ALL {
            let id = joint.id();
            out.push(Telemetry {
                joint,
                position_ticks: self.bus.read(id, registers::PRESENT_POSITION)?,
                speed: self.bus.read(id, registers::PRESENT_SPEED)?,
                load: self.bus.read(id, registers::PRESENT_LOAD)?,
                voltage_tenths: self.bus.read(id, registers::PRESENT_VOLTAGE)?,
                temperature_c: self.bus.read(id, registers::PRESENT_TEMPERATURE)?,
                status: ServoError(self.bus.read(id, registers::STATUS)? as u8),
                moving: self.bus.read(id, registers::MOVING)? != 0,
                current: self.bus.read(id, registers::PRESENT_CURRENT)?,
            });
        }
        Ok(out)
    }

    // Everything worth watching from one servo, in one round trip.
    pub fn read_telemetry_block(&mut self, joint: Joint) -> Result<Telemetry> {
        let bytes = self.bus.read_block(joint.id(), TELEMETRY_START, TELEMETRY_LEN)?;
        Telemetry::from_block(joint, &bytes).ok_or_else(|| Error::Corrupt(format!("short telemetry block from {}", joint.id())))
    }

    // Advisory status bits each joint raised since the last call.
    pub fn take_flags(&mut self) -> [ServoError; COUNT] {
        let mut out = [ServoError(0); COUNT];
        for (id, e) in self.bus.take_flags() {
            if let Some(j) = Joint::from_id(id) {
                out[j.index()].0 |= e.0;
            }
        }
        out
    }

    // Raw register access for tests and diagnostics.
    pub fn read_register(&mut self, joint: Joint, reg: registers::Register) -> Result<i32> {
        self.bus.read(joint.id(), reg)
    }
}

fn pairs(ticks: Ticks) -> [(u8, i32); COUNT] {
    std::array::from_fn(|i| (IDS[i], ticks[i]))
}
