//! The SO-101 arm: what its joints and poses are, and `Arm`, which reads and commands it over its
//! STS3215 servo bus. Positions on `Arm` are in the calibrated coordinate; it's the only code
//! above `bus` that names registers.

pub mod bus;
pub mod calibration;
pub mod registers;
pub mod serial;
pub mod sim;
pub mod teleop;

use self::bus::{Error, Result, ServoBus, ServoError};
use self::calibration::Calibration;
use crate::{CheckpointId, MonotonicClock, Timestamp};
use std::time::Duration;

pub const JOINT_COUNT: usize = 6;

/// LeRobot's joint order and names: a dataset's columns depend on both.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(usize)]
pub enum Joint {
    ShoulderPan,
    ShoulderLift,
    ElbowFlex,
    WristFlex,
    WristRoll,
    Gripper,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Unit {
    Degrees,
    Percent,
}

impl Joint {
    pub const ALL: [Joint; JOINT_COUNT] =
        [Joint::ShoulderPan, Joint::ShoulderLift, Joint::ElbowFlex, Joint::WristFlex, Joint::WristRoll, Joint::Gripper];

    pub fn index(self) -> usize {
        self as usize
    }

    pub fn name(self) -> &'static str {
        match self {
            Joint::ShoulderPan => "shoulder_pan",
            Joint::ShoulderLift => "shoulder_lift",
            Joint::ElbowFlex => "elbow_flex",
            Joint::WristFlex => "wrist_flex",
            Joint::WristRoll => "wrist_roll",
            Joint::Gripper => "gripper",
        }
    }

    pub fn from_name(name: &str) -> Option<Joint> {
        Joint::ALL.iter().copied().find(|j| j.name() == name)
    }

    pub fn unit(self) -> Unit {
        match self {
            Joint::Gripper => Unit::Percent,
            _ => Unit::Degrees,
        }
    }
}

impl Unit {
    pub fn symbol(self) -> &'static str {
        match self {
            Unit::Degrees => "°",
            Unit::Percent => "%",
        }
    }
}

/// Where every joint is, each in its joint's unit.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose(pub [f32; JOINT_COUNT]);

impl std::ops::Index<Joint> for Pose {
    type Output = f32;
    fn index(&self, joint: Joint) -> &f32 {
        &self.0[joint.index()]
    }
}

/// Raw encoder counts, one per joint.
pub type Counts = [i32; JOINT_COUNT];

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Source {
    Teleop,
    Script(String),
    Policy(CheckpointId),
}

#[derive(Clone, Debug, PartialEq)]
pub struct JointState {
    pub t: Timestamp,
    /// Under the calibration stored in the log (contract 6).
    pub pose: Pose,
    /// `None` when not read from servos (a mock or a replay).
    pub counts: Option<Counts>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct JointTarget {
    pub t: Timestamp,
    pub pose: Pose,
    pub source: Source,
    /// `None` when not written to servos.
    pub counts: Option<Counts>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Fault {
    Voltage,
    Sensor,
    Overheat,
    Overcurrent,
    AngleLimit,
    Overload,
}

/// One joint's actuator.
#[derive(Clone, Debug, PartialEq)]
pub struct ActuatorStatus {
    pub t: Timestamp,
    pub joint: Joint,
    pub velocity_deg_s: Option<f32>,
    /// Of maximum torque, signed.
    pub load_percent: Option<f32>,
    pub voltage_v: Option<f32>,
    /// Single readings are occasionally junk; act on a median (contract 8).
    pub temperature_c: Option<f32>,
    pub current_a: Option<f32>,
    pub faults: Vec<Fault>,
}

/// Servo IDs on the bus, in `Joint::ALL` order.
pub const IDS: [u8; JOINT_COUNT] = [1, 2, 3, 4, 5, 6];

impl Joint {
    pub fn id(self) -> u8 {
        IDS[self.index()]
    }

    pub fn from_id(id: u8) -> Option<Joint> {
        Joint::ALL.iter().copied().find(|j| j.id() == id)
    }
}

// The follower's position-loop gains. LeRobot halves P from the factory 32 to
// 16 to avoid shake; higher P is stiffer and quicker to close a gap.
#[derive(Clone, Copy, Debug)]
pub struct FollowerGains {
    pub p: i32,
    pub d: i32,
    // Gripper drive ceiling, 0..1000 = 0..100%. It caps grip force and, on
    // these servos, top speed with it: 500 measured ~1600 counts/s against
    // ~3100 at full drive. Stall protection does not depend on it (current
    // limit, overload trip, and teleop's release all stay on).
    pub grip_torque: i32,
}

impl FollowerGains {
    pub const LEROBOT: FollowerGains = FollowerGains { p: 16, d: 32, grip_torque: 500 };
}

pub struct Arm {
    label: String,
    bus: ServoBus,
    calibration: Calibration,
}

impl Arm {
    pub fn new(label: &str, bus: ServoBus, calibration: Calibration) -> Self {
        Self { label: label.to_string(), bus, calibration }
    }

    pub fn label(&self) -> &str {
        &self.label
    }

    pub fn bus_name(&self) -> &str {
        self.bus.name()
    }

    pub fn calibration(&self) -> &Calibration {
        &self.calibration
    }

    pub fn check_present(&mut self) -> Result<()> {
        for joint in Joint::ALL {
            let identity = self.bus.identify(joint.id())?;
            if !identity.is_sts3215() {
                return Err(Error::UnexpectedModel { id: joint.id(), model: identity.model });
            }
        }
        Ok(())
    }

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

    // The register set LeRobot writes to a follower at connect time.
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
        self.bus.write_eeprom(grip, registers::MAX_TORQUE, gains.grip_torque)?;
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
        let present = self.read_counts()?;
        self.bus.sync_write(registers::GOAL_POSITION, &pairs(present))?;
        self.bus.sync_write(registers::TORQUE_ENABLE, &IDS.map(|id| (id, 1)))
    }

    // Declare the current pose to be centre. Returns the Homing_Offset each servo chose.
    pub fn recentre(&mut self) -> Result<Counts> {
        self.torque_off()?;
        let mut offsets = [0; JOINT_COUNT];
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

    pub fn read_counts(&mut self) -> Result<Counts> {
        let v = self.bus.sync_read(&IDS, registers::PRESENT_POSITION)?;
        Ok(std::array::from_fn(|i| v[i]))
    }

    pub fn read_pose(&mut self) -> Result<Pose> {
        let counts = self.read_counts()?;
        Ok(self.calibration.to_pose(&counts))
    }

    pub fn to_pose(&self, counts: &Counts) -> Pose {
        self.calibration.to_pose(counts)
    }

    pub fn to_counts(&self, pose: &Pose) -> Counts {
        self.calibration.to_counts(pose)
    }

    // Send goals, each pulled to within `max_step` of `from` when both are
    // given (teleop passes the previous goal). Returns what was actually sent.
    pub fn write_goal_counts(&mut self, goals: Counts, from: Option<Counts>, max_step: Option<i32>) -> Result<Counts> {
        let sent: Counts = match (from, max_step) {
            (Some(from), Some(max_step)) => {
                std::array::from_fn(|i| from[i] + (goals[i] - from[i]).clamp(-max_step, max_step))
            }
            _ => goals,
        };
        self.bus.sync_write(registers::GOAL_POSITION, &pairs(sent))?;
        Ok(sent)
    }

    pub fn read_actuators(&mut self) -> Result<Vec<ActuatorStatus>> {
        Joint::ALL.iter().map(|&joint| self.read_actuator(joint)).collect()
    }

    /// One block read covers every field.
    pub fn read_actuator(&mut self, joint: Joint) -> Result<ActuatorStatus> {
        let block = self.bus.read_block(joint.id(), STATUS_BLOCK_START, STATUS_BLOCK_LEN)?;
        decode_actuator(joint, MonotonicClock::read(), &block)
            .ok_or_else(|| Error::Corrupt(format!("short status block from servo {}", joint.id())))
    }

    pub fn take_flags(&mut self) -> [ServoError; JOINT_COUNT] {
        let mut out = [ServoError(0); JOINT_COUNT];
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

fn pairs(counts: Counts) -> [(u8, i32); JOINT_COUNT] {
    std::array::from_fn(|i| (IDS[i], counts[i]))
}

// Present_Position (56) through Present_Current (69..70): one 15-byte read.
pub const STATUS_BLOCK_START: u8 = 56;
pub const STATUS_BLOCK_LEN: u8 = 15;

fn decode_actuator(joint: Joint, t: Timestamp, block: &[u8]) -> Option<ActuatorStatus> {
    if block.len() < STATUS_BLOCK_LEN as usize {
        return None;
    }
    let at = |r: registers::Register| r.decode(&block[(r.address - STATUS_BLOCK_START) as usize..]);
    let status = ServoError(at(registers::STATUS) as u8);
    Some(ActuatorStatus {
        t,
        joint,
        velocity_deg_s: Some(at(registers::PRESENT_SPEED) as f32 * 360.0 / registers::COUNTS_PER_TURN as f32),
        // The register is in tenths of a percent.
        load_percent: Some(at(registers::PRESENT_LOAD) as f32 / 10.0),
        voltage_v: Some(at(registers::PRESENT_VOLTAGE) as f32 / 10.0),
        temperature_c: Some(at(registers::PRESENT_TEMPERATURE) as f32),
        // 6.5 mA per unit.
        current_a: Some(at(registers::PRESENT_CURRENT) as f32 * 0.0065),
        faults: faults(status),
    })
}

pub(crate) fn faults(status: ServoError) -> Vec<Fault> {
    [
        (ServoError::VOLTAGE, Fault::Voltage),
        (ServoError::ANGLE_SENSOR, Fault::Sensor),
        (ServoError::OVERHEAT, Fault::Overheat),
        (ServoError::OVERCURRENT, Fault::Overcurrent),
        (ServoError::ANGLE_LIMIT, Fault::AngleLimit),
        (ServoError::OVERLOAD, Fault::Overload),
    ]
    .into_iter()
    .filter(|(bit, _)| status.has(*bit))
    .map(|(_, f)| f)
    .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn index_and_name_round_trip() {
        for (i, j) in Joint::ALL.iter().enumerate() {
            assert_eq!(j.index(), i);
            assert_eq!(Joint::from_name(j.name()), Some(*j));
        }
    }

    #[test]
    fn only_the_gripper_is_in_percent() {
        for joint in Joint::ALL {
            let expected = if joint == Joint::Gripper { Unit::Percent } else { Unit::Degrees };
            assert_eq!(joint.unit(), expected);
        }
    }

    #[test]
    fn ids_round_trip() {
        for (i, joint) in Joint::ALL.iter().enumerate() {
            assert_eq!(joint.id(), i as u8 + 1);
            assert_eq!(Joint::from_id(joint.id()), Some(*joint));
        }
        assert_eq!(Joint::from_id(0), None);
        assert_eq!(Joint::from_id(7), None);
    }
}
