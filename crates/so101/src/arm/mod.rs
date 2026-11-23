//! The SO-101 arm: what its joints and poses are, and `ServoArm`, which reads and commands it over its
//! STS3215 servo servos. Positions on `ServoArm` are in the calibrated coordinate; it's the only code
//! above `servos` that names a servo register.

pub mod calibration;
pub mod servos;
pub mod simulated_servos;
pub mod uart;

use self::calibration::Calibration;
use self::servos::{Error, Result, ServoError, Servos};
use crate::{CheckpointId, DynError, MonotonicClock, Timestamp};
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

/// What the control loop can do with an arm. `ServoArm` is the SO-101 on its servos; a mock arm
/// joins it in Phase 1.
pub trait Arm: Send {
    fn read_state(&mut self) -> std::result::Result<JointState, DynError>;
    /// Faults the actuators reported since the last call, per joint. Reported with every reply,
    /// so this costs no extra reads.
    fn take_faults(&mut self) -> [Vec<Fault>; JOINT_COUNT];
    fn write_goal(&mut self, goal: &Counts) -> std::result::Result<(), DynError>;
    fn read_actuator(&mut self, joint: Joint) -> std::result::Result<ActuatorStatus, DynError>;
    fn to_pose(&self, counts: &Counts) -> Pose;
    /// Clamped to each joint's calibrated range.
    fn to_counts(&self, pose: &Pose) -> Counts;
    fn torque_off(&mut self) -> std::result::Result<(), DynError>;
    /// Torque on, holding where it is now, so nothing jumps.
    fn hold(&mut self) -> std::result::Result<(), DynError>;
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

/// Servo IDs, in `Joint::ALL` order.
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

pub struct ServoArm {
    label: String,
    servos: Servos,
    calibration: Calibration,
}

impl ServoArm {
    pub fn new(label: &str, servos: Servos, calibration: Calibration) -> Self {
        Self { label: label.to_string(), servos, calibration }
    }

    pub fn label(&self) -> &str {
        &self.label
    }

    pub fn port_name(&self) -> &str {
        self.servos.name()
    }

    pub fn calibration(&self) -> &Calibration {
        &self.calibration
    }

    pub fn check_present(&mut self) -> Result<()> {
        for joint in Joint::ALL {
            let identity = self.servos.identify(joint.id())?;
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
            let phase = self.servos.read(id, servos::PHASE)?;
            if phase & servos::PHASE_ANGLE_FEEDBACK != 0 {
                self.servos.write_eeprom(id, servos::PHASE, phase & !servos::PHASE_ANGLE_FEEDBACK)?;
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
            self.servos.write(id, servos::TORQUE_ENABLE, 0)?;
            self.servos.write_eeprom(id, servos::RETURN_DELAY, 0)?;
            self.servos.write_eeprom(id, servos::OPERATING_MODE, 0)?;
            self.servos.write_eeprom(id, servos::P_COEFFICIENT, gains.p)?;
            self.servos.write_eeprom(id, servos::I_COEFFICIENT, 0)?;
            self.servos.write_eeprom(id, servos::D_COEFFICIENT, gains.d)?;
            self.servos.write_unlocked(id, servos::MAXIMUM_ACCELERATION, 254)?;
            self.servos.write(id, servos::ACCELERATION, 254)?;
        }
        // Gripper burnout guard, LeRobot's values. Closing the leader trigger
        // drives the claw into itself; without these it stalls at full torque.
        let grip = Joint::Gripper.id();
        self.servos.write_eeprom(grip, servos::MAX_TORQUE, gains.grip_torque)?;
        self.servos.write_eeprom(grip, servos::PROTECTION_CURRENT, 250)?; // factory default 0 = off
        self.servos.write_eeprom(grip, servos::OVERLOAD_TORQUE, 25)?; // overload trips at 25%
        self.servos.write(grip, servos::TORQUE_LIMIT, gains.grip_torque)?; // RAM copy, in case 16 only loads at power-on
        Ok(())
    }

    pub fn torque_off(&mut self) -> Result<()> {
        self.servos.sync_write(servos::TORQUE_ENABLE, &IDS.map(|id| (id, 0)))
    }

    // Seeds every goal with the present position first so nothing jumps.
    pub fn torque_on_holding(&mut self) -> Result<()> {
        let present = self.read_counts()?;
        self.servos.sync_write(servos::GOAL_POSITION, &pairs(present))?;
        self.servos.sync_write(servos::TORQUE_ENABLE, &IDS.map(|id| (id, 1)))
    }

    // Declare the current pose to be centre. Returns the Homing_Offset each servo chose.
    pub fn recentre(&mut self) -> Result<Counts> {
        self.torque_off()?;
        let mut offsets = [0; JOINT_COUNT];
        for (i, id) in IDS.iter().enumerate() {
            self.servos.write(*id, servos::TORQUE_ENABLE, servos::TORQUE_RECENTER)?;
            offsets[i] = self.read_after_eeprom_write(*id, servos::HOMING_OFFSET)?;
        }
        Ok(offsets)
    }

    // note: after Torque_Enable = 128 the servo stores its new offset in EEPROM
    // and can stay silent for several ms. Retry a silent read for up to ~200 ms;
    // any other error returns at once.
    fn read_after_eeprom_write(&mut self, id: u8, reg: servos::Register) -> Result<i32> {
        let mut last = Error::NoReply { id };
        for _ in 0..20 {
            std::thread::sleep(Duration::from_millis(10));
            match self.servos.read(id, reg) {
                Ok(v) => return Ok(v),
                Err(e @ Error::NoReply { .. }) => last = e,
                Err(e) => return Err(e),
            }
        }
        Err(last)
    }

    pub fn read_counts(&mut self) -> Result<Counts> {
        let v = self.servos.sync_read(&IDS, servos::PRESENT_POSITION)?;
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

    pub fn write_goal(&mut self, goal: &Counts) -> Result<()> {
        self.servos.sync_write(servos::GOAL_POSITION, &pairs(*goal))
    }

    pub fn read_actuators(&mut self) -> Result<Vec<ActuatorStatus>> {
        Joint::ALL.iter().map(|&joint| self.read_actuator(joint)).collect()
    }

    /// One block read covers every field.
    pub fn read_actuator(&mut self, joint: Joint) -> Result<ActuatorStatus> {
        let block = self.servos.read_block(joint.id(), STATUS_BLOCK_START, STATUS_BLOCK_LEN)?;
        decode_actuator(joint, MonotonicClock::read(), &block)
            .ok_or_else(|| Error::Corrupt(format!("short status block from servo {}", joint.id())))
    }

    pub fn take_flags(&mut self) -> [ServoError; JOINT_COUNT] {
        let mut out = [ServoError(0); JOINT_COUNT];
        for (id, e) in self.servos.take_flags() {
            if let Some(j) = Joint::from_id(id) {
                out[j.index()].0 |= e.0;
            }
        }
        out
    }

    // Raw register access for tests and diagnostics.
    pub fn read_register(&mut self, joint: Joint, reg: servos::Register) -> Result<i32> {
        self.servos.read(joint.id(), reg)
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
    let at = |r: servos::Register| r.decode(&block[(r.address - STATUS_BLOCK_START) as usize..]);
    let status = ServoError(at(servos::STATUS) as u8);
    Some(ActuatorStatus {
        t,
        joint,
        velocity_deg_s: Some(at(servos::PRESENT_SPEED) as f32 * 360.0 / servos::COUNTS_PER_TURN as f32),
        // The register is in tenths of a percent.
        load_percent: Some(at(servos::PRESENT_LOAD) as f32 / 10.0),
        voltage_v: Some(at(servos::PRESENT_VOLTAGE) as f32 / 10.0),
        temperature_c: Some(at(servos::PRESENT_TEMPERATURE) as f32),
        // 6.5 mA per unit.
        current_a: Some(at(servos::PRESENT_CURRENT) as f32 * 0.0065),
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

impl Arm for ServoArm {
    fn read_state(&mut self) -> std::result::Result<JointState, DynError> {
        let counts = self.read_counts()?;
        Ok(JointState { t: MonotonicClock::read(), pose: self.to_pose(&counts), counts: Some(counts) })
    }

    fn take_faults(&mut self) -> [Vec<Fault>; JOINT_COUNT] {
        self.take_flags().map(faults)
    }

    fn write_goal(&mut self, goal: &Counts) -> std::result::Result<(), DynError> {
        Ok(ServoArm::write_goal(self, goal)?)
    }

    fn read_actuator(&mut self, joint: Joint) -> std::result::Result<ActuatorStatus, DynError> {
        Ok(ServoArm::read_actuator(self, joint)?)
    }

    fn to_pose(&self, counts: &Counts) -> Pose {
        self.calibration.to_pose(counts)
    }

    fn to_counts(&self, pose: &Pose) -> Counts {
        self.calibration.to_counts(pose)
    }

    fn torque_off(&mut self) -> std::result::Result<(), DynError> {
        Ok(ServoArm::torque_off(self)?)
    }

    fn hold(&mut self) -> std::result::Result<(), DynError> {
        Ok(self.torque_on_holding()?)
    }
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
