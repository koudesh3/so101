//! `ServoArm`: the SO-101 on its STS3215 servos, the real `Arm`. Which servo is which joint, the
//! follower's gains, reading and writing through `Servos`, decoding actuator status. Positions here
//! are in the calibrated coordinate; it's the only code above `servos` that names a register.

use std::sync::Arc;
use std::time::Duration;

use super::calibration::Calibration;
use super::servos::{self, ServoError, Servos};
use super::{ActuatorStatus, Arm, Counts, Error, Fault, Joint, JointState, Pose, Result, JOINT_COUNT};
use crate::clock::{Clock, MonotonicClock, Timestamp};

/// Servo IDs, in `Joint::ALL` order.
pub const IDS: [u8; JOINT_COUNT] = [1, 2, 3, 4, 5, 6];

// Present_Position (56) through Present_Current (69..70): one 15-byte read.
pub const STATUS_BLOCK_START: u8 = 56;

pub const STATUS_BLOCK_LEN: u8 = 15;

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

pub struct ServoArm {
    label: String,
    servos: Servos,
    calibration: Calibration,
    clock: Arc<dyn Clock>,
}

impl Joint {
    pub fn id(self) -> u8 {
        IDS[self.index()]
    }

    pub fn from_id(id: u8) -> Option<Joint> {
        Joint::ALL.iter().copied().find(|j| j.id() == id)
    }
}

impl FollowerGains {
    pub const LEROBOT: FollowerGains = FollowerGains { p: 16, d: 32, grip_torque: 500 };
}

impl ServoArm {
    pub fn new(label: &str, servos: Servos, calibration: Calibration) -> Self {
        Self { label: label.to_string(), servos, calibration, clock: Arc::new(MonotonicClock) }
    }

    /// Stamps readings on `clock` instead of the host's monotonic clock.
    pub fn with_clock(mut self, clock: Arc<dyn Clock>) -> Self {
        self.clock = clock;
        self
    }

    pub fn clock(&self) -> Arc<dyn Clock> {
        self.clock.clone()
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
        decode_actuator(joint, self.clock.now(), &block)
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

impl Arm for ServoArm {
    fn read_state(&mut self) -> Result<JointState> {
        let counts = self.read_counts()?;
        Ok(JointState { t: self.clock.now(), pose: self.to_pose(&counts), counts: Some(counts) })
    }

    fn take_faults(&mut self) -> [Vec<Fault>; JOINT_COUNT] {
        self.take_flags().map(faults)
    }

    fn write_goal(&mut self, goal: &Counts) -> Result<()> {
        ServoArm::write_goal(self, goal)
    }

    fn read_actuator(&mut self, joint: Joint) -> Result<ActuatorStatus> {
        ServoArm::read_actuator(self, joint)
    }

    fn to_pose(&self, counts: &Counts) -> Pose {
        self.calibration.to_pose(counts)
    }

    fn to_counts(&self, pose: &Pose) -> Counts {
        self.calibration.to_counts(pose)
    }

    fn torque_off(&mut self) -> Result<()> {
        ServoArm::torque_off(self)
    }

    fn hold(&mut self) -> Result<()> {
        self.torque_on_holding()
    }
}

fn pairs(counts: Counts) -> [(u8, i32); JOINT_COUNT] {
    std::array::from_fn(|i| (IDS[i], counts[i]))
}

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

#[cfg(test)]
mod tests {
    use super::*;

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
