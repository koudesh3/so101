//! One arm: six joints on one bus, addressed by `Joint`, with positions in
//! the calibrated coordinate. The only module above `bus` that names registers.

use std::time::Duration;

use crate::device::bus::Bus;
use crate::robot::calibration::ArmCalibration;
use crate::error::{Error, Result};
use crate::robot::joint::{Joint, Ticks, Values, COUNT, IDS};
use crate::wire::protocol::ServoError;
use crate::wire::registers;

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
    pub fn configure_follower(&mut self) -> Result<()> {
        for id in IDS {
            self.bus.write(id, registers::TORQUE_ENABLE, 0)?;
            self.bus.write_eeprom(id, registers::RETURN_DELAY, 0)?;
            self.bus.write_eeprom(id, registers::OPERATING_MODE, 0)?;
            self.bus.write_eeprom(id, registers::P_COEFFICIENT, 16)?;
            self.bus.write_eeprom(id, registers::I_COEFFICIENT, 0)?;
            self.bus.write_eeprom(id, registers::D_COEFFICIENT, 32)?;
            self.bus.write(id, registers::ACCELERATION, 254)?;
        }
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
            std::thread::sleep(Duration::from_millis(5));
            offsets[i] = self.bus.read(*id, registers::HOMING_OFFSET)?;
        }
        Ok(offsets)
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

    // Send goals, each pulled to within `max_step` of `current` when both are
    // given. Returns what was actually sent.
    pub fn write_goal_ticks(&mut self, goals: Ticks, current: Option<Ticks>, max_step: Option<i32>) -> Result<Ticks> {
        let sent: Ticks = match (current, max_step) {
            (Some(current), Some(max_step)) => {
                std::array::from_fn(|i| current[i] + (goals[i] - current[i]).clamp(-max_step, max_step))
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
            });
        }
        Ok(out)
    }

    // Raw register access for tests and diagnostics.
    pub fn read_register(&mut self, joint: Joint, reg: registers::Register) -> Result<i32> {
        self.bus.read(joint.id(), reg)
    }
}

fn pairs(ticks: Ticks) -> [(u8, i32); COUNT] {
    std::array::from_fn(|i| (IDS[i], ticks[i]))
}
