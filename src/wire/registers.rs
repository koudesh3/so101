//! The STS3215 control table as data.
//!
//! note: addresses below 40 are EEPROM. They persist across power cycles
//! and the servo silently ignores writes to them while Lock = 1.

use crate::wire::protocol::{decode_sign_magnitude, encode_sign_magnitude};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Encoding {
    Unsigned,
    SignBit(u8),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Access {
    ReadOnly,
    ReadWrite,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Register {
    pub name: &'static str,
    pub address: u8,
    pub width: u8,
    pub access: Access,
    pub encoding: Encoding,
}

pub const EEPROM_END: u8 = 40;

pub fn is_eeprom_address(address: u8) -> bool {
    address < EEPROM_END
}

impl Register {
    pub const fn new(name: &'static str, address: u8, width: u8, access: Access, encoding: Encoding) -> Self {
        Self { name, address, width, access, encoding }
    }

    pub fn is_eeprom(&self) -> bool {
        is_eeprom_address(self.address)
    }

    pub fn is_writable(&self) -> bool {
        self.access == Access::ReadWrite
    }

    pub fn encode(&self, value: i32) -> Vec<u8> {
        let raw: u16 = match self.encoding {
            Encoding::Unsigned => value as u16,
            Encoding::SignBit(bit) => encode_sign_magnitude(value, bit),
        };
        match self.width {
            1 => vec![raw as u8],
            2 => raw.to_le_bytes().to_vec(),
            w => panic!("unsupported register width {w}"),
        }
    }

    pub fn decode(&self, bytes: &[u8]) -> i32 {
        let raw: u16 = match self.width {
            1 => bytes[0] as u16,
            2 => u16::from_le_bytes([bytes[0], bytes[1]]),
            w => panic!("unsupported register width {w}"),
        };
        match self.encoding {
            Encoding::Unsigned => raw as i32,
            Encoding::SignBit(bit) => decode_sign_magnitude(raw, bit),
        }
    }
}

use Access::*;
use Encoding::*;

pub const MODEL_NUMBER: Register = Register::new("Model_Number", 3, 2, ReadOnly, Unsigned);
pub const ID: Register = Register::new("ID", 5, 1, ReadWrite, Unsigned);
pub const BAUD_RATE: Register = Register::new("Baud_Rate", 6, 1, ReadWrite, Unsigned);
pub const RETURN_DELAY: Register = Register::new("Return_Delay", 7, 1, ReadWrite, Unsigned);
pub const RESPONSE_LEVEL: Register = Register::new("Response_Level", 8, 1, ReadWrite, Unsigned);
pub const MIN_ANGLE_LIMIT: Register = Register::new("Min_Angle_Limit", 9, 2, ReadWrite, Unsigned);
pub const MAX_ANGLE_LIMIT: Register = Register::new("Max_Angle_Limit", 11, 2, ReadWrite, Unsigned);
pub const MAX_TEMPERATURE: Register = Register::new("Max_Temperature", 13, 1, ReadWrite, Unsigned);
pub const MAX_TORQUE: Register = Register::new("Max_Torque", 16, 2, ReadWrite, Unsigned);
pub const PHASE: Register = Register::new("Phase", 18, 1, ReadWrite, Unsigned);
pub const P_COEFFICIENT: Register = Register::new("P_Coefficient", 21, 1, ReadWrite, Unsigned);
pub const D_COEFFICIENT: Register = Register::new("D_Coefficient", 22, 1, ReadWrite, Unsigned);
pub const I_COEFFICIENT: Register = Register::new("I_Coefficient", 23, 1, ReadWrite, Unsigned);
pub const HOMING_OFFSET: Register = Register::new("Homing_Offset", 31, 2, ReadWrite, SignBit(11));
pub const OPERATING_MODE: Register = Register::new("Operating_Mode", 33, 1, ReadWrite, Unsigned);

pub const TORQUE_ENABLE: Register = Register::new("Torque_Enable", 40, 1, ReadWrite, Unsigned);
pub const ACCELERATION: Register = Register::new("Acceleration", 41, 1, ReadWrite, Unsigned);
pub const GOAL_POSITION: Register = Register::new("Goal_Position", 42, 2, ReadWrite, SignBit(15));
pub const GOAL_TIME: Register = Register::new("Goal_Time", 44, 2, ReadWrite, Unsigned);
pub const GOAL_SPEED: Register = Register::new("Goal_Speed", 46, 2, ReadWrite, SignBit(15));
pub const TORQUE_LIMIT: Register = Register::new("Torque_Limit", 48, 2, ReadWrite, Unsigned);
pub const LOCK: Register = Register::new("Lock", 55, 1, ReadWrite, Unsigned);
pub const PRESENT_POSITION: Register = Register::new("Present_Position", 56, 2, ReadOnly, SignBit(15));
pub const PRESENT_SPEED: Register = Register::new("Present_Speed", 58, 2, ReadOnly, SignBit(15));
pub const PRESENT_LOAD: Register = Register::new("Present_Load", 60, 2, ReadOnly, Unsigned);
pub const PRESENT_VOLTAGE: Register = Register::new("Present_Voltage", 62, 1, ReadOnly, Unsigned);
pub const PRESENT_TEMPERATURE: Register = Register::new("Present_Temperature", 63, 1, ReadOnly, Unsigned);
pub const STATUS: Register = Register::new("Status", 65, 1, ReadOnly, Unsigned);
pub const MOVING: Register = Register::new("Moving", 66, 1, ReadOnly, Unsigned);
pub const PRESENT_CURRENT: Register = Register::new("Present_Current", 69, 2, ReadOnly, Unsigned);

pub const ALL: &[Register] = &[
    MODEL_NUMBER, ID, BAUD_RATE, RETURN_DELAY, RESPONSE_LEVEL, MIN_ANGLE_LIMIT, MAX_ANGLE_LIMIT,
    MAX_TEMPERATURE, MAX_TORQUE, PHASE, P_COEFFICIENT, D_COEFFICIENT, I_COEFFICIENT, HOMING_OFFSET,
    OPERATING_MODE, TORQUE_ENABLE, ACCELERATION, GOAL_POSITION, GOAL_TIME, GOAL_SPEED, TORQUE_LIMIT,
    LOCK, PRESENT_POSITION, PRESENT_SPEED, PRESENT_LOAD, PRESENT_VOLTAGE, PRESENT_TEMPERATURE, STATUS,
    MOVING, PRESENT_CURRENT,
];

pub fn by_address(address: u8) -> Option<Register> {
    ALL.iter().copied().find(|r| r.address == address)
}

pub const STS3215_MODEL_NUMBER: i32 = 777;
// note: writing 128 to Torque_Enable makes the servo store a Homing_Offset
// such that its current angle reads as 2048, then leaves torque off.
pub const TORQUE_RECENTER: i32 = 128;
pub const TICKS_PER_TURN: i32 = 4096;
pub const CENTER_TICK: i32 = 2048;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn goal_position_encodes_with_sign_bit_15() {
        assert_eq!(GOAL_POSITION.encode(2048), vec![0x00, 0x08]);
        assert_eq!(GOAL_POSITION.encode(-5), vec![0x05, 0x80]);
        assert_eq!(GOAL_POSITION.decode(&[0x05, 0x80]), -5);
    }

    #[test]
    fn homing_offset_uses_bit_11() {
        assert_eq!(HOMING_OFFSET.encode(-100), vec![0x64, 0x08]);
        assert_eq!(HOMING_OFFSET.decode(&[0x64, 0x08]), -100);
    }

    #[test]
    fn eeprom_boundary() {
        assert!(OPERATING_MODE.is_eeprom());
        assert!(!TORQUE_ENABLE.is_eeprom());
        assert!(!PRESENT_POSITION.is_writable());
    }
}
