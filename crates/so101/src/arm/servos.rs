//! `Servos`: the Feetech STS3215 servos on one half-duplex UART, addressed by ID: packets,
//! checksums, transactions, the status flags every reply carries, the errors, and the servo's
//! control table (its registers).

use std::fmt;
use std::time::Duration;

use crate::arm::Transport;
use crate::arm::{Error, Result};
use Access::*;
use Encoding::*;

pub const HEADER: [u8; 2] = [0xFF, 0xFF];
pub const BROADCAST_ID: u8 = 0xFE;
pub const MAX_SERVO_ID: u8 = 0xFD;

// How long to wait for the ack of a write that commits to non-volatile memory.
// Up to ~40 ms has been measured; allow more than double.
pub const EEPROM_ACK_TIMEOUT: Duration = Duration::from_millis(100);

pub const EEPROM_END: u8 = 40;
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

// Phase bit 4: angle feedback mode. Cleared keeps Present_Position in 0..4095
// instead of wrapping signed (LeRobot clears it on every STS3215).
pub const PHASE_ANGLE_FEEDBACK: i32 = 0x10;

pub const P_COEFFICIENT: Register = Register::new("P_Coefficient", 21, 1, ReadWrite, Unsigned);
pub const D_COEFFICIENT: Register = Register::new("D_Coefficient", 22, 1, ReadWrite, Unsigned);
pub const I_COEFFICIENT: Register = Register::new("I_Coefficient", 23, 1, ReadWrite, Unsigned);
pub const PROTECTION_CURRENT: Register = Register::new("Protection_Current", 28, 2, ReadWrite, Unsigned);
pub const OVERLOAD_TORQUE: Register = Register::new("Overload_Torque", 36, 1, ReadWrite, Unsigned);
pub const HOMING_OFFSET: Register = Register::new("Homing_Offset", 31, 2, ReadWrite, SignBit(11));
pub const OPERATING_MODE: Register = Register::new("Operating_Mode", 33, 1, ReadWrite, Unsigned);
pub const TORQUE_ENABLE: Register = Register::new("Torque_Enable", 40, 1, ReadWrite, Unsigned);
pub const ACCELERATION: Register = Register::new("Acceleration", 41, 1, ReadWrite, Unsigned);
pub const GOAL_POSITION: Register = Register::new("Goal_Position", 42, 2, ReadWrite, SignBit(15));
pub const GOAL_TIME: Register = Register::new("Goal_Time", 44, 2, ReadWrite, Unsigned);
pub const GOAL_SPEED: Register = Register::new("Goal_Speed", 46, 2, ReadWrite, SignBit(15));
pub const TORQUE_LIMIT: Register = Register::new("Torque_Limit", 48, 2, ReadWrite, Unsigned);

// Factory area, behind Lock like EEPROM. LeRobot sets 254 for fastest ramps.
pub const MAXIMUM_ACCELERATION: Register = Register::new("Maximum_Acceleration", 85, 1, ReadWrite, Unsigned);

pub const LOCK: Register = Register::new("Lock", 55, 1, ReadWrite, Unsigned);
pub const PRESENT_POSITION: Register = Register::new("Present_Position", 56, 2, ReadOnly, SignBit(15));
pub const PRESENT_SPEED: Register = Register::new("Present_Speed", 58, 2, ReadOnly, SignBit(15));

// Direction in bit 10, magnitude 0..1000 = 0..100% (LeRobot's encoding table).
pub const PRESENT_LOAD: Register = Register::new("Present_Load", 60, 2, ReadOnly, SignBit(10));

pub const PRESENT_VOLTAGE: Register = Register::new("Present_Voltage", 62, 1, ReadOnly, Unsigned);
pub const PRESENT_TEMPERATURE: Register = Register::new("Present_Temperature", 63, 1, ReadOnly, Unsigned);
pub const STATUS: Register = Register::new("Status", 65, 1, ReadOnly, Unsigned);
pub const MOVING: Register = Register::new("Moving", 66, 1, ReadOnly, Unsigned);
pub const PRESENT_CURRENT: Register = Register::new("Present_Current", 69, 2, ReadOnly, Unsigned);

pub const ALL: &[Register] = &[
    MODEL_NUMBER,
    ID,
    BAUD_RATE,
    RETURN_DELAY,
    RESPONSE_LEVEL,
    MIN_ANGLE_LIMIT,
    MAX_ANGLE_LIMIT,
    MAX_TEMPERATURE,
    MAX_TORQUE,
    PHASE,
    P_COEFFICIENT,
    D_COEFFICIENT,
    I_COEFFICIENT,
    PROTECTION_CURRENT,
    OVERLOAD_TORQUE,
    HOMING_OFFSET,
    MAXIMUM_ACCELERATION,
    OPERATING_MODE,
    TORQUE_ENABLE,
    ACCELERATION,
    GOAL_POSITION,
    GOAL_TIME,
    GOAL_SPEED,
    TORQUE_LIMIT,
    LOCK,
    PRESENT_POSITION,
    PRESENT_SPEED,
    PRESENT_LOAD,
    PRESENT_VOLTAGE,
    PRESENT_TEMPERATURE,
    STATUS,
    MOVING,
    PRESENT_CURRENT,
];

pub const STS3215_MODEL_NUMBER: i32 = 777;

// note: writing 128 to Torque_Enable makes the servo store a Homing_Offset
// such that its current angle reads as 2048, then leaves torque off.
pub const TORQUE_RECENTER: i32 = 128;

pub const COUNTS_PER_TURN: i32 = 4096;
pub const CENTER_COUNT: i32 = 2048;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Instruction {
    Ping = 0x01,
    Read = 0x02,
    Write = 0x03,
    RegWrite = 0x04,
    Action = 0x05,
    SyncRead = 0x82,
    SyncWrite = 0x83,
}

// Fault bits from a status packet's error byte or the Status register.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct ServoError(pub u8);

// A parsed frame. The fourth byte is an instruction host-to-servo and an
// error byte servo-to-host; the caller knows which direction it came from.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Packet {
    pub id: u8,
    pub fourth_byte: u8,
    pub params: Vec<u8>,
}

#[derive(Debug, PartialEq, Eq)]
pub enum ParseError {
    // At least this many more bytes are needed.
    Incomplete(usize),
    // No header at the front; discard `skip` bytes and try again.
    NoHeader { skip: usize },
    BadChecksum { expected: u8, found: u8 },
}

pub struct Servos {
    transport: Box<dyn Transport>,
    pub reply_timeout: Duration,
    // Advisory status bits seen since the last `take_flags`, per replying id.
    flags: Vec<(u8, ServoError)>,
}

#[derive(Clone, Copy, Debug)]
pub struct ServoIdentity {
    pub id: u8,
    pub model: i32,
    pub present_position: i32,
}

//
// note: addresses below 40 are EEPROM. They persist across power cycles
// and the servo silently ignores writes to them while Lock = 1.
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

impl Instruction {
    pub fn from_byte(byte: u8) -> Option<Self> {
        Some(match byte {
            0x01 => Self::Ping,
            0x02 => Self::Read,
            0x03 => Self::Write,
            0x04 => Self::RegWrite,
            0x05 => Self::Action,
            0x82 => Self::SyncRead,
            0x83 => Self::SyncWrite,
            _ => return None,
        })
    }
}

impl ServoError {
    pub const VOLTAGE: u8 = 1 << 0;
    pub const ANGLE_SENSOR: u8 = 1 << 1;
    pub const OVERHEAT: u8 = 1 << 2;
    pub const OVERCURRENT: u8 = 1 << 3;
    pub const ANGLE_LIMIT: u8 = 1 << 4;
    pub const OVERLOAD: u8 = 1 << 5;
    // Bits a servo raises while still answering with valid data: a supply dip,
    // or its own overload or current protection.
    pub const ADVISORY: u8 = Self::VOLTAGE | Self::OVERLOAD | Self::OVERCURRENT;

    pub fn is_advisory(self) -> bool {
        self.0 != 0 && self.0 & !Self::ADVISORY == 0
    }

    pub fn has(self, bit: u8) -> bool {
        self.0 & bit != 0
    }

    pub fn is_healthy(self) -> bool {
        self.0 == 0
    }
}

impl Packet {
    pub fn instruction(&self) -> Option<Instruction> {
        Instruction::from_byte(self.fourth_byte)
    }
    pub fn error(&self) -> ServoError {
        ServoError(self.fourth_byte)
    }
    pub fn wire_size(&self) -> usize {
        self.params.len() + 6
    }
}

impl ServoIdentity {
    pub fn is_sts3215(&self) -> bool {
        self.model == STS3215_MODEL_NUMBER
    }
}

impl Servos {
    pub fn new(transport: Box<dyn Transport>) -> Self {
        Self { transport, reply_timeout: Duration::from_millis(10), flags: Vec::new() }
    }

    pub fn name(&self) -> &str {
        self.transport.name()
    }

    fn send(&mut self, id: u8, instruction: Instruction, params: &[u8]) -> Result<()> {
        self.transport.discard_input()?;
        self.transport.send(&encode_instruction(id, instruction, params))?;
        Ok(())
    }

    fn receive_status(&mut self, expected_id: u8, expected_params: usize) -> Result<Packet> {
        let mut buf = vec![0u8; expected_params + 6];
        match self.transport.receive_exact(&mut buf, self.reply_timeout) {
            Ok(()) => {}
            Err(Error::Timeout { got: 0, .. }) => return Err(Error::NoReply { id: expected_id }),
            Err(e) => return Err(e),
        }
        let (frame, _) = match parse_packet(&buf) {
            Ok(ok) => ok,
            Err(ParseError::BadChecksum { expected, found }) => {
                return Err(Error::Corrupt(format!(
                    "checksum expected {expected:02X} found {found:02X} in {}",
                    hex(&buf)
                )))
            }
            Err(e) => return Err(Error::Corrupt(format!("{e:?} in {}", hex(&buf)))),
        };
        if frame.id != expected_id {
            return Err(Error::WrongId { expected: expected_id, got: frame.id });
        }
        let error = frame.error();
        if error.is_advisory() {
            // note: the data in this reply is still good. Failing the read here
            // is what froze teleop on a tripped gripper: no read, so no new goal,
            // so the servo stayed pinned against its load.
            self.flags.push((frame.id, error));
        } else if !error.is_healthy() {
            return Err(Error::ServoFault { id: frame.id, error });
        }
        Ok(frame)
    }

    pub fn take_flags(&mut self) -> Vec<(u8, ServoError)> {
        std::mem::take(&mut self.flags)
    }

    pub fn ping(&mut self, id: u8) -> Result<()> {
        self.send(id, Instruction::Ping, &[])?;
        self.receive_status(id, 0)?;
        Ok(())
    }

    pub fn identify(&mut self, id: u8) -> Result<ServoIdentity> {
        self.ping(id)?;
        let model = self.read(id, MODEL_NUMBER)?;
        let present_position = self.read(id, PRESENT_POSITION)?;
        Ok(ServoIdentity { id, model, present_position })
    }

    pub fn read_block(&mut self, id: u8, address: u8, len: u8) -> Result<Vec<u8>> {
        self.send(id, Instruction::Read, &[address, len])?;
        let frame = self.receive_status(id, len as usize)?;
        Ok(frame.params)
    }

    pub fn read(&mut self, id: u8, reg: Register) -> Result<i32> {
        self.send(id, Instruction::Read, &[reg.address, reg.width])?;
        let frame = self.receive_status(id, reg.width as usize)?;
        Ok(reg.decode(&frame.params))
    }

    pub fn write(&mut self, id: u8, reg: Register, value: i32) -> Result<()> {
        if !reg.is_writable() {
            return Err(Error::Refused(format!("{} is read-only", reg.name)));
        }
        let mut params = vec![reg.address];
        params.extend(reg.encode(value));
        self.send(id, Instruction::Write, &params)?;
        if id == BROADCAST_ID {
            return Ok(());
        }
        // note: a servo with Response_Level = 0 acknowledges nothing, so
        // silence after a write is success, not failure.
        match self.receive_status(id, 0) {
            Ok(_) | Err(Error::NoReply { .. }) => Ok(()),
            Err(e) => Err(e),
        }
    }

    // Replies arrive in `ids` order; one missing reply fails the whole read.
    pub fn sync_read(&mut self, ids: &[u8], reg: Register) -> Result<Vec<i32>> {
        let blocks = self.sync_read_block(ids, reg.address, reg.width)?;
        Ok(blocks.iter().map(|b| reg.decode(b)).collect())
    }

    /// `len` bytes from `address` on every servo in `ids`, in one exchange.
    pub fn sync_read_block(&mut self, ids: &[u8], address: u8, len: u8) -> Result<Vec<Vec<u8>>> {
        let mut params = vec![address, len];
        params.extend_from_slice(ids);
        self.send(BROADCAST_ID, Instruction::SyncRead, &params)?;
        let mut blocks = Vec::with_capacity(ids.len());
        for &id in ids {
            blocks.push(self.receive_status(id, len as usize)?.params);
        }
        Ok(blocks)
    }

    pub fn sync_write(&mut self, reg: Register, values: &[(u8, i32)]) -> Result<()> {
        if !reg.is_writable() {
            return Err(Error::Refused(format!("{} is read-only", reg.name)));
        }
        let mut params = vec![reg.address, reg.width];
        for &(id, value) in values {
            params.push(id);
            params.extend(reg.encode(value));
        }
        self.send(BROADCAST_ID, Instruction::SyncWrite, &params)
    }

    pub fn scan(&mut self) -> Vec<u8> {
        let saved = self.reply_timeout;
        self.reply_timeout = Duration::from_millis(3);
        let found = (0..=MAX_SERVO_ID).filter(|&id| self.ping(id).is_ok()).collect();
        self.reply_timeout = saved;
        found
    }

    // note: EEPROM registers only take a write while Lock = 0 and torque is off.
    pub fn write_eeprom(&mut self, id: u8, reg: Register, value: i32) -> Result<()> {
        if !reg.is_eeprom() {
            return self.write(id, reg, value);
        }
        self.write_unlocked(id, reg, value)
    }

    // Torque off, Lock = 0, write, Lock = 1: for EEPROM and the factory area.
    // note: a write that commits to EEPROM or the factory area acknowledges
    // late: 2 to ~40 ms measured on hardware, against the normal 10 ms reply
    // window. An ack that misses the window arrives during the next
    // transaction and is misread as another servo's reply.
    pub fn write_unlocked(&mut self, id: u8, reg: Register, value: i32) -> Result<()> {
        self.write(id, TORQUE_ENABLE, 0)?;
        self.write(id, LOCK, 0)?;
        let result = self.write_slow(id, reg, value);
        self.write_slow(id, LOCK, 1)?;
        result
    }

    fn write_slow(&mut self, id: u8, reg: Register, value: i32) -> Result<()> {
        let saved = self.reply_timeout;
        self.reply_timeout = EEPROM_ACK_TIMEOUT;
        let result = self.write(id, reg, value);
        self.reply_timeout = saved;
        result
    }
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

impl fmt::Display for ServoError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_healthy() {
            return write!(f, "healthy");
        }
        let names = [
            (Self::VOLTAGE, "voltage"),
            (Self::ANGLE_SENSOR, "angle-sensor"),
            (Self::OVERHEAT, "overheat"),
            (Self::OVERCURRENT, "overcurrent"),
            (Self::ANGLE_LIMIT, "angle-limit"),
            (Self::OVERLOAD, "overload"),
        ];
        let set: Vec<&str> = names.iter().filter(|(bit, _)| self.0 & bit != 0).map(|(_, name)| *name).collect();
        write!(f, "{}", set.join("+"))
    }
}

pub fn checksum(id: u8, length: u8, instruction_or_error: u8, params: &[u8]) -> u8 {
    let sum: u32 =
        id as u32 + length as u32 + instruction_or_error as u32 + params.iter().map(|&b| b as u32).sum::<u32>();
    !(sum as u8)
}

pub fn encode_instruction(id: u8, instruction: Instruction, params: &[u8]) -> Vec<u8> {
    encode(id, instruction as u8, params)
}

pub fn encode_status(id: u8, error: ServoError, params: &[u8]) -> Vec<u8> {
    encode(id, error.0, params)
}

fn encode(id: u8, fourth: u8, params: &[u8]) -> Vec<u8> {
    let length = (params.len() + 2) as u8;
    let mut frame = Vec::with_capacity(params.len() + 6);
    frame.extend_from_slice(&HEADER);
    frame.push(id);
    frame.push(length);
    frame.push(fourth);
    frame.extend_from_slice(params);
    frame.push(checksum(id, length, fourth, params));
    frame
}

// Parse one frame from the front of `bytes`, returning it and the bytes consumed.
pub fn parse_packet(bytes: &[u8]) -> std::result::Result<(Packet, usize), ParseError> {
    if bytes.len() < 2 {
        return Err(ParseError::Incomplete(2 - bytes.len()));
    }
    if bytes[0..2] != HEADER {
        let skip = bytes.windows(2).position(|w| w == HEADER).unwrap_or(bytes.len() - 1).max(1);
        return Err(ParseError::NoHeader { skip });
    }
    if bytes.len() < 4 {
        return Err(ParseError::Incomplete(4 - bytes.len()));
    }
    let id = bytes[2];
    let length = bytes[3] as usize;
    if length < 2 {
        return Err(ParseError::NoHeader { skip: 2 });
    }
    let total = 4 + length;
    if bytes.len() < total {
        return Err(ParseError::Incomplete(total - bytes.len()));
    }
    let fourth = bytes[4];
    let params = &bytes[5..total - 1];
    let found = bytes[total - 1];
    let expected = checksum(id, length as u8, fourth, params);
    if found != expected {
        return Err(ParseError::BadChecksum { expected, found });
    }
    Ok((Packet { id, fourth_byte: fourth, params: params.to_vec() }, total))
}

// note: two-byte registers are little-endian on the STS series.
pub fn u16_to_bytes(value: u16) -> [u8; 2] {
    value.to_le_bytes()
}

pub fn bytes_to_u16(bytes: &[u8]) -> u16 {
    u16::from_le_bytes([bytes[0], bytes[1]])
}

// note: signed registers are sign-magnitude, not two's complement. The
// magnitude sits in the low bits and one high bit is the sign flag, so -5
// with the sign in bit 15 is 0x8005, not 0xFFFB.
pub fn encode_sign_magnitude(value: i32, sign_bit: u8) -> u16 {
    let magnitude = value.unsigned_abs() as u16;
    if value < 0 {
        magnitude | (1 << sign_bit)
    } else {
        magnitude
    }
}

pub fn decode_sign_magnitude(raw: u16, sign_bit: u8) -> i32 {
    let mask = 1u16 << sign_bit;
    let magnitude = (raw & !mask) as i32;
    if raw & mask != 0 {
        -magnitude
    } else {
        magnitude
    }
}

pub fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ")
}

pub fn is_eeprom_address(address: u8) -> bool {
    address < EEPROM_END
}

pub fn by_address(address: u8) -> Option<Register> {
    ALL.iter().copied().find(|r| r.address == address)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ping_frame_matches_manual() {
        assert_eq!(encode_instruction(1, Instruction::Ping, &[]), [0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB]);
    }

    #[test]
    fn read_present_position_matches_manual() {
        assert_eq!(
            encode_instruction(1, Instruction::Read, &[0x38, 0x02]),
            [0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE]
        );
    }

    #[test]
    fn write_goal_position_matches_manual() {
        assert_eq!(
            encode_instruction(1, Instruction::Write, &[0x2A, 0x00, 0x08]),
            [0xFF, 0xFF, 0x01, 0x05, 0x03, 0x2A, 0x00, 0x08, 0xC4]
        );
    }

    #[test]
    fn sync_write_six_servos_matches_manual() {
        let mut params = vec![0x2A, 0x02];
        for id in 1..=6u8 {
            params.extend_from_slice(&[id, 0x00, 0x08]);
        }
        let frame = encode_instruction(BROADCAST_ID, Instruction::SyncWrite, &params);
        assert_eq!(frame.len(), 26);
        assert_eq!(frame[3], 0x16);
        assert_eq!(*frame.last().unwrap(), 0xF7);
    }

    #[test]
    fn sync_read_six_servos_matches_manual() {
        let frame = encode_instruction(BROADCAST_ID, Instruction::SyncRead, &[0x38, 0x02, 1, 2, 3, 4, 5, 6]);
        assert_eq!(frame.len(), 14);
        assert_eq!(*frame.last().unwrap(), 0x26);
    }

    #[test]
    fn vendor_manual_checksums_reproduce() {
        assert_eq!(checksum(0xFE, 6, 0x82, &[0x38, 8, 1, 2]), 0x36);
        assert_eq!(checksum(1, 9, 3, &[0x2A, 0, 8, 0, 0, 0xE8, 3]), 0xD5);
    }

    #[test]
    fn status_reply_round_trips() {
        let reply = encode_status(1, ServoError(0), &[0x00, 0x08]);
        assert_eq!(reply, [0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x08, 0xF2]);
        let (frame, used) = parse_packet(&reply).unwrap();
        assert_eq!(used, 8);
        assert_eq!(frame.id, 1);
        assert!(frame.error().is_healthy());
        assert_eq!(bytes_to_u16(&frame.params), 2048);
    }

    #[test]
    fn parser_reports_incomplete_and_resyncs() {
        assert_eq!(parse_packet(&[0xFF]), Err(ParseError::Incomplete(1)));
        assert_eq!(parse_packet(&[0xFF, 0xFF, 0x01, 0x04, 0x00]), Err(ParseError::Incomplete(3)));
        let mut bytes = vec![0x12, 0x34, 0x56];
        bytes.extend_from_slice(&[0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC]);
        assert_eq!(parse_packet(&bytes), Err(ParseError::NoHeader { skip: 3 }));
        let (frame, _) = parse_packet(&bytes[3..]).unwrap();
        assert_eq!(frame.id, 1);
    }

    #[test]
    fn parser_rejects_bad_checksum() {
        let mut reply = encode_status(1, ServoError(0), &[]);
        *reply.last_mut().unwrap() ^= 0x01;
        assert!(matches!(parse_packet(&reply), Err(ParseError::BadChecksum { .. })));
    }

    #[test]
    fn sign_magnitude_is_not_twos_complement() {
        assert_eq!(encode_sign_magnitude(-5, 15), 0x8005);
        assert_eq!(decode_sign_magnitude(0x8005, 15), -5);
        assert_eq!(decode_sign_magnitude(0xFFFB, 15), -32763);
        assert_eq!(encode_sign_magnitude(-2047, 11), 0x0FFF);
        assert_eq!(decode_sign_magnitude(0x0FFF, 11), -2047);
        for v in [-4095, -1, 0, 1, 2048, 4095] {
            assert_eq!(decode_sign_magnitude(encode_sign_magnitude(v, 15), 15), v);
        }
    }

    #[test]
    fn error_byte_names_bits() {
        assert_eq!(ServoError(0).to_string(), "healthy");
        assert_eq!(ServoError(ServoError::OVERHEAT | ServoError::OVERLOAD).to_string(), "overheat+overload");
    }

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
