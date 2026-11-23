//! `ServoBus`: talking to Feetech STS3215 servos on one half-duplex serial bus: packets,
//! checksums, transactions, the status flags every reply carries, and the errors.

use crate::arm::registers::{self, Register, LOCK, TORQUE_ENABLE};
use crate::arm::serial::{Transport, TransportError};
use std::fmt;
use std::io;
use std::time::Duration;

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug)]
pub enum Error {
    Transport(TransportError),
    NoReply { id: u8 },
    Corrupt(String),
    WrongId { expected: u8, got: u8 },
    ServoFault { id: u8, error: ServoError },
    UnexpectedModel { id: u8, model: i32 },
    // A write refused before touching the wire.
    Refused(String),
    Calibration(String),
    Io(io::Error),
    Other(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Transport(e) => write!(f, "{e}"),
            Error::NoReply { id } => write!(f, "no reply from servo {id}"),
            Error::Corrupt(s) => write!(f, "corrupt reply: {s}"),
            Error::WrongId { expected, got } => write!(f, "reply from servo {got}, expected {expected}"),
            Error::ServoFault { id, error } => write!(f, "servo {id} fault: {error}"),
            Error::UnexpectedModel { id, model } => write!(f, "servo {id} reports model {model}, not an STS3215"),
            Error::Refused(s) => write!(f, "refused: {s}"),
            Error::Calibration(s) => write!(f, "calibration: {s}"),
            Error::Io(e) => write!(f, "{e}"),
            Error::Other(s) => write!(f, "{s}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<TransportError> for Error {
    fn from(e: TransportError) -> Self {
        Error::Transport(e)
    }
}

impl From<io::Error> for Error {
    fn from(e: io::Error) -> Self {
        Error::Io(e)
    }
}

impl From<String> for Error {
    fn from(s: String) -> Self {
        Error::Other(s)
    }
}

impl From<&str> for Error {
    fn from(s: &str) -> Self {
        Error::Other(s.to_string())
    }
}

pub const HEADER: [u8; 2] = [0xFF, 0xFF];
pub const BROADCAST_ID: u8 = 0xFE;
pub const MAX_SERVO_ID: u8 = 0xFD;

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

// Fault bits from a status packet's error byte or the Status register.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct ServoError(pub u8);

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

// A parsed frame. The fourth byte is an instruction host-to-servo and an
// error byte servo-to-host; the caller knows which direction it came from.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Packet {
    pub id: u8,
    pub fourth_byte: u8,
    pub params: Vec<u8>,
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

#[derive(Debug, PartialEq, Eq)]
pub enum ParseError {
    // At least this many more bytes are needed.
    Incomplete(usize),
    // No header at the front; discard `skip` bytes and try again.
    NoHeader { skip: usize },
    BadChecksum { expected: u8, found: u8 },
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

// How long to wait for the ack of a write that commits to non-volatile memory.
// Up to ~40 ms has been measured; allow more than double.
pub const EEPROM_ACK_TIMEOUT: Duration = Duration::from_millis(100);

pub struct ServoBus {
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

impl ServoIdentity {
    pub fn is_sts3215(&self) -> bool {
        self.model == registers::STS3215_MODEL_NUMBER
    }
}

impl ServoBus {
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
            Err(TransportError::Timeout { got: 0, .. }) => return Err(Error::NoReply { id: expected_id }),
            Err(e) => return Err(e.into()),
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
        let model = self.read(id, registers::MODEL_NUMBER)?;
        let present_position = self.read(id, registers::PRESENT_POSITION)?;
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
        let mut params = vec![reg.address, reg.width];
        params.extend_from_slice(ids);
        self.send(BROADCAST_ID, Instruction::SyncRead, &params)?;
        let mut values = Vec::with_capacity(ids.len());
        for &id in ids {
            let frame = self.receive_status(id, reg.width as usize)?;
            values.push(reg.decode(&frame.params));
        }
        Ok(values)
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

pub fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ")
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
}
