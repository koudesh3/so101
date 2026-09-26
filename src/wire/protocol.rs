//! Feetech bus frames: pure functions on bytes, nothing touches hardware.
//!
//! note: wire format for both directions is
//!   FF FF | ID | LEN | INSTR-or-ERROR | params... | CHECKSUM
//!   LEN      = params.len() + 2
//!   CHECKSUM = !(ID + LEN + INSTR + sum(params)) & 0xFF

use std::fmt;

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
    // or its own overload or current protection. Reported, not treated as a
    // failed read: failing the read freezes teleop with the servo still pushing.
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
        let set: Vec<&str> = names
            .iter()
            .filter(|(bit, _)| self.0 & bit != 0)
            .map(|(_, name)| *name)
            .collect();
        write!(f, "{}", set.join("+"))
    }
}

pub fn checksum(id: u8, length: u8, instruction_or_error: u8, params: &[u8]) -> u8 {
    let sum: u32 = id as u32
        + length as u32
        + instruction_or_error as u32
        + params.iter().map(|&b| b as u32).sum::<u32>();
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
pub struct Frame {
    pub id: u8,
    pub fourth_byte: u8,
    pub params: Vec<u8>,
}

impl Frame {
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
pub fn parse_frame(bytes: &[u8]) -> Result<(Frame, usize), ParseError> {
    if bytes.len() < 2 {
        return Err(ParseError::Incomplete(2 - bytes.len()));
    }
    if bytes[0..2] != HEADER {
        let skip = bytes
            .windows(2)
            .position(|w| w == HEADER)
            .unwrap_or(bytes.len() - 1)
            .max(1);
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
    Ok((Frame { id, fourth_byte: fourth, params: params.to_vec() }, total))
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
        let (frame, used) = parse_frame(&reply).unwrap();
        assert_eq!(used, 8);
        assert_eq!(frame.id, 1);
        assert!(frame.error().is_healthy());
        assert_eq!(bytes_to_u16(&frame.params), 2048);
    }

    #[test]
    fn parser_reports_incomplete_and_resyncs() {
        assert_eq!(parse_frame(&[0xFF]), Err(ParseError::Incomplete(1)));
        assert_eq!(parse_frame(&[0xFF, 0xFF, 0x01, 0x04, 0x00]), Err(ParseError::Incomplete(3)));
        let mut bytes = vec![0x12, 0x34, 0x56];
        bytes.extend_from_slice(&[0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC]);
        assert_eq!(parse_frame(&bytes), Err(ParseError::NoHeader { skip: 3 }));
        let (frame, _) = parse_frame(&bytes[3..]).unwrap();
        assert_eq!(frame.id, 1);
    }

    #[test]
    fn parser_rejects_bad_checksum() {
        let mut reply = encode_status(1, ServoError(0), &[]);
        *reply.last_mut().unwrap() ^= 0x01;
        assert!(matches!(parse_frame(&reply), Err(ParseError::BadChecksum { .. })));
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
