//! Host side of one Feetech bus: every method is one complete transaction,
//! discard stale input, send one instruction, read the replies it earns.

use std::time::Duration;

use crate::error::{Error, Result};
use crate::wire::protocol::{ServoError, encode_instruction, parse_frame, Frame, Instruction, ParseError, BROADCAST_ID, MAX_SERVO_ID};
use crate::wire::registers::{self, Register, LOCK, TORQUE_ENABLE};
use crate::wire::transport::{Transport, TransportError};

// How long to wait for the ack of a write that commits to non-volatile memory.
// Up to ~40 ms has been measured; allow more than double.
pub const EEPROM_ACK_TIMEOUT: Duration = Duration::from_millis(100);

pub struct Bus {
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

impl Bus {
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

    fn receive_status(&mut self, expected_id: u8, expected_params: usize) -> Result<Frame> {
        let mut buf = vec![0u8; expected_params + 6];
        match self.transport.receive_exact(&mut buf, self.reply_timeout) {
            Ok(()) => {}
            Err(TransportError::Timeout { got: 0, .. }) => return Err(Error::NoReply { id: expected_id }),
            Err(e) => return Err(e.into()),
        }
        let (frame, _) = match parse_frame(&buf) {
            Ok(ok) => ok,
            Err(ParseError::BadChecksum { expected, found }) => {
                return Err(Error::Corrupt(format!("checksum expected {expected:02X} found {found:02X} in {}", hex(&buf))))
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

    // Advisory bits collected since the last call, then cleared.
    pub fn take_flags(&mut self) -> Vec<(u8, ServoError)> {
        std::mem::take(&mut self.flags)
    }

    pub fn ping(&mut self, id: u8) -> Result<()> {
        self.send(id, Instruction::Ping, &[])?;
        self.receive_status(id, 0)?;
        Ok(())
    }

    // Ping plus the two registers that say what is there and where it is.
    pub fn identify(&mut self, id: u8) -> Result<ServoIdentity> {
        self.ping(id)?;
        let model = self.read(id, registers::MODEL_NUMBER)?;
        let present_position = self.read(id, registers::PRESENT_POSITION)?;
        Ok(ServoIdentity { id, model, present_position })
    }

    // `len` raw bytes from `address` in one transaction.
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
    // transaction and is misread as another servo's reply. Wait long enough.
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
