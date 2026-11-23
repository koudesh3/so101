//! The arm: its joints, poses and messages, its errors, and the `Arm` trait the control loop uses.
//! `ServoArm` is the real one, on STS3215 servos.

mod calibration;
mod servo_arm;
pub mod servos;
mod simulated_servos;
mod uart;

use self::servos::ServoError;
use crate::clock::Timestamp;
use crate::policy::CheckpointId;

pub use calibration::{check_centred, sweep, Calibration, Sweep};
pub use servo_arm::{FollowerGains, ServoArm, IDS};
pub use servos::Servos;
pub use simulated_servos::{demo, Hand, Obstacle, SimulatedServos};
pub use uart::{read_trace, ByteTrace, SerialPort, Traced, Transport};

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

/// Where every joint is, each in its joint's unit.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose(pub [f32; JOINT_COUNT]);

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

pub type Result<T> = std::result::Result<T, Error>;

/// Everything that can go wrong with an arm, from the serial line up.
#[derive(Debug)]
pub enum Error {
    /// The line delivered fewer bytes than a reply needs before the deadline.
    Timeout {
        wanted: usize,
        got: usize,
    },
    NoReply {
        id: u8,
    },
    Corrupt(String),
    WrongId {
        expected: u8,
        got: u8,
    },
    ServoFault {
        id: u8,
        error: ServoError,
    },
    UnexpectedModel {
        id: u8,
        model: i32,
    },
    /// A write refused before touching the wire.
    Refused(String),
    Calibration(String),
    Io(std::io::Error),
    Other(String),
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

/// Why a joint's written goal differs from the commanded one.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Limit {
    /// The most a goal may move per write (LeRobot's `max_relative_target`).
    Step,
    Range,
    /// Held at a contact point after a stall or an overload trip.
    Stall,
}

/// What an arm actually wrote for a commanded pose.
#[derive(Clone, Debug, PartialEq)]
pub struct Sent {
    pub pose: Pose,
    /// `None` when not written to servos.
    pub counts: Option<Counts>,
    pub limits: [Option<Limit>; JOINT_COUNT],
}

/// What the control loop can do with an arm. `ServoArm` is the SO-101 on its servos; a mock arm
/// joins it in Phase 1.
pub trait Arm: Send {
    fn read_state(&mut self) -> Result<JointState>;
    /// Move toward `target`. The arm applies its own protections (range, stall hold, step limit)
    /// and reports what it actually wrote, and why each changed joint differs.
    fn command(&mut self, target: &Pose) -> Result<Sent>;
    fn read_actuator(&mut self, joint: Joint) -> Result<ActuatorStatus>;
    fn torque_off(&mut self) -> Result<()>;
    /// Torque on, holding where it is now, so nothing jumps; protections start fresh.
    fn hold(&mut self) -> Result<()>;
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

impl std::ops::Index<Joint> for Pose {
    type Output = f32;
    fn index(&self, joint: Joint) -> &f32 {
        &self.0[joint.index()]
    }
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Timeout { wanted, got } => write!(f, "timeout after {got} of {wanted} bytes"),
            Error::NoReply { id } => write!(f, "no reply from servo {id}"),
            Error::Corrupt(s) => write!(f, "corrupt reply: {s}"),
            Error::WrongId { expected, got } => write!(f, "reply from servo {got}, expected {expected}"),
            Error::ServoFault { id, error } => write!(f, "servo {id} fault: {error}"),
            Error::UnexpectedModel { id, model } => write!(f, "servo {id} reports model {model}, not an STS3215"),
            Error::Refused(s) => write!(f, "refused: {s}"),
            Error::Calibration(s) => write!(f, "calibration: {s}"),
            Error::Io(e) => write!(f, "io: {e}"),
            Error::Other(s) => write!(f, "{s}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        if e.kind() == std::io::ErrorKind::TimedOut {
            Error::Timeout { wanted: 0, got: 0 }
        } else {
            Error::Io(e)
        }
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

impl<A: Arm + ?Sized> Arm for &mut A {
    fn read_state(&mut self) -> Result<JointState> {
        (**self).read_state()
    }

    fn command(&mut self, target: &Pose) -> Result<Sent> {
        (**self).command(target)
    }

    fn read_actuator(&mut self, joint: Joint) -> Result<ActuatorStatus> {
        (**self).read_actuator(joint)
    }

    fn torque_off(&mut self) -> Result<()> {
        (**self).torque_off()
    }

    fn hold(&mut self) -> Result<()> {
        (**self).hold()
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
}
