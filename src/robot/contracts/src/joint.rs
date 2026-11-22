//! Joints: their names and order, positions, servo telemetry, targets, who produced a target,
//! and the traits for reading and commanding an arm.

use crate::pb::so101::v1 as pb;
use crate::{ContractError, DynError, Timestamp};

type Result<T> = std::result::Result<T, ContractError>;

pub const JOINT_COUNT: usize = 6;

/// The SO-101's joints, in LeRobot's order and with LeRobot's names.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Joint {
    ShoulderPan,
    ShoulderLift,
    ElbowFlex,
    WristFlex,
    WristRoll,
    Gripper,
}

impl Joint {
    pub const ALL: [Joint; JOINT_COUNT] =
        [Joint::ShoulderPan, Joint::ShoulderLift, Joint::ElbowFlex, Joint::WristFlex, Joint::WristRoll, Joint::Gripper];

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

    pub fn index(self) -> usize {
        Joint::ALL.iter().position(|j| *j == self).expect("every joint is in ALL")
    }
}

/// Six joint values in LeRobot's order: degrees, with the gripper 0-100.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Joints(pub [f32; JOINT_COUNT]);

impl Joints {
    pub fn get(&self, joint: Joint) -> f32 {
        self.0[joint.index()]
    }
}

/// Who produced a target or an action.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Source {
    Teleop,
    Script(String),
    /// A policy, by checkpoint ID.
    Policy(String),
}

/// All six positions from one sync read.
#[derive(Clone, Debug, PartialEq)]
pub struct JointState {
    pub t: Timestamp,
    pub pos: Joints,
}

/// One servo's telemetry block. Each field is `None` when it wasn't read.
#[derive(Clone, Debug, PartialEq)]
pub struct ServoTelemetry {
    pub t: Timestamp,
    pub joint: Joint,
    pub velocity_deg_s: Option<f32>,
    pub load_percent: Option<f32>,
    pub voltage_v: Option<f32>,
    pub temperature_c: Option<f32>,
    pub current_raw: Option<u32>,
    pub status: u8,
    pub moving: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct JointTarget {
    pub t: Timestamp,
    pub pos: Joints,
    pub source: Source,
}

/// The one writer to an arm (behind the safety wrapper, from Phase 1).
pub trait ArmDriver: Send {
    fn read_state(&mut self) -> std::result::Result<JointState, DynError>;
    fn write_target(&mut self, target: &JointTarget) -> std::result::Result<(), DynError>;
}

/// Something that proposes targets: the leader arm, a script, the action queue.
pub trait TargetSource: Send {
    fn next_target(&mut self) -> std::result::Result<JointTarget, DynError>;
}

// ---- protobuf conversions: to pb is infallible; from pb validates ----

/// Six values from a protobuf `repeated float`, or a contract error naming the field.
pub(crate) fn joints_from_pb(v: &[f32], what: &str) -> Result<Joints> {
    let a: [f32; JOINT_COUNT] = v
        .try_into()
        .map_err(|_| ContractError::new(format!("{what}: expected {JOINT_COUNT} values, got {}", v.len())))?;
    Ok(Joints(a))
}

impl From<Joint> for pb::Joint {
    fn from(j: Joint) -> Self {
        match j {
            Joint::ShoulderPan => pb::Joint::ShoulderPan,
            Joint::ShoulderLift => pb::Joint::ShoulderLift,
            Joint::ElbowFlex => pb::Joint::ElbowFlex,
            Joint::WristFlex => pb::Joint::WristFlex,
            Joint::WristRoll => pb::Joint::WristRoll,
            Joint::Gripper => pb::Joint::Gripper,
        }
    }
}

fn joint_from_pb(v: i32) -> Result<Joint> {
    match pb::Joint::try_from(v) {
        Ok(pb::Joint::ShoulderPan) => Ok(Joint::ShoulderPan),
        Ok(pb::Joint::ShoulderLift) => Ok(Joint::ShoulderLift),
        Ok(pb::Joint::ElbowFlex) => Ok(Joint::ElbowFlex),
        Ok(pb::Joint::WristFlex) => Ok(Joint::WristFlex),
        Ok(pb::Joint::WristRoll) => Ok(Joint::WristRoll),
        Ok(pb::Joint::Gripper) => Ok(Joint::Gripper),
        _ => Err(ContractError::new(format!("unknown joint {v}"))),
    }
}

impl From<&Source> for pb::Source {
    fn from(s: &Source) -> Self {
        let (kind, name) = match s {
            Source::Teleop => (pb::SourceKind::Teleop, String::new()),
            Source::Script(n) => (pb::SourceKind::Script, n.clone()),
            Source::Policy(n) => (pb::SourceKind::Policy, n.clone()),
        };
        pb::Source { kind: kind as i32, name }
    }
}

impl TryFrom<Option<pb::Source>> for Source {
    type Error = ContractError;
    fn try_from(s: Option<pb::Source>) -> Result<Self> {
        let s = s.ok_or_else(|| ContractError::new("missing source"))?;
        match pb::SourceKind::try_from(s.kind) {
            Ok(pb::SourceKind::Teleop) => Ok(Source::Teleop),
            Ok(pb::SourceKind::Script) => Ok(Source::Script(s.name)),
            Ok(pb::SourceKind::Policy) => Ok(Source::Policy(s.name)),
            _ => Err(ContractError::new(format!("unknown source kind {}", s.kind))),
        }
    }
}

impl From<&JointState> for pb::JointState {
    fn from(m: &JointState) -> Self {
        pb::JointState { t_ns: m.t.0, pos: m.pos.0.to_vec() }
    }
}

impl TryFrom<pb::JointState> for JointState {
    type Error = ContractError;
    fn try_from(m: pb::JointState) -> Result<Self> {
        Ok(JointState { t: Timestamp(m.t_ns), pos: joints_from_pb(&m.pos, "JointState.pos")? })
    }
}

impl From<&ServoTelemetry> for pb::ServoTelemetry {
    fn from(m: &ServoTelemetry) -> Self {
        pb::ServoTelemetry {
            t_ns: m.t.0,
            joint: pb::Joint::from(m.joint) as i32,
            velocity_deg_s: m.velocity_deg_s,
            load_percent: m.load_percent,
            voltage_v: m.voltage_v,
            temperature_c: m.temperature_c,
            current_raw: m.current_raw,
            status: m.status as u32,
            moving: m.moving,
        }
    }
}

impl TryFrom<pb::ServoTelemetry> for ServoTelemetry {
    type Error = ContractError;
    fn try_from(m: pb::ServoTelemetry) -> Result<Self> {
        Ok(ServoTelemetry {
            t: Timestamp(m.t_ns),
            joint: joint_from_pb(m.joint)?,
            velocity_deg_s: m.velocity_deg_s,
            load_percent: m.load_percent,
            voltage_v: m.voltage_v,
            temperature_c: m.temperature_c,
            current_raw: m.current_raw,
            status: u8::try_from(m.status).map_err(|_| ContractError::new("status is a byte"))?,
            moving: m.moving,
        })
    }
}

impl From<&JointTarget> for pb::JointTarget {
    fn from(m: &JointTarget) -> Self {
        pb::JointTarget { t_ns: m.t.0, pos: m.pos.0.to_vec(), source: Some((&m.source).into()) }
    }
}

impl TryFrom<pb::JointTarget> for JointTarget {
    type Error = ContractError;
    fn try_from(m: pb::JointTarget) -> Result<Self> {
        Ok(JointTarget {
            t: Timestamp(m.t_ns),
            pos: joints_from_pb(&m.pos, "JointTarget.pos")?,
            source: m.source.try_into()?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrong_joint_count_is_rejected() {
        let err = JointState::try_from(pb::JointState { t_ns: 1, pos: vec![1.0; 5] }).unwrap_err();
        assert!(err.0.contains("expected 6"), "{err}");
    }

    #[test]
    fn missing_source_is_rejected() {
        let m = pb::JointTarget { t_ns: 0, pos: vec![0.0; 6], source: None };
        assert!(JointTarget::try_from(m).is_err());
    }
}
