//! The domain types: what the rest of the code uses. Fixed arrays and validated fields; the
//! conversions to and from the protobuf messages are in `convert.rs`.

use bytes::Bytes;

use crate::Timestamp;

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

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Camera {
    Wrist,
    Scene,
}

impl Camera {
    pub fn name(self) -> &'static str {
        match self {
            Camera::Wrist => "wrist",
            Camera::Scene => "scene",
        }
    }

    pub fn from_name(name: &str) -> Option<Camera> {
        match name {
            "wrist" => Some(Camera::Wrist),
            "scene" => Some(Camera::Scene),
            _ => None,
        }
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

/// A camera frame's identity: its camera and the kernel's frame number.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct FrameRef {
    pub camera: Camera,
    pub sequence: u32,
}

/// One camera frame. In the log, `t_capture` is the record's `publish_time` (and inside the
/// message), `t_receive` its `log_time`, and `sequence` its `sequence`.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub camera: Camera,
    /// The kernel's frame number for a V4L2 camera; the source's own count otherwise.
    pub sequence: u32,
    /// When the frame was captured: the kernel's buffer timestamp for a V4L2 camera, the
    /// generation time for a synthetic one.
    pub t_capture: Timestamp,
    /// When our process received it.
    pub t_receive: Timestamp,
    pub jpeg: Bytes,
}

impl Frame {
    pub fn id(&self) -> FrameRef {
        FrameRef { camera: self.camera, sequence: self.sequence }
    }
}

/// What the control loop used at one tick (contract 2).
#[derive(Clone, Debug, PartialEq)]
pub struct Tick {
    pub t: Timestamp,
    pub state: Joints,
    pub wrist: Option<FrameRef>,
    pub scene: Option<FrameRef>,
    pub action: Joints,
    pub source: Source,
}

/// A predicted action chunk, in degrees (contract 4).
#[derive(Clone, Debug, PartialEq)]
pub struct Chunk {
    pub t_obs: Timestamp,
    pub actions: Vec<Joints>,
    pub ckpt_id: String,
}

/// A contiguous run of messages on one topic that the recorder dropped.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LogDrop {
    pub t: Timestamp,
    pub topic: String,
    pub first_sequence: u32,
    pub last_sequence: u32,
    pub count: u32,
}
