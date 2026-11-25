//! Domain types to and from the protobuf messages in `proto/`. Writing can't fail; reading
//! validates and names the field that's wrong. Struct conversions are generated from a field
//! map, so a field missing on either side is a compile error.

use std::time::Duration;

use prost::Message as _;

use super::pb::foxglove as fpb;
use super::pb::so101::v1 as pb;
use super::Error;
use crate::arm::*;
use crate::camera::*;
use crate::clock::*;
use crate::control::*;
use crate::policy::*;

pub(crate) const SCHEMA_JOINT_STATE: &str = "so101.v1.JointState";
pub(crate) const SCHEMA_ACTUATOR: &str = "so101.v1.ActuatorStatus";
pub(crate) const SCHEMA_JOINT_TARGET: &str = "so101.v1.JointTarget";
pub(crate) const SCHEMA_TICK: &str = "so101.v1.Tick";
pub(crate) const SCHEMA_CHUNK: &str = "so101.v1.Chunk";
pub(crate) const SCHEMA_RECORDER_DROP: &str = "so101.v1.RecorderDrop";
pub(crate) const SCHEMA_IMAGE: &str = "foxglove.CompressedImage";
pub(crate) const TOPIC_RECORDER_DROPS: &str = "/session/drops";

type Result<T> = std::result::Result<T, Error>;

pub(crate) trait Wire: Sized {
    type Pb;
    fn to_pb(&self) -> Self::Pb;
    /// `field` names the value in errors.
    fn from_pb(pb: Self::Pb, field: &str) -> Result<Self>;
}

impl Message {
    pub fn topic(&self) -> &'static str {
        use ArmRole::*;
        match self {
            Message::JointState(Leader, _) => "/leader/state",
            Message::JointState(Follower, _) => "/follower/state",
            Message::Actuator(Leader, _) => "/leader/actuators",
            Message::Actuator(Follower, _) => "/follower/actuators",
            Message::JointTarget(Leader, _) => "/leader/target",
            Message::JointTarget(Follower, _) => "/follower/target",
            Message::Tick(_) => "/tick",
            Message::Frame(f) => match f.camera {
                CameraRole::Wrist => "/cam/wrist",
                CameraRole::Scene => "/cam/scene",
            },
            Message::Chunk(_) => "/policy/chunk",
            Message::RecorderDrop(_) => TOPIC_RECORDER_DROPS,
        }
    }

    pub(crate) fn schema(&self) -> &'static str {
        match self {
            Message::JointState(..) => SCHEMA_JOINT_STATE,
            Message::Actuator(..) => SCHEMA_ACTUATOR,
            Message::JointTarget(..) => SCHEMA_JOINT_TARGET,
            Message::Tick(_) => SCHEMA_TICK,
            Message::Frame(_) => SCHEMA_IMAGE,
            Message::Chunk(_) => SCHEMA_CHUNK,
            Message::RecorderDrop(_) => SCHEMA_RECORDER_DROP,
        }
    }

    pub(crate) fn encode(&self) -> Vec<u8> {
        match self {
            Message::JointState(_, m) => m.to_pb().encode_to_vec(),
            Message::Actuator(_, m) => m.to_pb().encode_to_vec(),
            Message::JointTarget(_, m) => m.to_pb().encode_to_vec(),
            Message::Tick(m) => m.to_pb().encode_to_vec(),
            Message::Frame(f) => frame_to_pb(f).encode_to_vec(),
            Message::Chunk(m) => m.to_pb().encode_to_vec(),
            Message::RecorderDrop(m) => m.to_pb().encode_to_vec(),
        }
    }

    /// A frame's sequence and receive time come from its record, not its payload.
    pub(crate) fn decode(topic: &str, data: &[u8], sequence: u32, log_time: Timestamp) -> Result<Message> {
        let role = |name: &str| match name {
            "leader" => Ok(ArmRole::Leader),
            "follower" => Ok(ArmRole::Follower),
            _ => Err(Error::Format(format!("unknown arm in topic {topic}"))),
        };
        let parts: Vec<&str> = topic.trim_start_matches('/').split('/').collect();
        Ok(match parts.as_slice() {
            [a, "state"] => Message::JointState(role(a)?, Wire::from_pb(pb::JointState::decode(data)?, "JointState")?),
            [a, "actuators"] => {
                Message::Actuator(role(a)?, Wire::from_pb(pb::ActuatorStatus::decode(data)?, "ActuatorStatus")?)
            }
            [a, "target"] => {
                Message::JointTarget(role(a)?, Wire::from_pb(pb::JointTarget::decode(data)?, "JointTarget")?)
            }
            ["tick"] => Message::Tick(Wire::from_pb(pb::Tick::decode(data)?, "Tick")?),
            ["cam", _] => Message::Frame(frame_from_pb(fpb::CompressedImage::decode(data)?, sequence, log_time)?),
            ["policy", "chunk"] => Message::Chunk(Wire::from_pb(pb::Chunk::decode(data)?, "Chunk")?),
            ["session", "drops"] => {
                Message::RecorderDrop(Wire::from_pb(pb::RecorderDrop::decode(data)?, "RecorderDrop")?)
            }
            _ => return Err(Error::Format(format!("unknown topic {topic}"))),
        })
    }
}

impl Event {
    pub fn topic(&self) -> &'static str {
        self.message.topic()
    }
}

impl Wire for Timestamp {
    type Pb = u64;
    fn to_pb(&self) -> u64 {
        self.0
    }
    fn from_pb(v: u64, _: &str) -> Result<Self> {
        Ok(Timestamp(v))
    }
}

impl Wire for Pose {
    type Pb = Vec<f32>;
    fn to_pb(&self) -> Vec<f32> {
        self.0.to_vec()
    }
    fn from_pb(v: Vec<f32>, field: &str) -> Result<Self> {
        let n = v.len();
        let a = v.try_into().map_err(|_| invalid(format!("{field}: expected {JOINT_COUNT} values, got {n}")))?;
        Ok(Pose(a))
    }
}

/// Protobuf has no optional repeated field, so an empty list means no counts.
impl Wire for Option<Counts> {
    type Pb = Vec<i32>;
    fn to_pb(&self) -> Vec<i32> {
        self.map(|a| a.to_vec()).unwrap_or_default()
    }
    fn from_pb(v: Vec<i32>, field: &str) -> Result<Self> {
        if v.is_empty() {
            return Ok(None);
        }
        let n = v.len();
        let a = v.try_into().map_err(|_| invalid(format!("{field}: expected 0 or {JOINT_COUNT} counts, got {n}")))?;
        Ok(Some(a))
    }
}

impl<T: Wire> Wire for Vec<T> {
    type Pb = Vec<T::Pb>;
    fn to_pb(&self) -> Vec<T::Pb> {
        self.iter().map(Wire::to_pb).collect()
    }
    fn from_pb(v: Vec<T::Pb>, field: &str) -> Result<Self> {
        v.into_iter().map(|x| T::from_pb(x, field)).collect()
    }
}

impl Wire for CheckpointId {
    type Pb = String;
    fn to_pb(&self) -> String {
        self.0.clone()
    }
    fn from_pb(v: String, _: &str) -> Result<Self> {
        Ok(CheckpointId(v))
    }
}

/// A required message field: protobuf makes every message field optional.
impl Wire for Source {
    type Pb = Option<pb::Source>;
    fn to_pb(&self) -> Option<pb::Source> {
        let (kind, name) = match self {
            Source::Teleop => (pb::SourceKind::Teleop, String::new()),
            Source::Script(n) => (pb::SourceKind::Script, n.clone()),
            Source::Policy(id) => (pb::SourceKind::Policy, id.0.clone()),
        };
        Some(pb::Source { kind: kind as i32, name })
    }
    fn from_pb(v: Option<pb::Source>, field: &str) -> Result<Self> {
        let s = v.ok_or_else(|| invalid(format!("{field}: missing")))?;
        match pb::SourceKind::try_from(s.kind) {
            Ok(pb::SourceKind::Teleop) => Ok(Source::Teleop),
            Ok(pb::SourceKind::Script) => Ok(Source::Script(s.name)),
            Ok(pb::SourceKind::Policy) => Ok(Source::Policy(CheckpointId(s.name))),
            _ => Err(invalid(format!("{field}: unknown source kind {}", s.kind))),
        }
    }
}

impl Wire for Option<FrameRef> {
    type Pb = Option<pb::FrameRef>;
    fn to_pb(&self) -> Option<pb::FrameRef> {
        self.as_ref().map(Wire::to_pb)
    }
    fn from_pb(v: Option<pb::FrameRef>, field: &str) -> Result<Self> {
        v.map(|r| FrameRef::from_pb(r, field)).transpose()
    }
}

/// The action matrix is flattened row-major, so its shape has to be checked against `k`.
impl Wire for Chunk {
    type Pb = pb::Chunk;
    fn to_pb(&self) -> pb::Chunk {
        pb::Chunk {
            t_obs_ns: self.t_obs.0,
            period_ns: self.period.as_nanos() as u64,
            k: self.actions.len() as u32,
            actions: self.actions.iter().flat_map(|p| p.0).collect(),
            checkpoint_id: self.checkpoint_id.0.clone(),
        }
    }
    fn from_pb(m: pb::Chunk, _: &str) -> Result<Self> {
        let want = m.k as usize * JOINT_COUNT;
        if m.actions.len() != want {
            return Err(invalid(format!("Chunk: k = {} needs {want} values, got {}", m.k, m.actions.len())));
        }
        if m.period_ns == 0 {
            return Err(invalid("Chunk.period_ns: must be set"));
        }
        let actions = m
            .actions
            .chunks_exact(JOINT_COUNT)
            .map(|r| Pose::from_pb(r.to_vec(), "Chunk.actions"))
            .collect::<Result<_>>()?;
        Ok(Chunk {
            t_obs: Timestamp(m.t_obs_ns),
            period: Duration::from_nanos(m.period_ns),
            actions,
            checkpoint_id: CheckpointId(m.checkpoint_id),
        })
    }
}

macro_rules! wire_same {
    ($($t:ty),*) => {$(
        impl Wire for $t {
            type Pb = $t;
            fn to_pb(&self) -> $t {
                self.clone()
            }
            fn from_pb(v: $t, _: &str) -> Result<$t> {
                Ok(v)
            }
        }
    )*};
}

macro_rules! wire_enum {
    ($t:ident => $pb:ident { $($v:ident),* $(,)? }) => {
        impl Wire for $t {
            type Pb = i32;
            fn to_pb(&self) -> i32 {
                match self {
                    $($t::$v => pb::$pb::$v as i32,)*
                }
            }
            fn from_pb(v: i32, field: &str) -> Result<$t> {
                match pb::$pb::try_from(v) {
                    $(Ok(pb::$pb::$v) => Ok($t::$v),)*
                    _ => Err(invalid(format!("{field}: unknown value {v}"))),
                }
            }
        }
    };
}

macro_rules! wire_struct {
    ($t:ident => $pb:ident { $($f:ident: $pf:ident),* $(,)? }) => {
        impl Wire for $t {
            type Pb = pb::$pb;
            fn to_pb(&self) -> pb::$pb {
                pb::$pb { $($pf: self.$f.to_pb(),)* }
            }
            fn from_pb(m: pb::$pb, _: &str) -> Result<$t> {
                Ok($t { $($f: Wire::from_pb(m.$pf, concat!(stringify!($pb), ".", stringify!($pf)))?,)* })
            }
        }
    };
}

wire_same!(u32, String, Option<f32>);
wire_enum!(Joint => Joint { ShoulderPan, ShoulderLift, ElbowFlex, WristFlex, WristRoll, Gripper });
wire_enum!(CameraRole => Camera { Wrist, Scene });
wire_enum!(Limit => Limit { Step, Range, Stall, Collision });
wire_enum!(Fault => Fault { Voltage, Sensor, Overheat, Overcurrent, AngleLimit, Overload });
wire_struct!(JointState => JointState { t: t_ns, pose: pose, counts: counts });
wire_struct!(JointTarget => JointTarget { t: t_ns, pose: pose, source: source, counts: counts });

wire_struct!(ActuatorStatus => ActuatorStatus {
    t: t_ns,
    joint: joint,
    velocity_deg_s: velocity_deg_s,
    load_percent: load_percent,
    voltage_v: voltage_v,
    temperature_c: temperature_c,
    current_a: current_a,
    faults: faults,
});

wire_struct!(FrameRef => FrameRef { camera: camera, sequence: sequence });
wire_struct!(Clip => Clip { joint: joint, limit: limit });

wire_struct!(Tick => Tick {
    t: t_ns,
    cycle: cycle,
    state: state,
    wrist: wrist,
    scene: scene,
    action: action,
    source: source,
    requested: requested,
    clipped: clipped,
});

wire_struct!(RecorderDrop => RecorderDrop {
    t: t_ns,
    topic: topic,
    first_sequence: first_sequence,
    last_sequence: last_sequence,
    count: count,
});

fn invalid(msg: impl Into<String>) -> Error {
    Error::Wire(msg.into())
}

/// Frames are Foxglove's image message, so viewers show them without our schema.
pub(crate) fn frame_to_pb(f: &Frame) -> fpb::CompressedImage {
    let ns = f.t_capture.0;
    fpb::CompressedImage {
        timestamp: Some(prost_types::Timestamp {
            seconds: (ns / 1_000_000_000) as i64,
            nanos: (ns % 1_000_000_000) as i32,
        }),
        frame_id: f.camera.name().to_string(),
        data: f.jpeg.clone(),
        format: "jpeg".to_string(),
    }
}

/// The frame's sequence and receive time come from its MCAP record, not the image message.
pub(crate) fn frame_from_pb(img: fpb::CompressedImage, sequence: u32, t_receive: Timestamp) -> Result<Frame> {
    let ts = img.timestamp.ok_or_else(|| invalid("CompressedImage: missing timestamp"))?;
    if ts.seconds < 0 || !(0..1_000_000_000).contains(&ts.nanos) {
        return Err(invalid("CompressedImage: timestamp out of range"));
    }
    let camera = CameraRole::from_name(&img.frame_id)
        .ok_or_else(|| invalid(format!("CompressedImage: unknown camera {:?}", img.frame_id)))?;
    Ok(Frame {
        camera,
        sequence,
        t_capture: Timestamp(ts.seconds as u64 * 1_000_000_000 + ts.nanos as u64),
        t_receive,
        jpeg: img.data,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_pose_needs_six_values() {
        let err = JointState::from_pb(pb::JointState { t_ns: 1, pose: vec![1.0; 5], counts: vec![] }, "").unwrap_err();
        assert!(err.to_string().contains("JointState.pose: expected 6"), "{err}");
    }

    #[test]
    fn counts_are_absent_or_exactly_six() {
        let m = |counts: Vec<i32>| pb::JointState { t_ns: 1, pose: vec![0.0; 6], counts };
        assert_eq!(JointState::from_pb(m(vec![]), "").unwrap().counts, None);
        assert_eq!(JointState::from_pb(m(vec![2048; 6]), "").unwrap().counts, Some([2048; 6]));
        assert!(JointState::from_pb(m(vec![2048; 3]), "").is_err());
    }

    #[test]
    fn a_missing_source_is_rejected() {
        let m = pb::JointTarget { t_ns: 0, pose: vec![0.0; 6], source: None, counts: vec![] };
        assert!(JointTarget::from_pb(m, "").unwrap_err().to_string().contains("JointTarget.source: missing"));
    }

    #[test]
    fn an_unknown_enum_value_is_rejected() {
        let m = pb::Clip { joint: 42, limit: 1 };
        assert!(Clip::from_pb(m, "").unwrap_err().to_string().contains("Clip.joint: unknown value 42"));
    }

    #[test]
    fn a_chunk_shape_and_period_are_checked() {
        let m =
            |k, n, period_ns| pb::Chunk { t_obs_ns: 0, period_ns, k, actions: vec![0.0; n], checkpoint_id: "x".into() };
        assert!(Chunk::from_pb(m(2, 6, 33_333_333), "").is_err());
        assert!(Chunk::from_pb(m(1, 6, 0), "").is_err());
        assert!(Chunk::from_pb(m(1, 6, 33_333_333), "").is_ok());
    }

    #[test]
    fn actuator_status_round_trips() {
        let a = ActuatorStatus {
            t: Timestamp(5),
            joint: Joint::WristFlex,
            velocity_deg_s: Some(1.5),
            load_percent: None,
            voltage_v: Some(12.0),
            temperature_c: Some(33.0),
            current_a: Some(0.1),
            faults: vec![Fault::Overload, Fault::Overheat],
        };
        assert_eq!(ActuatorStatus::from_pb(a.to_pb(), "").unwrap(), a);
    }
}
