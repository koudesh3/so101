//! Domain types to protobuf (infallible) and back (validated: array lengths, required fields,
//! known enum values).

use crate::pb::foxglove as fpb;
use crate::pb::so101::v1 as pb;
use crate::{
    Camera, Chunk, ContractError, Frame, FrameRef, Joint, JointState, JointTarget, Joints, LogDrop, ServoTelemetry,
    Source, Tick, Timestamp, JOINT_COUNT,
};

type Result<T> = std::result::Result<T, ContractError>;

fn joints(v: &[f32], what: &str) -> Result<Joints> {
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

fn joint_from(v: i32) -> Result<Joint> {
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

impl From<Camera> for pb::Camera {
    fn from(c: Camera) -> Self {
        match c {
            Camera::Wrist => pb::Camera::Wrist,
            Camera::Scene => pb::Camera::Scene,
        }
    }
}

fn camera_from(v: i32) -> Result<Camera> {
    match pb::Camera::try_from(v) {
        Ok(pb::Camera::Wrist) => Ok(Camera::Wrist),
        Ok(pb::Camera::Scene) => Ok(Camera::Scene),
        _ => Err(ContractError::new(format!("unknown camera {v}"))),
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
        Ok(JointState { t: Timestamp(m.t_ns), pos: joints(&m.pos, "JointState.pos")? })
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
            joint: joint_from(m.joint)?,
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
        Ok(JointTarget { t: Timestamp(m.t_ns), pos: joints(&m.pos, "JointTarget.pos")?, source: m.source.try_into()? })
    }
}

impl From<FrameRef> for pb::FrameRef {
    fn from(r: FrameRef) -> Self {
        pb::FrameRef { camera: pb::Camera::from(r.camera) as i32, sequence: r.sequence }
    }
}

impl TryFrom<pb::FrameRef> for FrameRef {
    type Error = ContractError;
    fn try_from(r: pb::FrameRef) -> Result<Self> {
        Ok(FrameRef { camera: camera_from(r.camera)?, sequence: r.sequence })
    }
}

impl From<&Tick> for pb::Tick {
    fn from(m: &Tick) -> Self {
        pb::Tick {
            t_ns: m.t.0,
            state: m.state.0.to_vec(),
            wrist: m.wrist.map(Into::into),
            scene: m.scene.map(Into::into),
            action: m.action.0.to_vec(),
            source: Some((&m.source).into()),
        }
    }
}

impl TryFrom<pb::Tick> for Tick {
    type Error = ContractError;
    fn try_from(m: pb::Tick) -> Result<Self> {
        Ok(Tick {
            t: Timestamp(m.t_ns),
            state: joints(&m.state, "Tick.state")?,
            wrist: m.wrist.map(TryInto::try_into).transpose()?,
            scene: m.scene.map(TryInto::try_into).transpose()?,
            action: joints(&m.action, "Tick.action")?,
            source: m.source.try_into()?,
        })
    }
}

impl From<&Chunk> for pb::Chunk {
    fn from(m: &Chunk) -> Self {
        pb::Chunk {
            t_obs_ns: m.t_obs.0,
            k: m.actions.len() as u32,
            actions: m.actions.iter().flat_map(|j| j.0).collect(),
            ckpt_id: m.ckpt_id.clone(),
        }
    }
}

impl TryFrom<pb::Chunk> for Chunk {
    type Error = ContractError;
    fn try_from(m: pb::Chunk) -> Result<Self> {
        if m.actions.len() != m.k as usize * JOINT_COUNT {
            return Err(ContractError::new(format!(
                "Chunk: k = {} needs {} values, got {}",
                m.k,
                m.k as usize * JOINT_COUNT,
                m.actions.len()
            )));
        }
        let actions = m.actions.chunks_exact(JOINT_COUNT).map(|r| joints(r, "Chunk.actions")).collect::<Result<_>>()?;
        Ok(Chunk { t_obs: Timestamp(m.t_obs_ns), actions, ckpt_id: m.ckpt_id })
    }
}

impl From<&LogDrop> for pb::LogDrop {
    fn from(m: &LogDrop) -> Self {
        pb::LogDrop {
            t_ns: m.t.0,
            topic: m.topic.clone(),
            first_sequence: m.first_sequence,
            last_sequence: m.last_sequence,
            count: m.count,
        }
    }
}

impl From<pb::LogDrop> for LogDrop {
    fn from(m: pb::LogDrop) -> Self {
        LogDrop {
            t: Timestamp(m.t_ns),
            topic: m.topic,
            first_sequence: m.first_sequence,
            last_sequence: m.last_sequence,
            count: m.count,
        }
    }
}

impl From<&Frame> for fpb::CompressedImage {
    fn from(f: &Frame) -> Self {
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
}

impl Frame {
    /// Rebuild a frame from its image message and its MCAP record's `sequence` and `log_time`.
    pub fn from_pb(img: fpb::CompressedImage, sequence: u32, t_receive: Timestamp) -> Result<Frame> {
        let ts = img.timestamp.ok_or_else(|| ContractError::new("CompressedImage: missing timestamp"))?;
        if ts.seconds < 0 || !(0..1_000_000_000).contains(&ts.nanos) {
            return Err(ContractError::new("CompressedImage: timestamp out of range"));
        }
        let camera = Camera::from_name(&img.frame_id)
            .ok_or_else(|| ContractError::new(format!("CompressedImage: unknown camera {:?}", img.frame_id)))?;
        Ok(Frame {
            camera,
            sequence,
            t_capture: Timestamp(ts.seconds as u64 * 1_000_000_000 + ts.nanos as u64),
            t_receive,
            jpeg: img.data,
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
    fn chunk_shape_is_checked() {
        let bad = pb::Chunk { t_obs_ns: 0, k: 2, actions: vec![0.0; 6], ckpt_id: "x".into() };
        assert!(Chunk::try_from(bad).is_err());
    }

    #[test]
    fn missing_source_is_rejected() {
        let m = pb::JointTarget { t_ns: 0, pos: vec![0.0; 6], source: None };
        assert!(JointTarget::try_from(m).is_err());
    }
}
