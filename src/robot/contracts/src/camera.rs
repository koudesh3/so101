//! Cameras and their frames. A frame is logged as Foxglove's `CompressedImage`; its sequence
//! and receive time live in the MCAP record (contract 3).

use bytes::Bytes;

use crate::pb::foxglove as fpb;
use crate::pb::so101::v1 as pb;
use crate::{ContractError, DynError, Timestamp};

type Result<T> = std::result::Result<T, ContractError>;

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

/// A camera, real (V4L2), replayed or synthetic. Blocks until the next frame.
pub trait FrameSource: Send {
    fn camera(&self) -> Camera;
    fn next_frame(&mut self) -> std::result::Result<Frame, DynError>;
}

// ---- protobuf conversions ----

impl From<Camera> for pb::Camera {
    fn from(c: Camera) -> Self {
        match c {
            Camera::Wrist => pb::Camera::Wrist,
            Camera::Scene => pb::Camera::Scene,
        }
    }
}

fn camera_from_pb(v: i32) -> Result<Camera> {
    match pb::Camera::try_from(v) {
        Ok(pb::Camera::Wrist) => Ok(Camera::Wrist),
        Ok(pb::Camera::Scene) => Ok(Camera::Scene),
        _ => Err(ContractError::new(format!("unknown camera {v}"))),
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
        Ok(FrameRef { camera: camera_from_pb(r.camera)?, sequence: r.sequence })
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
