//! The cameras: which one, and the frames they produce, with their capture and receive times.

use bytes::Bytes;

use crate::clock::Timestamp;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Camera {
    Wrist,
    Scene,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct FrameRef {
    pub camera: Camera,
    pub sequence: u32,
}

/// Logged as Foxglove's `CompressedImage`; `t_capture`, `t_receive` and `sequence` are the MCAP
/// record's `publish_time`, `log_time` and `sequence` (contract 3).
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub camera: Camera,
    /// The kernel's frame number for a V4L2 camera.
    pub sequence: u32,
    /// The kernel's buffer timestamp for a V4L2 camera; exposure starts about 10 ms earlier.
    pub t_capture: Timestamp,
    pub t_receive: Timestamp,
    pub jpeg: Bytes,
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

impl Frame {
    pub fn id(&self) -> FrameRef {
        FrameRef { camera: self.camera, sequence: self.sequence }
    }
}
