//! The contracts every part of the robot speaks (docs/phase0.md).
//!
//! One module per concept, each holding its domain types, their protobuf conversions and the
//! trait that produces or consumes them. A new phase adds modules rather than growing these.
//!
//! - [`joint`]: joints, positions, telemetry, targets; [`ArmDriver`], [`TargetSource`]
//! - [`camera`]: cameras and frames; [`FrameSource`]
//! - [`tick`]: what the control loop used at each tick (contract 2)
//! - [`policy`]: observations, chunks; [`Policy`] (contract 4)
//! - [`log`]: messages about the log itself
//! - [`time`]: one monotonic time base, real or replayed
//! - [`pb`]: the protobuf messages generated from `proto/`, the single definition of
//!   everything that lands in a log. Only the log (and later the Python bindings) use them.

pub mod camera;
mod error;
pub mod joint;
pub mod log;
pub mod policy;
pub mod tick;
pub mod time;

pub use camera::{Camera, Frame, FrameRef, FrameSource};
pub use error::ContractError;
pub use joint::{ArmDriver, Joint, JointState, JointTarget, Joints, ServoTelemetry, Source, TargetSource, JOINT_COUNT};
pub use log::LogDrop;
pub use policy::{Chunk, Obs, Policy};
pub use tick::Tick;
pub use time::{Clock, MonotonicClock, ReplayClock, ReplaySpeed, Timestamp};

/// The error a trait implementation returns: whatever the hardware or model layer produced.
pub type DynError = Box<dyn std::error::Error + Send + Sync>;

/// The generated protobuf types.
pub mod pb {
    pub mod so101 {
        pub mod v1 {
            include!(concat!(env!("OUT_DIR"), "/so101.v1.rs"));
        }
    }
    pub mod foxglove {
        include!(concat!(env!("OUT_DIR"), "/foxglove.rs"));
    }
}

/// The encoded `FileDescriptorSet` of every message (so101.v1, foxglove, and their imports).
/// Each MCAP schema record embeds it, so any log can be decoded without generated code.
pub const DESCRIPTOR_SET: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/descriptors.binpb"));
