//! The contracts every part of the robot speaks (docs/phase0.md).
//!
//! - [`pb`]: the protobuf messages generated from `src/proto/`, the single definition of everything
//!   that lands in a log. Only the log (and later the Python bindings) should touch these.
//! - The domain types ([`JointState`], [`Frame`], [`Tick`], ...): fixed arrays and validated
//!   fields, with conversions to and from [`pb`]. Traits and the rest of the code use these.
//! - [`Clock`]: one monotonic time base, real or replayed.
//! - The traits the runtime is built from: [`ArmDriver`], [`FrameSource`], [`TargetSource`],
//!   [`Policy`].

mod convert;
mod error;
mod time;
mod traits;
mod types;

pub use error::ContractError;
pub use time::{Clock, MonotonicClock, ReplayClock, ReplaySpeed, Timestamp};
pub use traits::{ArmDriver, DynError, FrameSource, Obs, Policy, TargetSource};
pub use types::{
    Camera, Chunk, Frame, FrameRef, Joint, JointState, JointTarget, Joints, LogDrop, ServoTelemetry, Source, Tick,
    JOINT_COUNT,
};

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
