//! The session log (docs/phase0.md): protobuf messages in MCAP, one file per session.

mod drops;
mod error;
mod header;
mod raw;
mod reader;
mod recorder;
mod recover;
#[doc(hidden)]
pub mod sample;
mod wire;

pub use drops::{Drop, DropKind};
pub use error::{Error, Result};
pub use header::{Attachment, CameraInfo, Header, SessionEnd};
pub use reader::{replay, Session};
pub use recorder::{RecordSummary, Recorder, RecorderOptions};
pub use recover::{recover, RecoverSummary};

/// Generated from `proto/`.
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

/// Embedded in every MCAP schema record, so any log decodes without generated code.
pub const DESCRIPTOR_SET: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/descriptors.binpb"));
