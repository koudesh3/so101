//! The session log (docs/phase0.md): the in-process event bus, the MCAP recorder, the reader
//! (with drop detection), replay, and recovery of files cut short.
//!
//! Every message is protobuf (`contracts::pb`) inside an MCAP file. Each record carries
//! `publish_time` (when the thing happened), `log_time` (when our process received it) and
//! `sequence` (the producer's counter), so channels line up and drops show without decoding.

mod bus;
mod drops;
mod error;
mod event;
mod header;
mod raw;
mod reader;
mod recorder;
mod recover;
mod replay;
#[doc(hidden)]
pub mod sample;

pub use bus::{Bus, Sink};
pub use drops::{Drop, DropKind};
pub use error::{Error, Result};
pub use event::{Arm, Envelope, Event};
pub use header::{Attachment, CameraInfo, Header, SessionEnd};
pub use reader::Log;
pub use recorder::{RecordSummary, Recorder, RecorderOptions};
pub use recover::{recover, RecoverSummary};
pub use replay::replay;
