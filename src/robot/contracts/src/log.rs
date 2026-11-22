//! Messages about the log itself.

use crate::pb::so101::v1 as pb;
use crate::Timestamp;

/// A contiguous run of messages on one topic that the recorder dropped because it fell behind.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LogDrop {
    pub t: Timestamp,
    pub topic: String,
    pub first_sequence: u32,
    pub last_sequence: u32,
    pub count: u32,
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
