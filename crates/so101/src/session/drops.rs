use std::collections::{BTreeMap, HashMap};
use std::time::Duration;

use super::event::TOPIC_RECORDER_DROPS;
use super::Session;
use crate::{Event, Message};

impl Session {
    /// Every drop in the log (contract 3), by topic then sequence.
    pub fn drops(&self) -> Vec<Drop> {
        let mut out = Vec::new();
        // What the recorder itself dropped, so those gaps aren't reported twice.
        let mut recorder_ranges: HashMap<&str, Vec<(u32, u32)>> = HashMap::new();
        for e in &self.events {
            if let Message::RecorderDrop(d) = &e.message {
                recorder_ranges.entry(d.topic.as_str()).or_default().push((d.first_sequence, d.last_sequence));
                out.push(Drop {
                    topic: d.topic.clone(),
                    kind: DropKind::Recorder { first: d.first_sequence, last: d.last_sequence, count: d.count },
                });
            }
        }
        let mut by_topic: BTreeMap<&str, Vec<&Event>> = BTreeMap::new();
        for e in &self.events {
            if e.topic() != TOPIC_RECORDER_DROPS {
                by_topic.entry(e.topic()).or_default().push(e);
            }
        }
        for (topic, envs) in by_topic {
            let ranges = recorder_ranges.get(topic).map(Vec::as_slice).unwrap_or(&[]);
            let by_recorder = |s: u32| ranges.iter().any(|&(a, b)| a <= s && s <= b);
            let is_camera = envs.first().is_some_and(|e| e.message.is_frame());
            let nominal = if is_camera {
                let mut iv: Vec<u64> = envs
                    .windows(2)
                    .filter(|w| w[1].sequence == w[0].sequence.wrapping_add(1))
                    .map(|w| w[1].publish_time.0.saturating_sub(w[0].publish_time.0))
                    .collect();
                iv.sort_unstable();
                iv.get(iv.len() / 2).copied()
            } else {
                None
            };
            for w in envs.windows(2) {
                let (a, b) = (w[0], w[1]);
                let gap = b.sequence.wrapping_sub(a.sequence).wrapping_sub(1);
                if gap > 0 {
                    let missing = (1..=gap).filter(|i| !by_recorder(a.sequence.wrapping_add(*i))).count() as u32;
                    if missing > 0 {
                        out.push(Drop {
                            topic: topic.to_string(),
                            kind: DropKind::SequenceGap { after: a.sequence, missing },
                        });
                    }
                } else if let Some(nominal) = nominal {
                    let interval = b.publish_time.0.saturating_sub(a.publish_time.0);
                    if nominal > 0 && interval * 2 > nominal * 3 {
                        out.push(Drop {
                            topic: topic.to_string(),
                            kind: DropKind::LongCaptureInterval {
                                after: a.sequence,
                                interval: Duration::from_nanos(interval),
                                nominal: Duration::from_nanos(nominal),
                            },
                        });
                    }
                }
            }
        }
        out
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Drop {
    pub topic: String,
    pub kind: DropKind,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DropKind {
    /// The host lost messages: the sequence skipped `missing` numbers after `after`.
    SequenceGap { after: u32, missing: u32 },
    /// No sequence gap, but the capture interval after frame `after` was over 1.5 × the
    /// camera's nominal period: the camera itself skipped (the counter can't show that).
    LongCaptureInterval { after: u32, interval: Duration, nominal: Duration },
    /// The recorder dropped this run because it fell behind (`/log/drops`).
    Recorder { first: u32, last: u32, count: u32 },
}
