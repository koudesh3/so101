//! Reading a session back: open it, check its calibration hashes, find dropped messages, replay
//! it in time, and recover a file that was cut short (kill, power loss).

use std::borrow::Cow;
use std::collections::{BTreeMap, HashMap};
use std::io::{Seek, Write};
use std::path::Path;
use std::time::Duration;

use mcap::read::{ChunkFlattener, Options};
use mcap::records::{AttachmentHeader, MessageHeader, Metadata, Record};
use mcap::WriteOptions;

use super::wire::TOPIC_RECORDER_DROPS;
use super::{calibration_attachment, sha256_hex, Attachment, Error, Header, Result, SessionEnd};
use super::{META_SESSION, META_SESSION_END};
use crate::{Clock, Event, Message, Timestamp};

#[derive(Debug)]
pub struct Session {
    pub header: Header,
    /// `None` if the writer never finished (killed, power loss).
    pub end: Option<SessionEnd>,
    /// Every event, in file order (the order the recorder received them).
    pub events: Vec<Event>,
    pub attachments: Vec<Attachment>,
}

pub(crate) struct RawLog<'a> {
    /// id -> (name, encoding, data)
    pub schemas: BTreeMap<u16, (String, String, Cow<'a, [u8]>)>,
    /// id -> (topic, schema id, message encoding, metadata)
    pub channels: BTreeMap<u16, (String, u16, String, BTreeMap<String, String>)>,
    pub messages: Vec<(MessageHeader, Cow<'a, [u8]>)>,
    pub metadata: Vec<Metadata>,
    pub attachments: Vec<(AttachmentHeader, Cow<'a, [u8]>)>,
    /// Why parsing stopped early, if it did (only in lenient mode).
    pub truncated: Option<String>,
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

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RecoverSummary {
    pub messages: usize,
    /// Why the input stopped early, or `None` if it was already complete.
    pub truncated: Option<String>,
}

impl Session {
    /// Reads the whole file into memory. A damaged or truncated file is an error: recover it
    /// first (`so101 log recover`).
    pub fn open(path: impl AsRef<Path>) -> Result<Session> {
        Self::from_bytes(&std::fs::read(path)?)
    }

    pub fn from_bytes(bytes: &[u8]) -> Result<Session> {
        Self::decode(RawLog::parse(bytes, false)?)
    }

    pub(crate) fn decode(raw: RawLog<'_>) -> Result<Session> {
        let mut header = None;
        let mut end = None;
        for m in &raw.metadata {
            match m.name.as_str() {
                META_SESSION => header = Some(Header::from_metadata(&m.metadata)?),
                META_SESSION_END => end = Some(SessionEnd::from_metadata(&m.metadata)?),
                _ => {}
            }
        }
        let header = header.ok_or_else(|| Error::Format("no session header".into()))?;
        let topics: HashMap<u16, &str> = raw.channels.iter().map(|(id, c)| (*id, c.0.as_str())).collect();
        let events = raw
            .messages
            .iter()
            .map(|(h, data)| {
                let topic = topics
                    .get(&h.channel_id)
                    .ok_or_else(|| Error::Format(format!("message on unknown channel {}", h.channel_id)))?;
                let log_time = Timestamp(h.log_time);
                Ok(Event {
                    sequence: h.sequence,
                    log_time,
                    publish_time: Timestamp(h.publish_time),
                    message: Message::decode(topic, data, h.sequence, log_time)?,
                })
            })
            .collect::<Result<Vec<_>>>()?;
        let attachments = raw
            .attachments
            .iter()
            .map(|(h, d)| Attachment { name: h.name.clone(), media_type: h.media_type.clone(), data: d.to_vec() })
            .collect();
        let log = Session { header, end, events, attachments };
        for (name, hash) in &log.header.calibrations {
            let file = log
                .calibration(name)
                .ok_or_else(|| Error::Format(format!("calibration {name:?} is named in the header but missing")))?;
            if sha256_hex(file) != *hash {
                return Err(Error::Format(format!("calibration {name:?} doesn't match its hash in the header")));
            }
        }
        Ok(log)
    }

    /// The calibration file a named arm's degrees were computed with.
    pub fn calibration(&self, name: &str) -> Option<&[u8]> {
        let want = calibration_attachment(name);
        self.attachments.iter().find(|a| a.name == want).map(|a| a.data.as_slice())
    }

    /// Events ordered by `log_time` (ties keep file order): the order to replay them in.
    pub fn by_time(&self) -> Vec<&Event> {
        let mut v: Vec<&Event> = self.events.iter().collect();
        v.sort_by_key(|e| e.log_time);
        v
    }

    pub fn topic<'a>(&'a self, topic: &'a str) -> impl Iterator<Item = &'a Event> + 'a {
        self.events.iter().filter(move |e| e.topic() == topic)
    }
}

impl<'a> RawLog<'a> {
    /// Parse every record. Strict mode fails on a damaged or truncated file; lenient mode keeps
    /// everything before the damage and says where it stopped.
    pub fn parse(bytes: &'a [u8], lenient: bool) -> Result<Self> {
        let records = if lenient {
            ChunkFlattener::new_with_options(bytes, Options::IgnoreEndMagic.into())
        } else {
            ChunkFlattener::new(bytes)
        };
        let mut raw = RawLog {
            schemas: BTreeMap::new(),
            channels: BTreeMap::new(),
            messages: Vec::new(),
            metadata: Vec::new(),
            attachments: Vec::new(),
            truncated: None,
        };
        let mut records = match records {
            Ok(r) => r,
            Err(e) if lenient => {
                raw.truncated = Some(e.to_string());
                return Ok(raw);
            }
            Err(e) => return Err(e.into()),
        };
        for record in &mut records {
            let record = match record {
                Ok(r) => r,
                Err(e) if lenient => {
                    raw.truncated = Some(e.to_string());
                    break;
                }
                Err(e) => return Err(e.into()),
            };
            match record {
                Record::Schema { header, data } => {
                    raw.schemas.entry(header.id).or_insert((header.name, header.encoding, data));
                }
                Record::Channel(c) => {
                    raw.channels.entry(c.id).or_insert((c.topic, c.schema_id, c.message_encoding, c.metadata));
                }
                Record::Message { header, data } => raw.messages.push((header, data)),
                Record::Metadata(m) => raw.metadata.push(m),
                Record::Attachment { header, data, .. } => raw.attachments.push((header, data)),
                _ => {}
            }
        }
        Ok(raw)
    }
}

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

/// Hand each event to `f` at its `log_time` on `clock`. Pass events in time order
/// (`Session::by_time`) and a `ReplayClock` started at the first one's `log_time`.
pub fn replay<'a>(events: impl IntoIterator<Item = &'a Event>, clock: &dyn Clock, mut f: impl FnMut(&Event)) {
    for e in events {
        clock.sleep_until(e.log_time);
        f(e);
    }
}

pub fn recover<W: Write + Seek>(input: &[u8], out: W) -> Result<(W, RecoverSummary)> {
    let raw = RawLog::parse(input, true)?;
    let mut w = WriteOptions::new().compression(None).profile("so101").create(out)?;
    let mut schema_ids = HashMap::new();
    for (old, (name, encoding, data)) in &raw.schemas {
        schema_ids.insert(*old, w.add_schema(name, encoding, data)?);
    }
    let mut channel_ids = HashMap::new();
    for (old, (topic, schema, encoding, metadata)) in &raw.channels {
        let schema = schema_ids.get(schema).copied().unwrap_or(0);
        channel_ids.insert(*old, w.add_channel(schema, topic, encoding, metadata)?);
    }
    for m in &raw.metadata {
        w.write_metadata(m)?;
    }
    for (h, data) in &raw.attachments {
        w.attach(&mcap::Attachment {
            log_time: h.log_time,
            create_time: h.create_time,
            name: h.name.clone(),
            media_type: h.media_type.clone(),
            data: data.clone(),
        })?;
    }
    let mut messages = 0;
    for (h, data) in &raw.messages {
        // A message whose channel record was lost can't be decoded; skip it.
        let Some(&channel_id) = channel_ids.get(&h.channel_id) else { continue };
        w.write_to_known_channel(&MessageHeader { channel_id, ..*h }, data)?;
        messages += 1;
    }
    let note = BTreeMap::from([
        ("messages".to_string(), messages.to_string()),
        ("truncated".to_string(), raw.truncated.clone().unwrap_or_default()),
    ]);
    w.write_metadata(&Metadata { name: "so101.recovered".into(), metadata: note })?;
    w.finish()?;
    Ok((w.into_inner(), RecoverSummary { messages, truncated: raw.truncated }))
}
