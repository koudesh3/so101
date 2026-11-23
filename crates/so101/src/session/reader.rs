use std::collections::HashMap;
use std::path::Path;

use crate::Clock;

use crate::Timestamp;

use super::header::{calibration_attachment, sha256_hex, META_SESSION, META_SESSION_END};
use super::raw::RawLog;
use super::{Attachment, Error, Header, Result, SessionEnd};
use crate::{Event, Message};

#[derive(Debug)]
pub struct Session {
    pub header: Header,
    /// `None` if the writer never finished (killed, power loss).
    pub end: Option<SessionEnd>,
    /// Every event, in file order (the order the recorder received them).
    pub events: Vec<Event>,
    pub attachments: Vec<Attachment>,
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

/// Hand each event to `f` at its `log_time` on `clock`. Pass events in time order
/// (`Session::by_time`) and a `ReplayClock` started at the first one's `log_time`.
pub fn replay<'a>(events: impl IntoIterator<Item = &'a Event>, clock: &dyn Clock, mut f: impl FnMut(&Event)) {
    for e in events {
        clock.sleep_until(e.log_time);
        f(e);
    }
}
