use std::borrow::Cow;
use std::collections::BTreeMap;

use mcap::read::{ChunkFlattener, Options};
use mcap::records::{AttachmentHeader, MessageHeader, Metadata, Record};

use super::Result;

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
