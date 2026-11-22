//! Rewriting a log that was cut short (kill, power loss) into a complete MCAP file that any
//! reader, including Python's, can open. Keeps everything up to the last complete chunk.

use std::collections::{BTreeMap, HashMap};
use std::io::{Seek, Write};

use mcap::records::{MessageHeader, Metadata};
use mcap::WriteOptions;

use crate::reader::RawLog;
use crate::Result;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RecoverSummary {
    pub messages: usize,
    /// Why the input stopped early, or `None` if it was already complete.
    pub truncated: Option<String>,
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
