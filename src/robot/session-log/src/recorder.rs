//! The recorder: one thread owns the MCAP writer and writes every envelope it's given.
//!
//! Behaviour under a slow disk (docs/phase0.md): the control loop always wins. `accept` never
//! blocks. The queue is bounded; camera frames have their own, smaller budget inside it, so when
//! the writer falls behind, frames are dropped first, each contiguous run recorded on
//! `/log/drops`. Only if the whole queue fills (the disk is effectively dead) does the recorder
//! stop accepting, and the session-end record says the log is incomplete.

use std::collections::{BTreeMap, HashMap};
use std::io::{Seek, Write};
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::mpsc::{self, SyncSender, TrySendError};
use std::sync::{Arc, Mutex, RwLock};
use std::thread::JoinHandle;

use contracts::{Clock, LogDrop, DESCRIPTOR_SET};
use mcap::records::{MessageHeader, Metadata};
use mcap::{WriteOptions, Writer};

use crate::header::{META_SESSION, META_SESSION_END};
use crate::{Attachment, Envelope, Error, Event, Header, Result, SessionEnd, Sink};

#[derive(Clone, Debug)]
pub struct RecorderOptions {
    /// How many camera frames may wait to be written before new ones are dropped. The default,
    /// 90, is about 1.5 s of two 30 fps cameras.
    pub frame_queue: usize,
    /// How many envelopes of any kind may wait. If this fills, the recorder stops.
    pub queue: usize,
    /// MCAP chunk size: a kill or power loss loses at most the last chunk.
    pub chunk_size: u64,
}

impl Default for RecorderOptions {
    fn default() -> Self {
        Self { frame_queue: 90, queue: 100_000, chunk_size: 1024 * 1024 }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RecordSummary {
    pub messages_written: u64,
    pub frames_dropped: u64,
    pub messages_lost: u64,
    pub complete: bool,
}

#[derive(Default)]
struct Drops {
    /// The run of dropped frames still growing, per topic: (first, last, count).
    open: HashMap<&'static str, (u32, u32, u32)>,
    /// Finished runs, waiting to be written.
    closed: Vec<(&'static str, u32, u32, u32)>,
}

struct Shared {
    tx: RwLock<Option<SyncSender<Envelope>>>,
    frame_budget: usize,
    queued_frames: AtomicUsize,
    drops: Mutex<Drops>,
    frames_dropped: AtomicU64,
    overflowed: AtomicBool,
    lost: AtomicU64,
}

impl Shared {
    fn drop_frame(&self, topic: &'static str, sequence: u32) {
        self.frames_dropped.fetch_add(1, Ordering::Relaxed);
        let mut d = self.drops.lock().expect("drops lock");
        match d.open.get_mut(topic) {
            Some(run) if run.1.wrapping_add(1) == sequence => {
                run.1 = sequence;
                run.2 += 1;
            }
            Some(run) => {
                let old = *run;
                *run = (sequence, sequence, 1);
                d.closed.push((topic, old.0, old.1, old.2));
            }
            None => {
                d.open.insert(topic, (sequence, sequence, 1));
            }
        }
    }

    fn frame_got_through(&self, topic: &'static str) {
        let mut d = self.drops.lock().expect("drops lock");
        if let Some(run) = d.open.remove(topic) {
            d.closed.push((topic, run.0, run.1, run.2));
        }
    }

    fn take_drops(&self, all: bool) -> Vec<(&'static str, u32, u32, u32)> {
        let mut d = self.drops.lock().expect("drops lock");
        let mut out = std::mem::take(&mut d.closed);
        if all {
            out.extend(d.open.drain().map(|(t, r)| (t, r.0, r.1, r.2)));
        }
        out
    }
}

impl Sink for Shared {
    fn accept(&self, envelope: &Envelope) {
        if self.overflowed.load(Ordering::Relaxed) {
            self.lost.fetch_add(1, Ordering::Relaxed);
            return;
        }
        let guard = self.tx.read().expect("recorder lock");
        let Some(tx) = guard.as_ref() else {
            self.lost.fetch_add(1, Ordering::Relaxed);
            return;
        };
        let frame = envelope.event.is_frame();
        if frame {
            if self.queued_frames.load(Ordering::Relaxed) >= self.frame_budget {
                self.drop_frame(envelope.topic(), envelope.sequence);
                return;
            }
            self.queued_frames.fetch_add(1, Ordering::Relaxed);
        }
        match tx.try_send(envelope.clone()) {
            Ok(()) => {
                if frame {
                    self.frame_got_through(envelope.topic());
                }
            }
            Err(TrySendError::Full(_)) if frame => {
                self.queued_frames.fetch_sub(1, Ordering::Relaxed);
                self.drop_frame(envelope.topic(), envelope.sequence);
            }
            Err(TrySendError::Full(_)) => {
                self.overflowed.store(true, Ordering::Relaxed);
                self.lost.fetch_add(1, Ordering::Relaxed);
            }
            Err(TrySendError::Disconnected(_)) => {
                self.lost.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

/// Owns the writer thread. Attach [`Recorder::sink`] to a [`crate::Bus`]; call
/// [`Recorder::finish`] to close the file. Dropping it (for instance while unwinding a panic)
/// also finishes the file.
pub struct Recorder<W: Write + Seek + Send + 'static> {
    shared: Arc<Shared>,
    thread: Option<JoinHandle<Result<(W, RecordSummary)>>>,
}

impl<W: Write + Seek + Send + 'static> Recorder<W> {
    /// Start a log on `out`: writes the header (and attachments) now, then records on a thread.
    pub fn start(
        out: W,
        header: &Header,
        attachments: &[Attachment],
        clock: Arc<dyn Clock>,
        options: RecorderOptions,
    ) -> Result<Self> {
        let mut writer =
            WriteOptions::new().compression(None).chunk_size(Some(options.chunk_size)).profile("so101").create(out)?;
        writer.write_metadata(&Metadata { name: META_SESSION.into(), metadata: header.to_metadata() })?;
        for a in attachments {
            writer.attach(&mcap::Attachment {
                log_time: header.start_monotonic.0,
                create_time: header.start_monotonic.0,
                name: a.name.clone(),
                media_type: a.media_type.clone(),
                data: std::borrow::Cow::Borrowed(&a.data),
            })?;
        }

        let (tx, rx) = mpsc::sync_channel::<Envelope>(options.queue);
        let shared = Arc::new(Shared {
            tx: RwLock::new(Some(tx)),
            frame_budget: options.frame_queue,
            queued_frames: AtomicUsize::new(0),
            drops: Mutex::new(Drops::default()),
            frames_dropped: AtomicU64::new(0),
            overflowed: AtomicBool::new(false),
            lost: AtomicU64::new(0),
        });
        let thread_shared = Arc::clone(&shared);
        let thread = std::thread::Builder::new()
            .name("recorder".into())
            .spawn(move || {
                let mut w = LogWriter { writer, schemas: HashMap::new(), channels: HashMap::new(), written: 0 };
                let mut drop_sequence = 0u32;
                let mut write_drops = |w: &mut LogWriter<W>, all: bool| -> Result<()> {
                    for (topic, first, last, count) in thread_shared.take_drops(all) {
                        let t = clock.now();
                        let drop =
                            LogDrop { t, topic: topic.to_string(), first_sequence: first, last_sequence: last, count };
                        w.write(&Envelope::new(Event::LogDrop(drop), drop_sequence, t))?;
                        drop_sequence = drop_sequence.wrapping_add(1);
                    }
                    Ok(())
                };
                for envelope in rx {
                    if envelope.event.is_frame() {
                        thread_shared.queued_frames.fetch_sub(1, Ordering::Relaxed);
                    }
                    w.write(&envelope)?;
                    write_drops(&mut w, false)?;
                }
                write_drops(&mut w, true)?;
                let summary = RecordSummary {
                    messages_written: w.written,
                    frames_dropped: thread_shared.frames_dropped.load(Ordering::Relaxed),
                    messages_lost: thread_shared.lost.load(Ordering::Relaxed),
                    complete: !thread_shared.overflowed.load(Ordering::Relaxed),
                };
                let end = SessionEnd {
                    complete: summary.complete,
                    messages_written: summary.messages_written,
                    frames_dropped: summary.frames_dropped,
                    messages_lost: summary.messages_lost,
                };
                w.writer.write_metadata(&Metadata { name: META_SESSION_END.into(), metadata: end.to_metadata() })?;
                w.writer.finish()?;
                Ok((w.writer.into_inner(), summary))
            })
            .map_err(Error::Io)?;
        Ok(Self { shared, thread: Some(thread) })
    }

    /// The bus sink feeding this recorder.
    pub fn sink(&self) -> Arc<dyn Sink> {
        self.shared.clone()
    }

    /// Stop accepting, write everything queued, close the file, and hand back the writer.
    pub fn finish(mut self) -> Result<(W, RecordSummary)> {
        self.close()
    }

    fn close(&mut self) -> Result<(W, RecordSummary)> {
        self.shared.tx.write().expect("recorder lock").take();
        let thread = self.thread.take().ok_or_else(|| Error::Format("recorder already finished".into()))?;
        thread.join().map_err(|_| Error::Format("recorder thread panicked".into()))?
    }
}

impl<W: Write + Seek + Send + 'static> std::ops::Drop for Recorder<W> {
    fn drop(&mut self) {
        if self.thread.is_some() {
            let _ = self.close();
        }
    }
}

struct LogWriter<W: Write + Seek> {
    writer: Writer<W>,
    schemas: HashMap<&'static str, u16>,
    channels: HashMap<&'static str, u16>,
    written: u64,
}

impl<W: Write + Seek> LogWriter<W> {
    fn write(&mut self, envelope: &Envelope) -> Result<()> {
        let topic = envelope.topic();
        let channel_id = match self.channels.get(topic) {
            Some(id) => *id,
            None => {
                let schema = envelope.event.schema();
                let schema_id = match self.schemas.get(schema) {
                    Some(id) => *id,
                    None => {
                        let id = self.writer.add_schema(schema, "protobuf", DESCRIPTOR_SET)?;
                        self.schemas.insert(schema, id);
                        id
                    }
                };
                let id = self.writer.add_channel(schema_id, topic, "protobuf", &BTreeMap::new())?;
                self.channels.insert(topic, id);
                id
            }
        };
        let header = MessageHeader {
            channel_id,
            sequence: envelope.sequence,
            log_time: envelope.log_time.0,
            publish_time: envelope.publish_time.0,
        };
        self.writer.write_to_known_channel(&header, &envelope.event.encode())?;
        self.written += 1;
        Ok(())
    }
}
