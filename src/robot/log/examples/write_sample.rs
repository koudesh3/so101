//! Writes the sample session to a file: `cargo run -p session-log --example write_sample -- out.mcap`.
//! `just check` reads it back from Python to prove logs need no generated code (contract 1).

use std::fs::File;
use std::io::BufWriter;
use std::sync::Arc;

use contracts::{MonotonicClock, Timestamp};
use session_log::{Bus, Header, Recorder, RecorderOptions};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args().nth(1).ok_or("usage: write_sample OUT.mcap")?;
    let mut header = Header::now();
    header.git_commit = "sample".into();
    header.config.insert("servo_hz".into(), "120".into());
    let out = BufWriter::new(File::create(&path)?);
    let recorder = Recorder::start(out, &header, &[], Arc::new(MonotonicClock), RecorderOptions::default())?;
    let bus = Bus::new();
    bus.attach(recorder.sink());
    for e in session_log::sample::session(Timestamp(5_000_000_000)) {
        bus.publish(e);
    }
    let (_, summary) = recorder.finish()?;
    println!("wrote {path}: {} messages", summary.messages_written);
    Ok(())
}
