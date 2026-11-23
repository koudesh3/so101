//! Writes the sample session to a file: `cargo run -p session-log --example write_sample -- out.mcap`.
//! `just check` reads it back from Python to prove logs need no generated code (contract 1).

use std::fs::File;
use std::io::BufWriter;
use std::sync::Arc;

use so101::control::bus::EventBus;
use so101::log::{CameraInfo, Header, Recorder, RecorderOptions};
use so101::{MonotonicClock, Timestamp};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args().nth(1).ok_or("usage: write_sample OUT.mcap")?;
    let mut header = Header::now();
    header.git_commit = "sample".into();
    header.config.insert("servo_hz".into(), "120".into());
    header.cameras.insert(
        "wrist".into(),
        CameraInfo {
            device: "/dev/v4l/by-id/sample-wrist".into(),
            settings: "auto_exposure=1 exposure_time_absolute=300".into(),
        },
    );
    let calibration = header.add_calibration("follower", br#"{"shoulder_pan": {"range_min": 800, "range_max": 3300}}"#);
    let out = BufWriter::new(File::create(&path)?);
    let recorder = Recorder::start(out, &header, &[calibration], Arc::new(MonotonicClock), RecorderOptions::default())?;
    let bus = EventBus::new();
    bus.attach(recorder.sink());
    for e in so101::log::sample::session(Timestamp(5_000_000_000)) {
        bus.publish(e);
    }
    let (_, summary) = recorder.finish()?;
    println!("wrote {path}: {} messages", summary.messages_written);
    Ok(())
}
