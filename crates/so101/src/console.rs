//! The teleop console: one page served by teleop on the Spark, on 127.0.0.1 only (`just console`
//! tunnels it to the laptop), with both cameras live, the follower's load, and a record button.
//! A recording is the session's MCAP (events and the wrist camera) plus the scene camera beside
//! it: saved raw while recording and encoded after it stops (`scene_video`), with a CSV tying
//! each scene frame to the MCAP's clock.

use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::net::{TcpListener, TcpStream};
use std::path::PathBuf;
use std::process::Command;
use std::sync::{Arc, Condvar, Mutex, RwLock};
use std::time::Instant;

use bytes::Bytes;
use so101::arm::{ActuatorStatus, Joint, Limit, JOINT_COUNT};
use so101::camera::Frame;
use so101::clock::{Clock, Timestamp};
use so101::control::{Event, Health, Message, Sink};
use so101::session::{Attachment, Header, Recorder, RecorderOptions};

use crate::scene_video::{self, SceneRaw};

const PAGE: &str = include_str!("console.html");

/// The newest JPEG from one camera, and a count that says when it changed.
#[derive(Default)]
pub struct Feed {
    frame: Mutex<(u64, Bytes)>,
    fresh: Condvar,
}

impl Feed {
    pub fn put(&self, jpeg: Bytes) {
        let mut frame = self.frame.lock().expect("feed");
        *frame = (frame.0 + 1, jpeg);
        self.fresh.notify_all();
    }
}

/// The follower's latest load and current per joint, its supply voltage, and which joints a
/// guard is holding.
#[derive(Default)]
pub struct Live {
    load_percent: [f32; JOINT_COUNT],
    current_a: [f32; JOINT_COUNT],
    // Voltage and temperature through the health filter (median of 5), not raw.
    health: Health,
    pub held: [Option<Limit>; JOINT_COUNT],
}

impl Live {
    pub fn add(&mut self, a: &ActuatorStatus) {
        let i = a.joint.index();
        self.load_percent[i] = a.load_percent.unwrap_or(self.load_percent[i]);
        self.current_a[i] = a.current_a.unwrap_or(self.current_a[i]);
        self.health.add(a);
    }
}

/// What a recording needs from teleop: where files go, and a fresh header for each one.
pub struct RecordingSetup {
    pub dir: PathBuf,
    /// Whether there's a scene camera to save beside the MCAP.
    pub scene_video: bool,
    pub clock: Arc<dyn Clock>,
    pub header: Box<dyn Fn() -> (Header, Vec<Attachment>) + Send + Sync>,
}

/// Starts and stops recordings while teleop runs. Attach it to the bus once: it passes events to
/// the open recording's MCAP, if there is one.
///
/// A recording starts clean: every tick in it names frames that are in it. Between recordings it
/// keeps each camera's newest frame and writes it first when one starts (the first ticks decide
/// on it), and ticks decided before the start are left out, as their follower state is.
pub struct Recording {
    setup: RecordingSetup,
    active: Mutex<Option<Active>>,
    // The open recording's MCAP, and when it started. Taken for writing only to start or stop.
    sink: RwLock<Option<(Arc<dyn Sink>, Timestamp)>>,
    last: Mutex<Option<String>>,
    // Each camera's newest frame while nothing records.
    idle_wrist: Mutex<Option<Event>>,
    idle_scene: Mutex<Option<Frame>>,
}

struct Active {
    name: String,
    started: Instant,
    recorder: Recorder<BufWriter<File>>,
    // The scene camera's frames, saved raw; encoded after the recording stops.
    scene: Option<SceneRaw>,
    prefix: PathBuf,
}

impl Recording {
    pub fn new(setup: RecordingSetup) -> Self {
        Self {
            setup,
            active: Mutex::new(None),
            sink: RwLock::new(None),
            last: Mutex::new(None),
            idle_wrist: Mutex::new(None),
            idle_scene: Mutex::new(None),
        }
    }

    pub fn is_recording(&self) -> bool {
        self.active.lock().expect("recording").is_some()
    }

    /// Opens `<dir>/<date>-<time>.mcap`, `-scene.mkv` and `-scene.csv`; returns the name.
    pub fn start(&self) -> Result<String, String> {
        let mut active = self.active.lock().expect("recording");
        if let Some(a) = active.as_ref() {
            return Err(format!("already recording {}", a.name));
        }
        std::fs::create_dir_all(&self.setup.dir).map_err(|e| format!("{}: {e}", self.setup.dir.display()))?;
        let name = local_stamp();
        let path = |suffix: &str| self.setup.dir.join(format!("{name}{suffix}"));
        let (header, attachments) = (self.setup.header)();
        let file = File::options().write(true).create_new(true).open(path(".mcap")).map_err(|e| e.to_string())?;
        let recorder = Recorder::start(
            BufWriter::new(file),
            &header,
            &attachments,
            self.setup.clock.clone(),
            RecorderOptions::default(),
        )
        .map_err(|e| e.to_string())?;
        let prefix = self.setup.dir.join(&name);
        let mut scene = match self.setup.scene_video {
            true => Some(SceneRaw::create(&prefix).map_err(|e| e.to_string())?),
            false => None,
        };
        // Holding `active` keeps scene frames out until the held one is written; holding the sink
        // for writing does the same for events, so nothing lands between the held frame and the
        // ones after it.
        if let (Some(raw), Some(frame)) = (scene.as_mut(), self.idle_scene.lock().expect("idle").take()) {
            raw.add(&frame).map_err(|e| e.to_string())?;
        }
        let mut sink = self.sink.write().expect("sink");
        let mcap = recorder.sink();
        if let Some(frame) = self.idle_wrist.lock().expect("idle").take() {
            mcap.accept(&frame);
        }
        *sink = Some((mcap, self.setup.clock.now()));
        drop(sink);
        println!("recording {}", path(".mcap").display());
        *active = Some(Active { name: name.clone(), started: Instant::now(), recorder, scene, prefix });
        Ok(name)
    }

    /// Closes the files; returns a one-line summary.
    pub fn stop(&self) -> Result<String, String> {
        let Some(a) = self.active.lock().expect("recording").take() else {
            return Err("not recording".into());
        };
        *self.sink.write().expect("sink") = None;
        let seconds = a.started.elapsed().as_secs_f64();
        let (_, summary) = a.recorder.finish().map_err(|e| e.to_string())?;
        let scene_frames = a.scene.map(SceneRaw::finish).transpose().map_err(|e| e.to_string())?;
        // (Tests stop at the raw files: the encoder is this program's own subcommand.)
        if scene_frames.is_some() && !cfg!(test) {
            // Off the live path: its own low-priority process, which outlives teleop if need be.
            if let Err(e) = scene_video::encode_in_background(&a.prefix) {
                eprintln!("scene video stays raw ({e}); `just encode-scenes` encodes it");
            }
        }
        let scene_frames = scene_frames.unwrap_or(0);
        let line = format!(
            "{}: {seconds:.0} s, {} messages, {} wrist frames dropped, {scene_frames} scene frames{}",
            a.name,
            summary.messages_written,
            summary.frames_dropped,
            if summary.complete { "" } else { " (INCOMPLETE: the disk fell behind)" }
        );
        println!("recorded {line}");
        *self.last.lock().expect("last") = Some(line.clone());
        Ok(line)
    }

    /// The scene camera's frames go to the video, not the MCAP.
    pub fn scene_frame(&self, frame: &Frame) {
        let mut active = self.active.lock().expect("recording");
        match active.as_mut() {
            Some(a) => {
                if let Some(Err(e)) = a.scene.as_mut().map(|scene| scene.add(frame)) {
                    eprintln!("scene video: {e}");
                }
            }
            None => *self.idle_scene.lock().expect("idle") = Some(frame.clone()),
        }
    }

    fn json(&self) -> serde_json::Value {
        let active = self.active.lock().expect("recording");
        serde_json::json!({
            "active": active.is_some(),
            "name": active.as_ref().map(|a| a.name.clone()),
            "seconds": active.as_ref().map(|a| a.started.elapsed().as_secs()),
            "last": *self.last.lock().expect("last"),
            "encoding": scene_video::pending(&self.setup.dir)
                .iter()
                .filter_map(|p| p.file_name().map(|n| n.to_string_lossy().into_owned()))
                .collect::<Vec<_>>(),
        })
    }
}

impl Sink for Recording {
    fn accept(&self, event: &Event) {
        let sink = self.sink.read().expect("sink");
        match (sink.as_ref(), &event.message) {
            (None, Message::Frame(_)) => *self.idle_wrist.lock().expect("idle") = Some(event.clone()),
            (None, _) => {}
            (Some((_, started)), Message::Tick(tick)) if tick.t < *started => {}
            (Some((mcap, _)), _) => mcap.accept(event),
        }
    }
}

/// Everything the page shows; shared with teleop's threads.
pub struct Console {
    pub wrist: Arc<Feed>,
    pub scene: Arc<Feed>,
    pub live: Arc<Mutex<Live>>,
    pub recording: Arc<Recording>,
}

impl Console {
    /// Serves the page on 127.0.0.1:`port` from a thread.
    pub fn serve(self: &Arc<Self>, port: u16) -> std::io::Result<()> {
        let listener = TcpListener::bind(("127.0.0.1", port))?;
        let console = self.clone();
        std::thread::spawn(move || {
            for conn in listener.incoming().flatten() {
                let console = console.clone();
                std::thread::spawn(move || console.answer(conn));
            }
        });
        Ok(())
    }

    fn answer(&self, conn: TcpStream) -> std::io::Result<()> {
        let mut reader = BufReader::new(conn.try_clone()?);
        let mut request = String::new();
        reader.read_line(&mut request)?;
        loop {
            let mut header = String::new();
            if reader.read_line(&mut header)? == 0 || header.trim().is_empty() {
                break;
            }
        }
        let mut out = conn;
        match request.split_whitespace().nth(1).unwrap_or("/") {
            "/" => respond(&mut out, "text/html; charset=utf-8", PAGE.as_bytes()),
            "/cam/wrist" => stream(&mut out, &self.wrist),
            "/cam/scene" => stream(&mut out, &self.scene),
            "/api/status" => respond(&mut out, "application/json", self.status(None).as_bytes()),
            "/api/record/start" => {
                let result = self.recording.start().map(|name| format!("recording {name}"));
                respond(&mut out, "application/json", self.status(Some(result)).as_bytes())
            }
            "/api/record/stop" => {
                let result = self.recording.stop();
                respond(&mut out, "application/json", self.status(Some(result)).as_bytes())
            }
            _ => write!(out, "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"),
        }
    }

    fn status(&self, result: Option<Result<String, String>>) -> String {
        let live = self.live.lock().expect("live");
        let round = |v: f32, by: f64| (f64::from(v) * by).round() / by;
        serde_json::json!({
            "joints": Joint::ALL.map(|j| j.name()),
            "load_percent": live.load_percent.map(|v| round(v, 10.0)),
            "current_a": live.current_a.map(|v| round(v, 1000.0)),
            "voltage_v": live.health.supply_v().map(|v| round(v, 10.0)),
            "temperature_c": Joint::ALL.map(|j| live.health.temperature_c(j).map(|c| round(c, 1.0))),
            "held": live.held.map(|h| h.map(|l| format!("{l:?}").to_lowercase())),
            "recording": self.recording.json(),
            "message": result.as_ref().map(|r| r.as_ref().unwrap_or_else(|e| e).clone()),
            "error": result.as_ref().is_some_and(|r| r.is_err()),
        })
        .to_string()
    }
}

fn respond(out: &mut TcpStream, kind: &str, body: &[u8]) -> std::io::Result<()> {
    write!(
        out,
        "HTTP/1.1 200 OK\r\nContent-Type: {kind}\r\nContent-Length: {}\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n",
        body.len()
    )?;
    out.write_all(body)
}

fn stream(out: &mut TcpStream, feed: &Feed) -> std::io::Result<()> {
    write!(
        out,
        "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n"
    )?;
    let mut seen = 0;
    loop {
        let jpeg = {
            let mut frame = feed.frame.lock().expect("feed");
            while frame.0 == seen {
                frame = feed.fresh.wait(frame).expect("feed");
            }
            seen = frame.0;
            frame.1.clone()
        };
        write!(out, "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n", jpeg.len())?;
        out.write_all(&jpeg)?;
        out.write_all(b"\r\n")?;
    }
}

// Local date and time for file names, e.g. 20261124-194512; `date` knows the Spark's timezone.
fn local_stamp() -> String {
    Command::new("date")
        .arg("+%Y%m%d-%H%M%S")
        .output()
        .ok()
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| format!("session-{}", std::process::id()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use so101::arm::Pose;
    use so101::camera::{CameraRole, FrameRef};
    use so101::clock::{ReplayClock, ReplaySpeed};
    use so101::control::{Source, Tick};
    use so101::session::Session;

    const MS: u64 = 1_000_000;

    fn frame(camera: CameraRole, sequence: u32, received_ms: u64) -> Frame {
        Frame {
            camera,
            sequence,
            t_capture: Timestamp((received_ms - 33) * MS),
            t_receive: Timestamp(received_ms * MS),
            jpeg: Bytes::from_static(&[0xFF, 0xD8, 0xFF, 0xD9]),
        }
    }

    fn tick(decided_ms: u64, wrist: u32, scene: u32) -> Event {
        let t = Timestamp(decided_ms * MS);
        let tick = Tick {
            t,
            cycle: 0,
            state: Pose::default(),
            wrist: Some(FrameRef { camera: CameraRole::Wrist, sequence: wrist }),
            scene: Some(FrameRef { camera: CameraRole::Scene, sequence: scene }),
            action: Pose::default(),
            source: Source::Teleop,
            requested: Pose::default(),
            clipped: Vec::new(),
        };
        Event { sequence: 0, log_time: t, publish_time: t, message: Message::Tick(tick) }
    }

    #[test]
    fn a_recording_starts_with_the_frames_its_first_ticks_decided_on() {
        let dir = std::env::temp_dir().join(format!("so101-console-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let clock = Arc::new(ReplayClock::new(Timestamp(1000 * MS), ReplaySpeed::AsFastAsPossible));
        let recording = Recording::new(RecordingSetup {
            dir: dir.clone(),
            scene_video: true,
            clock: clock.clone(),
            header: Box::new(|| (Header::now(), Vec::new())),
        });
        // Before Record: frames arrive, and a tick decides on them.
        recording.accept(&Event::frame(frame(CameraRole::Wrist, 4, 960)));
        recording.accept(&Event::frame(frame(CameraRole::Wrist, 5, 990)));
        recording.scene_frame(&frame(CameraRole::Scene, 7, 995));
        recording.accept(&tick(998, 5, 7));
        let name = recording.start().unwrap();
        // After: the next tick still sees those frames; then new ones arrive.
        clock.sleep_until(Timestamp(1010 * MS));
        recording.accept(&tick(1005, 5, 7));
        recording.accept(&Event::frame(frame(CameraRole::Wrist, 6, 1020)));
        recording.scene_frame(&frame(CameraRole::Scene, 8, 1025));
        recording.stop().unwrap();

        let session = Session::open(dir.join(format!("{name}.mcap"))).unwrap();
        let wrist: Vec<u32> = session.topic("/cam/wrist").map(|e| e.sequence).collect();
        let ticks: Vec<Timestamp> = session
            .topic("/tick")
            .map(|e| match &e.message {
                Message::Tick(t) => t.t,
                other => panic!("{other:?}"),
            })
            .collect();
        let scene_csv = std::fs::read_to_string(dir.join(format!("{name}-scene.csv"))).unwrap();
        let scene: Vec<&str> = scene_csv.lines().skip(1).map(|l| l.split(',').nth(1).unwrap()).collect();
        std::fs::remove_dir_all(&dir).unwrap();
        assert_eq!(wrist, [5, 6], "the newest wrist frame from before the start, then the rest");
        assert_eq!(scene, ["7", "8"], "the same for the scene");
        assert_eq!(ticks, [Timestamp(1005 * MS)], "the tick decided before the start is left out");
    }
}
