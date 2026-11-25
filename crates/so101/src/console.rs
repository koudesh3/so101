//! The teleop console: one page served by teleop on the Spark, on 127.0.0.1 only (`just console`
//! tunnels it to the laptop), with both cameras live, the follower's load, and a record button.
//! A recording is the session's MCAP (events and the wrist camera) plus the scene camera beside
//! it as video, with a CSV tying each scene frame to the MCAP's clock.

use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::net::{TcpListener, TcpStream};
use std::path::PathBuf;
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::{Arc, Condvar, Mutex, RwLock};
use std::time::Instant;

use bytes::Bytes;
use so101::arm::{ActuatorStatus, Joint, Limit, JOINT_COUNT};
use so101::camera::Frame;
use so101::clock::Clock;
use so101::control::{Event, Sink};
use so101::session::{Attachment, Header, Recorder, RecorderOptions};

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
    voltage_v: Option<f32>,
    pub held: [Option<Limit>; JOINT_COUNT],
}

impl Live {
    pub fn add(&mut self, a: &ActuatorStatus) {
        let i = a.joint.index();
        self.load_percent[i] = a.load_percent.unwrap_or(self.load_percent[i]);
        self.current_a[i] = a.current_a.unwrap_or(self.current_a[i]);
        self.voltage_v = a.voltage_v.or(self.voltage_v);
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
pub struct Recording {
    setup: RecordingSetup,
    active: Mutex<Option<Active>>,
    sink: RwLock<Option<Arc<dyn Sink>>>,
    last: Mutex<Option<String>>,
}

struct Active {
    name: String,
    started: Instant,
    recorder: Recorder<BufWriter<File>>,
    scene: Option<SceneVideo>,
}

// The scene camera's frames, compressed into an MKV by ffmpeg (H.264, timed by arrival; the
// camera's MJPEG copied straight is about 22 GB an hour at 1280x720), and each frame's receive
// time on the session's clock in a CSV.
struct SceneVideo {
    ffmpeg: Child,
    stdin: Option<ChildStdin>,
    times: BufWriter<File>,
    frames: u64,
}

impl SceneVideo {
    fn start(video: &PathBuf, times: &PathBuf) -> std::io::Result<Self> {
        let mut ffmpeg = Command::new("ffmpeg")
            .args(["-hide_banner", "-loglevel", "error", "-use_wallclock_as_timestamps", "1"])
            // image2pipe stamps each frame when it arrives; the mjpeg demuxer would assume 25 fps.
            .args(["-f", "image2pipe", "-c:v", "mjpeg", "-i", "-", "-fps_mode", "passthrough"])
            .args(["-c:v", "libx264", "-preset", "veryfast", "-crf", "23", "-pix_fmt", "yuv420p"])
            .arg(video)
            .stdin(Stdio::piped())
            .spawn()?;
        let stdin = ffmpeg.stdin.take();
        let mut times = BufWriter::new(File::create(times)?);
        writeln!(times, "frame,sequence,t_receive_ns")?;
        Ok(Self { ffmpeg, stdin, times, frames: 0 })
    }

    fn add(&mut self, frame: &Frame) -> std::io::Result<()> {
        let stdin = self.stdin.as_mut().ok_or_else(|| std::io::Error::other("closed"))?;
        stdin.write_all(&frame.jpeg)?;
        writeln!(self.times, "{},{},{}", self.frames, frame.sequence, frame.t_receive.0)?;
        self.frames += 1;
        Ok(())
    }

    fn finish(mut self) -> std::io::Result<u64> {
        drop(self.stdin.take());
        self.ffmpeg.wait()?;
        self.times.flush()?;
        Ok(self.frames)
    }
}

impl Recording {
    pub fn new(setup: RecordingSetup) -> Self {
        Self { setup, active: Mutex::new(None), sink: RwLock::new(None), last: Mutex::new(None) }
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
        let scene = match self.setup.scene_video {
            true => Some(SceneVideo::start(&path("-scene.mkv"), &path("-scene.csv")).map_err(|e| e.to_string())?),
            false => None,
        };
        *self.sink.write().expect("sink") = Some(recorder.sink());
        println!("recording {}", path(".mcap").display());
        *active = Some(Active { name: name.clone(), started: Instant::now(), recorder, scene });
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
        let scene_frames = a.scene.map(SceneVideo::finish).transpose().map_err(|e| e.to_string())?.unwrap_or(0);
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
        if let Some(scene) = self.active.lock().expect("recording").as_mut().and_then(|a| a.scene.as_mut()) {
            if let Err(e) = scene.add(frame) {
                eprintln!("scene video: {e}");
            }
        }
    }

    fn json(&self) -> serde_json::Value {
        let active = self.active.lock().expect("recording");
        serde_json::json!({
            "active": active.is_some(),
            "name": active.as_ref().map(|a| a.name.clone()),
            "seconds": active.as_ref().map(|a| a.started.elapsed().as_secs()),
            "last": *self.last.lock().expect("last"),
        })
    }
}

impl Sink for Recording {
    fn accept(&self, event: &Event) {
        if let Some(sink) = self.sink.read().expect("sink").as_ref() {
            sink.accept(event);
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
            "voltage_v": live.voltage_v.map(|v| round(v, 10.0)),
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
