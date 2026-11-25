//! The teleop console: runs on the laptop and serves one page on localhost with both cameras, the
//! follower's per-joint load, and the fine-mode switch. Cameras come over SSH as the MJPEG the
//! cameras send (ffmpeg on the Spark, copied, not re-encoded); the switch goes over an SSH tunnel
//! to teleop's control port (`so101 teleop --control-port`). std only.

use std::io::{BufRead, BufReader, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::process::{Command, Stdio};
use std::sync::{Arc, Condvar, Mutex};
use std::thread;
use std::time::Duration;

const PAGE: &str = include_str!("page.html");
const WRIST: &str = "/dev/v4l/by-id/usb-icSpring_icspring_camera_20240307110322-video-index0";
const SCENE: &str = "/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-1080p-S1_SN0001-video-index0";

// Applies the camera's v4l2 controls from /projects/robot.conf (its exposure is manual), then
// streams it. Run by `bash -s DEVICE SIZE` on the Spark.
const CAMERA_SCRIPT: &str = r#"
. /projects/robot.conf
for entry in "${CAMERA_CONTROLS[@]}"; do
  set -- $entry
  if [ "$1" = "$DEVICE" ]; then shift; for c in "$@"; do v4l2-ctl -d "$DEVICE" -c "$c"; done; fi
done
exec ffmpeg -nostdin -hide_banner -loglevel error -f v4l2 -input_format mjpeg -video_size "$SIZE" -framerate 30 -i "$DEVICE" -c:v copy -f mjpeg -
"#;

struct Options {
    host: String,
    port: u16,
    control_port: u16,
}

/// The newest JPEG from one camera, and a count that says when it changed.
#[derive(Default)]
struct Feed {
    frame: Mutex<(u64, Arc<Vec<u8>>)>,
    fresh: Condvar,
}

fn main() {
    let options = parse_args();
    let wrist = Arc::new(Feed::default());
    let scene = Arc::new(Feed::default());
    for (feed, device, size) in [(&wrist, WRIST, "640x480"), (&scene, SCENE, "1280x720")] {
        let (feed, host) = (feed.clone(), options.host.clone());
        thread::spawn(move || keep_streaming(&host, device, size, &feed));
    }
    {
        let (host, port) = (options.host.clone(), options.control_port);
        thread::spawn(move || keep_tunnel(&host, port));
    }
    let listener = TcpListener::bind(("127.0.0.1", options.port)).unwrap_or_else(|e| {
        eprintln!("cannot listen on 127.0.0.1:{}: {e}", options.port);
        std::process::exit(1)
    });
    eprintln!("console on http://localhost:{} (cameras and teleop on {})", options.port, options.host);
    let control = Arc::new(Mutex::new(None::<BufReader<TcpStream>>));
    for conn in listener.incoming().flatten() {
        let (wrist, scene, control) = (wrist.clone(), scene.clone(), control.clone());
        let control_port = options.control_port;
        thread::spawn(move || {
            let _ = serve(conn, &wrist, &scene, &control, control_port);
        });
    }
}

fn parse_args() -> Options {
    let mut options = Options { host: "spark".into(), port: 8090, control_port: 8091 };
    let mut args = std::env::args().skip(1);
    while let Some(flag) = args.next() {
        let value = args.next();
        match (flag.as_str(), value) {
            ("--host", Some(v)) => options.host = v,
            ("--port", Some(v)) => options.port = v.parse().expect("--port: a number"),
            ("--control-port", Some(v)) => options.control_port = v.parse().expect("--control-port: a number"),
            _ => {
                eprintln!("usage: console [--host spark] [--port 8090] [--control-port 8091]");
                std::process::exit(2)
            }
        }
    }
    options
}

/// Streams one camera, restarting a second after it stops (unplugged, or busy with a recording).
fn keep_streaming(host: &str, device: &str, size: &str, feed: &Feed) {
    loop {
        match stream_camera(host, device, size, feed) {
            Ok(frames) => eprintln!("{device}: stream ended after {frames} frames; retrying"),
            Err(e) => eprintln!("{device}: {e}; retrying"),
        }
        thread::sleep(Duration::from_secs(1));
    }
}

fn stream_camera(host: &str, device: &str, size: &str, feed: &Feed) -> std::io::Result<u64> {
    let mut child = Command::new("ssh")
        .args(["-T", "-o", "ServerAliveInterval=5", host, &format!("DEVICE={device} SIZE={size} bash -s")])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()?;
    child.stdin.take().expect("piped").write_all(CAMERA_SCRIPT.as_bytes())?;
    let mut out = child.stdout.take().expect("piped");
    let (mut buf, mut chunk, mut frames) = (Vec::with_capacity(1 << 20), vec![0u8; 1 << 16], 0);
    loop {
        let n = out.read(&mut chunk)?;
        if n == 0 {
            break;
        }
        buf.extend_from_slice(&chunk[..n]);
        // Each JPEG runs from FFD8 to FFD9; UVC cameras' MJPEG frames carry no embedded thumbnail.
        while let Some(start) = find(&buf, &[0xFF, 0xD8], 0) {
            let Some(end) = find(&buf, &[0xFF, 0xD9], start + 2) else {
                buf.drain(..start);
                break;
            };
            let jpeg = Arc::new(buf[start..end + 2].to_vec());
            buf.drain(..end + 2);
            let mut frame = feed.frame.lock().expect("feed");
            *frame = (frame.0 + 1, jpeg);
            feed.fresh.notify_all();
            frames += 1;
        }
    }
    let _ = child.wait();
    Ok(frames)
}

fn find(hay: &[u8], needle: &[u8], from: usize) -> Option<usize> {
    hay.get(from..)?.windows(needle.len()).position(|w| w == needle).map(|p| p + from)
}

/// Forwards localhost:PORT to the Spark's 127.0.0.1:PORT, where teleop listens; restarts if it drops.
fn keep_tunnel(host: &str, port: u16) {
    loop {
        let status = Command::new("ssh")
            .args(["-N", "-o", "ExitOnForwardFailure=yes", "-o", "ServerAliveInterval=5"])
            .args(["-L", &format!("127.0.0.1:{port}:127.0.0.1:{port}"), host])
            .status();
        eprintln!("control tunnel ended ({status:?}); retrying");
        thread::sleep(Duration::from_secs(2));
    }
}

/// One command to teleop's control port; the answer is its JSON status line.
fn ask_teleop(control: &Mutex<Option<BufReader<TcpStream>>>, port: u16, command: &str) -> Option<String> {
    let mut conn = control.lock().expect("control");
    for _ in 0..2 {
        if conn.is_none() {
            let stream = TcpStream::connect_timeout(&([127, 0, 0, 1], port).into(), Duration::from_millis(300)).ok()?;
            stream.set_read_timeout(Some(Duration::from_millis(500))).ok()?;
            *conn = Some(BufReader::new(stream));
        }
        let reader = conn.as_mut().expect("connected");
        let mut line = String::new();
        let ok = writeln!(reader.get_mut(), "{command}").is_ok() && reader.read_line(&mut line).is_ok_and(|n| n > 0);
        if ok {
            return Some(line.trim().to_string());
        }
        *conn = None;
    }
    None
}

fn serve(
    conn: TcpStream,
    wrist: &Feed,
    scene: &Feed,
    control: &Mutex<Option<BufReader<TcpStream>>>,
    control_port: u16,
) -> std::io::Result<()> {
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
    let path = request.split_whitespace().nth(1).unwrap_or("/");
    match path {
        "/" => respond(&mut out, "text/html; charset=utf-8", PAGE.as_bytes()),
        "/cam/wrist" => stream_to(&mut out, wrist),
        "/cam/scene" => stream_to(&mut out, scene),
        _ if path.starts_with("/api/") => {
            // /api/fine/toggle -> "fine toggle"; /api/status -> "status"
            let command = path["/api/".len()..].replace('/', " ");
            let body = ask_teleop(control, control_port, &command)
                .unwrap_or_else(|| r#"{"error":"teleop isn't running (no control port)"}"#.into());
            respond(&mut out, "application/json", body.as_bytes())
        }
        _ => write!(out, "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"),
    }
}

fn respond(out: &mut TcpStream, kind: &str, body: &[u8]) -> std::io::Result<()> {
    write!(out, "HTTP/1.1 200 OK\r\nContent-Type: {kind}\r\nContent-Length: {}\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n", body.len())?;
    out.write_all(body)
}

fn stream_to(out: &mut TcpStream, feed: &Feed) -> std::io::Result<()> {
    write!(out, "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n")?;
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
