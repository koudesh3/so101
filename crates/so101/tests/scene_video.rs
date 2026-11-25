//! The scene camera's raw-then-encode path, through the binary: `so101 encode-scenes` turns a raw
//! recording into H.264 with every frame at its CSV time, or leaves the raw file alone.

use std::path::Path;
use std::process::Command;

const JPEG: &[u8] = include_bytes!("../src/camera/synthetic.jpg");

/// Writes `<prefix>-scene.mjpeg` and its CSV for frames captured at `times_ms`; the last frame's
/// bytes are cut short if `crash`.
fn raw_recording(prefix: &Path, times_ms: &[f64], crash: bool) {
    let mut video = Vec::new();
    let mut csv = String::from("frame,sequence,t_capture_ns,t_receive_ns,offset,length\n");
    for (i, t) in times_ms.iter().enumerate() {
        let t_ns = (1_000_000_000.0 + t * 1e6) as u64;
        csv.push_str(&format!("{i},{},{t_ns},{},{},{}\n", i + 40, t_ns + 30_000_000, video.len(), JPEG.len()));
        video.extend_from_slice(JPEG);
    }
    if crash {
        video.truncate(video.len() - 100);
    }
    std::fs::write(format!("{}-scene.mjpeg", prefix.display()), video).unwrap();
    std::fs::write(format!("{}-scene.csv", prefix.display()), csv).unwrap();
}

fn frame_times(video: &Path) -> Vec<f64> {
    let out = Command::new("ffprobe")
        .args(["-v", "error", "-select_streams", "v:0", "-show_entries", "packet=pts_time", "-of", "csv=p=0"])
        .arg(video)
        .output()
        .unwrap();
    let mut t: Vec<f64> =
        String::from_utf8_lossy(&out.stdout).lines().filter_map(|l| l.trim_end_matches(',').parse().ok()).collect();
    t.sort_by(f64::total_cmp);
    t
}

fn encode(path: &Path) -> std::process::Output {
    Command::new(env!("CARGO_BIN_EXE_so101")).arg("encode-scenes").arg(path).output().unwrap()
}

fn have_ffmpeg() -> bool {
    Command::new("ffmpeg").arg("-version").output().is_ok_and(|o| o.status.success())
}

#[test]
fn frames_land_at_their_capture_times_and_a_half_written_one_is_left_out() {
    if !have_ffmpeg() {
        eprintln!("skipped: no ffmpeg");
        return;
    }
    let dir = std::env::temp_dir().join(format!("so101-scene-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let prefix = dir.join("20261124-120000");
    // 29.8 fps, a lost frame (67 ms), uneven intervals, and a last frame cut short by a crash.
    let times = [0.0, 33.6, 67.1, 100.7, 167.8, 201.4, 236.9, 268.5, 302.1];
    raw_recording(&prefix, &times, true);
    let out = encode(&dir);
    assert!(out.status.success(), "{}", String::from_utf8_lossy(&out.stdout));
    let mkv = dir.join("20261124-120000-scene.mkv");
    let got = frame_times(&mkv);
    assert_eq!(got.len(), times.len() - 1, "the half-written frame is left out");
    for (g, want) in got.iter().zip(&times) {
        assert!((g - want / 1e3).abs() <= 0.0015, "frame at {g:.4} s, captured at {:.4} s", want / 1e3);
    }
    assert!(!dir.join("20261124-120000-scene.mjpeg").exists(), "raw removed once the video checks out");
    assert!(dir.join("20261124-120000-scene.csv").exists(), "the CSV stays: it's the frames' times");
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn a_raw_file_that_cant_be_encoded_is_kept() {
    let dir = std::env::temp_dir().join(format!("so101-scene-bad-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let prefix = dir.join("20261124-130000");
    std::fs::write(format!("{}-scene.mjpeg", prefix.display()), b"not a jpeg").unwrap();
    std::fs::write(format!("{}-scene.csv", prefix.display()), "not a frame table\n").unwrap();
    let out = encode(&dir);
    assert!(!out.status.success());
    assert!(dir.join("20261124-130000-scene.mjpeg").exists(), "raw kept");
    std::fs::remove_dir_all(&dir).unwrap();
}
