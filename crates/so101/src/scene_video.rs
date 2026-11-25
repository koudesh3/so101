//! The scene camera beside a session: saved raw while recording (the camera's JPEGs appended to
//! `<name>-scene.mjpeg`, and a CSV giving each frame's kernel sequence, capture and receive times,
//! and where its bytes are), then encoded to `<name>-scene.mkv` after the recording stops, off the
//! live path. The encode places every frame at its capture time from the CSV, checks the video
//! has them all, and only then deletes the raw file; the CSV stays as the frames' times.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::process::Command;

use so101::camera::Frame;

const CSV_HEADER: &str = "frame,sequence,t_capture_ns,t_receive_ns,offset,length";
// Matroska keeps times to 1 ms.
const MAX_TIME_ERROR_S: f64 = 0.0015;

/// Appends frames while a recording runs. Nothing here waits on another process.
pub struct SceneRaw {
    video: BufWriter<File>,
    times: BufWriter<File>,
    offset: u64,
    frames: u64,
}

/// One frame's row in the CSV.
struct Row {
    t_capture_ns: u64,
    offset: u64,
    length: u64,
}

impl SceneRaw {
    /// Creates `<prefix>-scene.mjpeg` and `<prefix>-scene.csv`; refuses to overwrite either.
    pub fn create(prefix: &Path) -> std::io::Result<Self> {
        let new = |path: PathBuf| File::options().write(true).create_new(true).open(path);
        let video = BufWriter::new(new(with_suffix(prefix, "-scene.mjpeg"))?);
        let mut times = BufWriter::new(new(with_suffix(prefix, "-scene.csv"))?);
        writeln!(times, "{CSV_HEADER}")?;
        Ok(Self { video, times, offset: 0, frames: 0 })
    }

    pub fn add(&mut self, frame: &Frame) -> std::io::Result<()> {
        self.video.write_all(&frame.jpeg)?;
        let length = frame.jpeg.len() as u64;
        writeln!(
            self.times,
            "{},{},{},{},{},{length}",
            self.frames, frame.sequence, frame.t_capture.0, frame.t_receive.0, self.offset
        )?;
        self.offset += length;
        self.frames += 1;
        Ok(())
    }

    /// Flushes both files; returns how many frames were written.
    pub fn finish(mut self) -> std::io::Result<u64> {
        self.video.flush()?;
        self.times.flush()?;
        Ok(self.frames)
    }
}

/// Starts `so101 encode-scenes` on `prefix` as its own low-priority process, so the encode
/// finishes even if teleop exits first.
pub fn encode_in_background(prefix: &Path) -> std::io::Result<()> {
    let exe = std::env::current_exe()?;
    Command::new("nice").arg("-n").arg("19").arg(exe).arg("encode-scenes").arg(prefix).spawn()?;
    Ok(())
}

/// Recording prefixes under `dir` whose scene video is still raw.
pub fn pending(dir: &Path) -> Vec<PathBuf> {
    let Ok(entries) = std::fs::read_dir(dir) else { return Vec::new() };
    let mut out: Vec<PathBuf> = entries
        .flatten()
        .filter_map(|e| {
            let name = e.file_name().into_string().ok()?;
            Some(dir.join(name.strip_suffix("-scene.mjpeg")?))
        })
        .collect();
    out.sort();
    out
}

/// Encodes one recording's raw scene video; returns a one-line summary. On any failure the raw
/// file stays, so it can be run again.
pub fn encode(prefix: &Path) -> Result<String, String> {
    let raw = with_suffix(prefix, "-scene.mjpeg");
    let csv = with_suffix(prefix, "-scene.csv");
    let mkv = with_suffix(prefix, "-scene.mkv");
    let mut rows = read_rows(&csv)?;
    let bytes = std::fs::read(&raw).map_err(|e| format!("{}: {e}", raw.display()))?;
    // A recording cut short (kill, power loss) can end in a frame only partly written.
    rows.retain(|r| r.offset + r.length <= bytes.len() as u64);
    if rows.is_empty() {
        return Err(format!("{}: no complete frames", csv.display()));
    }
    let frames_dir = with_suffix(prefix, "-scene.frames");
    let _ = std::fs::remove_dir_all(&frames_dir);
    std::fs::create_dir(&frames_dir).map_err(|e| format!("{}: {e}", frames_dir.display()))?;
    let result = encode_frames(&rows, &bytes, &frames_dir, &mkv);
    let _ = std::fs::remove_dir_all(&frames_dir);
    let (seconds, frames) = result?;
    std::fs::remove_file(&raw).map_err(|e| format!("{}: {e}", raw.display()))?;
    Ok(format!("{}: {frames} frames, {seconds:.1} s", mkv.display()))
}

// Each frame becomes its own file, listed for ffmpeg's concat input with its duration up to the
// next capture, so the video's timing is the CSV's: uneven intervals and drops included.
// note: each image is read at a 1/25 s time base unless told otherwise, which rounded every frame
// onto a 40 ms grid (2026-11-24); `option framerate 1000` gives 1 ms, Matroska's own resolution.
// Encoded with passthrough timing; ffmpeg's vfr mode dropped frames it judged duplicates.
fn encode_frames(rows: &[Row], bytes: &[u8], dir: &Path, mkv: &Path) -> Result<(f64, usize), String> {
    let mut list = String::from("ffconcat version 1.0\n");
    let mut durations: Vec<u64> = rows.windows(2).map(|w| w[1].t_capture_ns - w[0].t_capture_ns).collect();
    let mut sorted = durations.clone();
    sorted.sort_unstable();
    // The last frame lasts a typical interval.
    durations.push(sorted.get(sorted.len() / 2).copied().unwrap_or(33_333_333));
    for (i, (row, duration)) in rows.iter().zip(&durations).enumerate() {
        let (start, end) = (row.offset as usize, (row.offset + row.length) as usize);
        let jpeg = bytes.get(start..end).ok_or_else(|| format!("frame {i} lies past the raw file's end"))?;
        let name = format!("{i:07}.jpg");
        std::fs::write(dir.join(&name), jpeg).map_err(|e| e.to_string())?;
        list.push_str(&format!("file '{name}'\noption framerate 1000\nduration {:.6}\n", *duration as f64 / 1e9));
    }
    let list_path = dir.join("frames.ffconcat");
    std::fs::write(&list_path, list).map_err(|e| e.to_string())?;
    let partial = mkv.with_extension("mkv.part");
    let status = Command::new("ffmpeg")
        .args(["-nostdin", "-hide_banner", "-loglevel", "error", "-y", "-f", "concat", "-safe", "0", "-i"])
        .arg(&list_path)
        .args([
            "-fps_mode",
            "passthrough",
            "-c:v",
            "libx264",
            "-preset",
            "veryfast",
            "-crf",
            "23",
            "-pix_fmt",
            "yuv420p",
        ])
        .args(["-f", "matroska"])
        .arg(&partial)
        .status()
        .map_err(|e| format!("ffmpeg: {e}"))?;
    if !status.success() {
        return Err(format!("ffmpeg failed ({status}) encoding {}", mkv.display()));
    }
    // Every frame where the CSV puts it, or the raw file stays.
    let got = frame_times(&partial)?;
    let start = rows[0].t_capture_ns;
    let want: Vec<f64> = rows.iter().map(|r| (r.t_capture_ns - start) as f64 / 1e9).collect();
    let worst = got.iter().zip(&want).map(|(g, w)| (g - w).abs()).fold(0.0, f64::max);
    if got.len() != want.len() || worst > MAX_TIME_ERROR_S {
        return Err(format!(
            "{}: {} frames, worst {:.1} ms off; the CSV has {}; raw kept",
            partial.display(),
            got.len(),
            worst * 1e3,
            want.len()
        ));
    }
    std::fs::rename(&partial, mkv).map_err(|e| e.to_string())?;
    Ok((want[want.len() - 1] + durations[durations.len() - 1] as f64 / 1e9, got.len()))
}

// Each frame's presentation time in seconds, in order, as ffprobe reads them.
fn frame_times(video: &Path) -> Result<Vec<f64>, String> {
    let out = Command::new("ffprobe")
        .args(["-v", "error", "-select_streams", "v:0", "-show_entries", "packet=pts_time", "-of", "csv=p=0"])
        .arg(video)
        .output()
        .map_err(|e| format!("ffprobe: {e}"))?;
    let mut times: Vec<f64> = String::from_utf8_lossy(&out.stdout)
        .lines()
        .filter_map(|l| l.trim().trim_end_matches(',').parse().ok())
        .collect();
    times.sort_by(f64::total_cmp);
    Ok(times)
}

fn read_rows(csv: &Path) -> Result<Vec<Row>, String> {
    let text = std::fs::read_to_string(csv).map_err(|e| format!("{}: {e}", csv.display()))?;
    let mut lines: Vec<&str> = text.lines().collect();
    if lines.first() != Some(&CSV_HEADER) {
        return Err(format!("{}: not a scene frame table", csv.display()));
    }
    lines.remove(0);
    // A recording cut short can end mid-row: only a whole last line counts.
    if !text.ends_with('\n') {
        lines.pop();
    }
    lines
        .into_iter()
        .enumerate()
        .map(|(i, line)| {
            let f: Vec<u64> =
                line.split(',').map(|v| v.parse().map_err(|_| format!("row {i}: {line}"))).collect::<Result<_, _>>()?;
            match f.as_slice() {
                [_, _, t_capture_ns, _, offset, length] => {
                    Ok(Row { t_capture_ns: *t_capture_ns, offset: *offset, length: *length })
                }
                _ => Err(format!("row {i}: {line}")),
            }
        })
        .collect()
}

fn with_suffix(prefix: &Path, suffix: &str) -> PathBuf {
    let mut s = prefix.as_os_str().to_owned();
    s.push(suffix);
    PathBuf::from(s)
}
