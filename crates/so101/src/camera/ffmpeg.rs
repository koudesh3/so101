//! `FfmpegCamera`: a V4L2 camera read through ffmpeg until the camera folder gets its own V4L2
//! capture (Phase 4). ffmpeg copies the camera's MJPEG to a pipe; this splits it into frames.

use std::io::Read;
use std::process::{Child, Command, Stdio};
use std::sync::Arc;
use std::thread::JoinHandle;

use bytes::Bytes;

use crate::camera::{Camera, Frame};
use crate::clock::Clock;

/// One camera streaming on its own thread. Dropping it stops ffmpeg.
pub struct FfmpegCamera {
    child: Child,
    thread: Option<JoinHandle<u32>>,
}

impl FfmpegCamera {
    /// Applies `controls` (v4l2-ctl `name=value`) to `device`, then streams it at `size`
    /// ("640x480"), 30 fps, handing each frame to `on_frame` as it arrives. A frame's capture
    /// and receive times are both when it came off the pipe: ffmpeg's pipe drops the kernel's
    /// timestamps, so capture time is a few ms late (exposure starts about 10 ms before that).
    pub fn start(
        camera: Camera,
        device: &str,
        size: &str,
        controls: &[String],
        clock: Arc<dyn Clock>,
        mut on_frame: impl FnMut(Frame) + Send + 'static,
    ) -> std::io::Result<Self> {
        for control in controls {
            let status = Command::new("v4l2-ctl").args(["-d", device, "-c", control]).status()?;
            if !status.success() {
                return Err(std::io::Error::other(format!("v4l2-ctl -d {device} -c {control} failed")));
            }
        }
        let mut child = Command::new("ffmpeg")
            .args(["-nostdin", "-hide_banner", "-loglevel", "error", "-f", "v4l2", "-input_format", "mjpeg"])
            .args(["-video_size", size, "-framerate", "30", "-i", device, "-c:v", "copy", "-f", "mjpeg", "-"])
            .stdout(Stdio::piped())
            .spawn()?;
        let mut out = child.stdout.take().expect("piped");
        let thread = std::thread::Builder::new().name(format!("camera-{}", camera.name())).spawn(move || {
            let (mut buf, mut chunk, mut sequence) = (Vec::with_capacity(1 << 20), vec![0u8; 1 << 16], 0u32);
            while let Ok(n @ 1..) = out.read(&mut chunk) {
                buf.extend_from_slice(&chunk[..n]);
                while let Some(jpeg) = next_jpeg(&mut buf) {
                    let t = clock.now();
                    on_frame(Frame { camera, sequence, t_capture: t, t_receive: t, jpeg: Bytes::from(jpeg) });
                    sequence = sequence.wrapping_add(1);
                }
            }
            sequence
        })?;
        Ok(Self { child, thread: Some(thread) })
    }

    /// Stops ffmpeg; returns how many frames came through.
    pub fn stop(mut self) -> u32 {
        self.halt()
    }

    fn halt(&mut self) -> u32 {
        let _ = self.child.kill();
        let _ = self.child.wait();
        self.thread.take().and_then(|t| t.join().ok()).unwrap_or(0)
    }
}

impl Drop for FfmpegCamera {
    fn drop(&mut self) {
        self.halt();
    }
}

/// Reads a control's value back, e.g. ["white_balance_temperature=3500"], for the session header.
pub fn read_controls(device: &str, names: &[&str]) -> String {
    if names.is_empty() {
        return String::new();
    }
    let out = Command::new("v4l2-ctl").args(["-d", device, "-C", &names.join(",")]).output();
    match out {
        Ok(o) if o.status.success() => String::from_utf8_lossy(&o.stdout)
            .lines()
            .filter_map(|l| l.split_once(':').map(|(k, v)| format!("{}={}", k.trim(), v.trim())))
            .collect::<Vec<_>>()
            .join(" "),
        _ => "unreadable".into(),
    }
}

// Takes the first whole JPEG (FFD8 .. FFD9) out of `buf`, dropping anything before it. UVC
// cameras' MJPEG frames carry no embedded thumbnail, so the first FFD9 ends the frame.
fn next_jpeg(buf: &mut Vec<u8>) -> Option<Vec<u8>> {
    let start = find(buf, &[0xFF, 0xD8], 0)?;
    let Some(end) = find(buf, &[0xFF, 0xD9], start + 2) else {
        buf.drain(..start);
        return None;
    };
    let jpeg = buf[start..end + 2].to_vec();
    buf.drain(..end + 2);
    Some(jpeg)
}

fn find(hay: &[u8], needle: &[u8], from: usize) -> Option<usize> {
    hay.get(from..)?.windows(needle.len()).position(|w| w == needle).map(|p| p + from)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn splits_a_pipe_into_jpegs_across_reads() {
        let mut buf = vec![0x00, 0xFF, 0xD8, 1, 2, 0xFF];
        assert_eq!(next_jpeg(&mut buf), None, "half a frame waits");
        buf.extend_from_slice(&[0xD9, 0xFF, 0xD8, 3]);
        assert_eq!(next_jpeg(&mut buf), Some(vec![0xFF, 0xD8, 1, 2, 0xFF, 0xD9]));
        assert_eq!(next_jpeg(&mut buf), None);
        assert_eq!(buf, vec![0xFF, 0xD8, 3], "the next frame's start is kept");
    }
}
