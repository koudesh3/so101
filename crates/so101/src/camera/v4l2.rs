//! `V4l2Camera`: a Linux camera read through the kernel's buffers, so each frame carries the
//! kernel's timestamp and frame number (contract 3). Applies its controls by v4l2-ctl's names
//! (as in robot.conf) and reads them back for the session header (contract 7).

use std::sync::Arc;
use std::time::Duration;

use bytes::Bytes;
use v4l::buffer::{Flags, Type};
use v4l::control::{Control, Description as ControlDescription, Type as ControlType, Value};
use v4l::io::mmap::Stream;
use v4l::io::traits::CaptureStream;
use v4l::video::capture::Parameters;
use v4l::video::Capture;
use v4l::{Device, Format, FourCC};

use crate::camera::{Camera, CameraRole, Description, Error, Frame, Result};
use crate::clock::{Clock, Timestamp};

const BUFFERS: u32 = 4;
// How long next_frame waits before returning `Timeout`, so a stopping thread notices in time.
const WAIT: Duration = Duration::from_millis(200);

pub struct V4l2Camera {
    role: CameraRole,
    path: String,
    period: Duration,
    settings: String,
    // Dropping the stream stops it; it goes before the device.
    stream: Stream<'static>,
    _device: Device,
    clock: Arc<dyn Clock>,
}

impl V4l2Camera {
    /// Opens `path` for MJPEG at `width`x`height`, 30 fps, applies `controls` ("name=value", by
    /// v4l2-ctl's names) and reads them back; errors if one didn't take. `clock` must read
    /// CLOCK_MONOTONIC (`MonotonicClock`), the kernel stamps' clock.
    pub fn open(
        role: CameraRole,
        path: &str,
        (width, height): (u32, u32),
        controls: &[String],
        clock: Arc<dyn Clock>,
    ) -> Result<Self> {
        let device = Device::with_path(path).map_err(|e| Error::Device(format!("{path}: {e}")))?;
        let mjpg = FourCC::new(b"MJPG");
        let format = device.set_format(&Format::new(width, height, mjpg))?;
        if format.fourcc != mjpg || (format.width, format.height) != (width, height) {
            return Err(Error::Device(format!(
                "{path} gave {}x{} {} for MJPG {width}x{height}",
                format.width, format.height, format.fourcc
            )));
        }
        let params = device.set_params(&Parameters::with_fps(30))?;
        let interval = params.interval;
        if interval.numerator == 0 || interval.denominator == 0 {
            return Err(Error::Device(format!("{path} reported a frame interval of {interval:?}")));
        }
        let period = Duration::from_secs_f64(f64::from(interval.numerator) / f64::from(interval.denominator));
        let settings = apply_controls(&device, path, controls)?;
        let mut stream = Stream::with_buffers(&device, Type::VideoCapture, BUFFERS)?;
        stream.set_timeout(WAIT);
        Ok(Self { role, path: path.to_string(), period, settings, stream, _device: device, clock })
    }
}

impl Camera for V4l2Camera {
    fn role(&self) -> CameraRole {
        self.role
    }

    fn nominal_period(&self) -> Duration {
        self.period
    }

    fn describe(&self) -> Description {
        Description { device: self.path.clone(), settings: self.settings.clone() }
    }

    fn next_frame(&mut self) -> Result<Frame> {
        loop {
            let (buffer, meta) = match self.stream.next() {
                Ok(next) => next,
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => return Err(Error::Timeout),
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
                Err(e) => return Err(Error::Io(e)),
            };
            let t_receive = self.clock.now();
            if meta.flags & Flags::TIMESTAMP_MASK != Flags::TIMESTAMP_MONOTONIC {
                return Err(Error::Device(format!("{}: kernel timestamps aren't CLOCK_MONOTONIC", self.path)));
            }
            let jpeg = &buffer[..(meta.bytesused as usize).min(buffer.len())];
            // A buffer the driver marks corrupt, or one that isn't a JPEG, is skipped; its
            // sequence number still counts, so it shows as a gap.
            if meta.flags.contains(Flags::ERROR) || !jpeg.starts_with(&[0xFF, 0xD8]) {
                continue;
            }
            let t_capture = Timestamp(meta.timestamp.sec as u64 * 1_000_000_000 + meta.timestamp.usec as u64 * 1_000);
            return Ok(Frame {
                camera: self.role,
                sequence: meta.sequence,
                t_capture,
                t_receive,
                jpeg: Bytes::copy_from_slice(jpeg),
            });
        }
    }
}

// Applies "name=value" controls, auto switches first (uvcvideo refuses a manual value while its
// auto mode is on), then reads every named control back. Returns "name=value ..." as read.
fn apply_controls(device: &Device, path: &str, controls: &[String]) -> Result<String> {
    let available: Vec<ControlDescription> = device.query_controls()?;
    let find = |name: &str| {
        available
            .iter()
            .find(|c| control_name(&c.name) == name)
            .ok_or_else(|| Error::Device(format!("{path} has no control {name}")))
    };
    let mut wanted = Vec::new();
    for entry in controls {
        let (name, value) =
            entry.split_once('=').ok_or_else(|| Error::Device(format!("camera control '{entry}' isn't name=value")))?;
        let value: i64 = value.parse().map_err(|_| Error::Device(format!("camera control '{entry}': not a number")))?;
        wanted.push((name.to_string(), value, find(name)?));
    }
    wanted.sort_by_key(|(name, _, _)| !is_auto_switch(name));
    for (name, value, description) in &wanted {
        let value = match description.typ {
            ControlType::Boolean => Value::Boolean(*value != 0),
            _ => Value::Integer(*value),
        };
        device
            .set_control(Control { id: description.id, value })
            .map_err(|e| Error::Device(format!("{path} {name}: {e}")))?;
    }
    let mut read = Vec::new();
    let mut wrong = Vec::new();
    for (name, want, description) in &wanted {
        let got = match device.control(description.id)?.value {
            Value::Integer(v) => v,
            Value::Boolean(b) => i64::from(b),
            other => return Err(Error::Device(format!("{path} {name} reads {other:?}"))),
        };
        if got != *want {
            wrong.push(format!("{name} reads {got}, set {want}"));
        }
        read.push(format!("{name}={got}"));
    }
    if !wrong.is_empty() {
        return Err(Error::Device(format!("{path}: controls didn't take: {}", wrong.join("; "))));
    }
    Ok(read.join(" "))
}

fn is_auto_switch(name: &str) -> bool {
    name.starts_with("auto_") || name.ends_with("_automatic") || name.ends_with("_auto")
}

/// The kernel's control name as v4l2-ctl spells it: lowercase, every run of other characters
/// one `_`, none at the ends ("Exposure Time, Absolute" -> "exposure_time_absolute").
pub fn control_name(kernel_name: &str) -> String {
    let mut out = String::new();
    for c in kernel_name.chars() {
        if c.is_ascii_alphanumeric() {
            out.push(c.to_ascii_lowercase());
        } else if !out.is_empty() && !out.ends_with('_') {
            out.push('_');
        }
    }
    out.trim_end_matches('_').to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn control_names_match_v4l2_ctl() {
        assert_eq!(control_name("Exposure Time, Absolute"), "exposure_time_absolute");
        assert_eq!(control_name("White Balance, Automatic"), "white_balance_automatic");
        assert_eq!(control_name("White Balance Temperature"), "white_balance_temperature");
        assert_eq!(control_name("Auto Exposure"), "auto_exposure");
        assert_eq!(control_name("Gain"), "gain");
        assert_eq!(control_name("Power Line Frequency"), "power_line_frequency");
    }

    #[test]
    fn auto_switches_are_set_first() {
        assert!(is_auto_switch("auto_exposure") && is_auto_switch("white_balance_automatic"));
        assert!(!is_auto_switch("exposure_time_absolute") && !is_auto_switch("gain"));
    }
}
