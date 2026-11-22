//! The session header (contract 1) and the end-of-session record, stored as MCAP metadata.

use std::collections::BTreeMap;
use std::time::{SystemTime, UNIX_EPOCH};

use contracts::{MonotonicClock, Timestamp};

use crate::{Error, Result};

pub(crate) const META_SESSION: &str = "so101.session";
pub(crate) const META_SESSION_END: &str = "so101.session_end";
const CONFIG_PREFIX: &str = "config.";
const CALIBRATION_PREFIX: &str = "calibration.";
const CALIBRATION_SUFFIX: &str = ".sha256";
const CAMERA_PREFIX: &str = "camera.";
const CAMERA_DEVICE_SUFFIX: &str = ".device";
const CAMERA_SETTINGS_SUFFIX: &str = ".settings";

/// Who wrote the log, from which commit, on which machine and clock.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Header {
    pub git_commit: String,
    pub git_dirty: bool,
    pub host: String,
    /// What capture and event times mean in this log, e.g. "CLOCK_MONOTONIC" (contract 3).
    pub clock_source: String,
    /// Read together at session start: converts any timestamp in the log to a real date.
    pub start_monotonic: Timestamp,
    pub start_wall_unix_ns: u64,
    /// Flat config values; the full config file can go in as an [`Attachment`].
    pub config: BTreeMap<String, String>,
    /// Calibration name ("leader", "follower") -> SHA-256 (hex) of the file, which is stored as
    /// the attachment `calibration/<name>.json` and checked when the log is read. Degrees mean
    /// something only with the calibration they were computed with.
    pub calibrations: BTreeMap<String, String>,
    /// Camera role ("wrist", "scene") -> which physical device filled it and its settings as
    /// read back at session start. A policy must run at the settings it was trained at, on the
    /// same camera; tying settings to the device shows a swapped config instead of hiding it.
    pub cameras: BTreeMap<String, CameraInfo>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CameraInfo {
    /// The physical device: its USB path or serial, e.g. the `/dev/v4l/by-id/...` name.
    pub device: String,
    /// The controls as read back from the device, e.g. "auto_exposure=1 exposure_time_absolute=300".
    pub settings: String,
}

impl Header {
    /// A header stamped now on this host's monotonic clock. The caller fills in the git fields
    /// and config.
    pub fn now() -> Self {
        let start_monotonic = MonotonicClock::read();
        let start_wall_unix_ns = SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_nanos() as u64).unwrap_or(0);
        Header {
            git_commit: String::new(),
            git_dirty: false,
            host: hostname(),
            clock_source: "CLOCK_MONOTONIC".to_string(),
            start_monotonic,
            start_wall_unix_ns,
            config: BTreeMap::new(),
            calibrations: BTreeMap::new(),
            cameras: BTreeMap::new(),
        }
    }

    /// Record a calibration file: its hash goes in the header, and the returned attachment
    /// (pass it to `Recorder::start`) stores the file itself.
    pub fn add_calibration(&mut self, name: &str, file: &[u8]) -> Attachment {
        self.calibrations.insert(name.to_string(), sha256_hex(file));
        Attachment { name: calibration_attachment(name), media_type: "application/json".into(), data: file.to_vec() }
    }

    /// The wall-clock time (Unix ns) of a monotonic timestamp in this log.
    pub fn wall_time_ns(&self, t: Timestamp) -> i128 {
        self.start_wall_unix_ns as i128 + (t.0 as i128 - self.start_monotonic.0 as i128)
    }

    pub(crate) fn to_metadata(&self) -> BTreeMap<String, String> {
        let mut m = BTreeMap::new();
        m.insert("git_commit".into(), self.git_commit.clone());
        m.insert("git_dirty".into(), self.git_dirty.to_string());
        m.insert("host".into(), self.host.clone());
        m.insert("clock_source".into(), self.clock_source.clone());
        m.insert("start_monotonic_ns".into(), self.start_monotonic.0.to_string());
        m.insert("start_wall_unix_ns".into(), self.start_wall_unix_ns.to_string());
        for (k, v) in &self.config {
            m.insert(format!("{CONFIG_PREFIX}{k}"), v.clone());
        }
        for (k, v) in &self.calibrations {
            m.insert(format!("{CALIBRATION_PREFIX}{k}{CALIBRATION_SUFFIX}"), v.clone());
        }
        for (role, c) in &self.cameras {
            m.insert(format!("{CAMERA_PREFIX}{role}{CAMERA_DEVICE_SUFFIX}"), c.device.clone());
            m.insert(format!("{CAMERA_PREFIX}{role}{CAMERA_SETTINGS_SUFFIX}"), c.settings.clone());
        }
        m
    }

    pub(crate) fn from_metadata(m: &BTreeMap<String, String>) -> Result<Self> {
        let get = |k: &str| m.get(k).cloned().ok_or_else(|| Error::Format(format!("session header lacks {k}")));
        let num = |k: &str| get(k)?.parse::<u64>().map_err(|_| Error::Format(format!("session header: bad {k}")));
        Ok(Header {
            git_commit: get("git_commit")?,
            git_dirty: get("git_dirty")? == "true",
            host: get("host")?,
            clock_source: get("clock_source")?,
            start_monotonic: Timestamp(num("start_monotonic_ns")?),
            start_wall_unix_ns: num("start_wall_unix_ns")?,
            config: m
                .iter()
                .filter_map(|(k, v)| k.strip_prefix(CONFIG_PREFIX).map(|k| (k.to_string(), v.clone())))
                .collect(),
            calibrations: between(m, CALIBRATION_PREFIX, CALIBRATION_SUFFIX),
            cameras: {
                let settings = between(m, CAMERA_PREFIX, CAMERA_SETTINGS_SUFFIX);
                between(m, CAMERA_PREFIX, CAMERA_DEVICE_SUFFIX)
                    .into_iter()
                    .map(|(role, device)| {
                        let settings = settings.get(&role).cloned().unwrap_or_default();
                        (role, CameraInfo { device, settings })
                    })
                    .collect()
            },
        })
    }
}

/// Written when the recorder finishes. Missing if the process was killed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SessionEnd {
    /// False if the recorder overflowed and stopped writing (the disk was effectively dead).
    pub complete: bool,
    pub messages_written: u64,
    pub frames_dropped: u64,
    pub messages_lost: u64,
}

impl SessionEnd {
    pub(crate) fn to_metadata(&self) -> BTreeMap<String, String> {
        BTreeMap::from([
            ("complete".to_string(), self.complete.to_string()),
            ("messages_written".to_string(), self.messages_written.to_string()),
            ("frames_dropped".to_string(), self.frames_dropped.to_string()),
            ("messages_lost".to_string(), self.messages_lost.to_string()),
        ])
    }

    pub(crate) fn from_metadata(m: &BTreeMap<String, String>) -> Result<Self> {
        let num = |k: &str| {
            m.get(k).and_then(|v| v.parse::<u64>().ok()).ok_or_else(|| Error::Format(format!("session end: bad {k}")))
        };
        Ok(SessionEnd {
            complete: m.get("complete").map(String::as_str) == Some("true"),
            messages_written: num("messages_written")?,
            frames_dropped: num("frames_dropped")?,
            messages_lost: num("messages_lost")?,
        })
    }
}

/// A file stored inside the log, such as the full config.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Attachment {
    pub name: String,
    pub media_type: String,
    pub data: Vec<u8>,
}

/// Entries whose key is `<prefix><name><suffix>`, keyed by name.
fn between(m: &BTreeMap<String, String>, prefix: &str, suffix: &str) -> BTreeMap<String, String> {
    m.iter()
        .filter_map(|(k, v)| k.strip_prefix(prefix)?.strip_suffix(suffix).map(|n| (n.to_string(), v.clone())))
        .collect()
}

pub(crate) fn calibration_attachment(name: &str) -> String {
    format!("calibration/{name}.json")
}

pub(crate) fn sha256_hex(bytes: &[u8]) -> String {
    use sha2::{Digest, Sha256};
    Sha256::digest(bytes).iter().map(|b| format!("{b:02x}")).collect()
}

fn hostname() -> String {
    let mut buf = [0u8; 256];
    // SAFETY: the buffer is valid for its length; gethostname NUL-terminates on success.
    let rc = unsafe { libc::gethostname(buf.as_mut_ptr().cast(), buf.len()) };
    if rc != 0 {
        return "unknown".into();
    }
    let end = buf.iter().position(|b| *b == 0).unwrap_or(buf.len());
    String::from_utf8_lossy(&buf[..end]).into_owned()
}
