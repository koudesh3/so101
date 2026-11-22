//! One time base for everything (contract 3): nanoseconds of the host's `CLOCK_MONOTONIC`.

use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

/// Nanoseconds of the host's monotonic clock. Never jumps; compares only within one log.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Timestamp(pub u64);

impl Timestamp {
    pub const fn from_nanos(ns: u64) -> Self {
        Self(ns)
    }

    pub const fn nanos(self) -> u64 {
        self.0
    }

    /// Time from `earlier` to `self`, or zero if `earlier` is later.
    pub fn since(self, earlier: Timestamp) -> Duration {
        Duration::from_nanos(self.0.saturating_sub(earlier.0))
    }
}

impl std::ops::Add<Duration> for Timestamp {
    type Output = Timestamp;
    /// Saturating: a timestamp never wraps.
    fn add(self, d: Duration) -> Timestamp {
        Timestamp(self.0.saturating_add(d.as_nanos() as u64))
    }
}

/// Where "now" comes from. Real runs use [`MonotonicClock`]; replays use [`ReplayClock`].
pub trait Clock: Send + Sync {
    fn now(&self) -> Timestamp;
    /// Block until `now() >= t`. Returns at once if `t` has passed.
    fn sleep_until(&self, t: Timestamp);
}

/// The host's `CLOCK_MONOTONIC`, read directly: Rust's `Instant` can't give the raw value, and
/// the raw value is what the kernel stamps camera frames with and Python's `time.monotonic()`
/// reads.
#[derive(Clone, Copy, Debug, Default)]
pub struct MonotonicClock;

impl MonotonicClock {
    pub fn read() -> Timestamp {
        let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
        // SAFETY: `ts` is a valid, writable timespec; CLOCK_MONOTONIC exists on Linux and macOS.
        let rc = unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
        assert_eq!(rc, 0, "clock_gettime(CLOCK_MONOTONIC) failed");
        Timestamp(ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64)
    }
}

impl Clock for MonotonicClock {
    fn now(&self) -> Timestamp {
        Self::read()
    }

    fn sleep_until(&self, t: Timestamp) {
        loop {
            let now = Self::read();
            if now >= t {
                return;
            }
            std::thread::sleep(t.since(now));
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ReplaySpeed {
    /// Real time multiplied by this factor (1.0 is the original speed).
    Factor(f64),
    /// No waiting: `sleep_until` jumps straight to its target.
    AsFastAsPossible,
}

/// Replays a log's timeline: `now()` reports log time, `sleep_until` waits (scaled) real time.
pub struct ReplayClock {
    log_start: Timestamp,
    real_start: Instant,
    speed: ReplaySpeed,
    current: AtomicU64,
}

impl ReplayClock {
    /// A clock whose time starts at `log_start` (usually the first event's time).
    pub fn new(log_start: Timestamp, speed: ReplaySpeed) -> Self {
        if let ReplaySpeed::Factor(f) = speed {
            assert!(f > 0.0, "replay speed must be positive");
        }
        Self { log_start, real_start: Instant::now(), speed, current: AtomicU64::new(log_start.0) }
    }
}

impl Clock for ReplayClock {
    fn now(&self) -> Timestamp {
        match self.speed {
            ReplaySpeed::Factor(f) => self.log_start + self.real_start.elapsed().mul_f64(f),
            ReplaySpeed::AsFastAsPossible => Timestamp(self.current.load(Ordering::SeqCst)),
        }
    }

    fn sleep_until(&self, t: Timestamp) {
        match self.speed {
            ReplaySpeed::Factor(f) => {
                let real_target = self.real_start + t.since(self.log_start).div_f64(f);
                let now = Instant::now();
                if real_target > now {
                    std::thread::sleep(real_target - now);
                }
            }
            ReplaySpeed::AsFastAsPossible => {
                self.current.fetch_max(t.0, Ordering::SeqCst);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn monotonic_clock_advances_and_sleeps() {
        let c = MonotonicClock;
        let a = c.now();
        c.sleep_until(a + Duration::from_millis(5));
        let b = c.now();
        assert!(b.since(a) >= Duration::from_millis(5), "slept {:?}", b.since(a));
        assert!(b.since(a) < Duration::from_millis(50), "slept {:?}", b.since(a));
    }

    #[test]
    fn replay_as_fast_as_possible_reports_log_time() {
        let c = ReplayClock::new(Timestamp(1_000), ReplaySpeed::AsFastAsPossible);
        assert_eq!(c.now(), Timestamp(1_000));
        c.sleep_until(Timestamp(5_000));
        assert_eq!(c.now(), Timestamp(5_000));
        c.sleep_until(Timestamp(2_000)); // the past: no change
        assert_eq!(c.now(), Timestamp(5_000));
    }
}
