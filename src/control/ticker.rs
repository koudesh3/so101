//! Fixed-rate scheduling on a grid: tick k starts at start + k * period,
//! not one period after the previous tick's work finished. Late ticks skip forward.

use std::time::{Duration, Instant};

pub struct Ticker {
    period: Duration,
    start: Instant,
    next_index: u64,
    pub overruns: u64,
}

#[derive(Clone, Copy, Debug)]
pub struct Tick {
    pub index: u64,
    pub lateness: Duration,
    // Grid points skipped because the previous work ran long.
    pub skipped: u64,
}

impl Ticker {
    pub fn new(hz: f64) -> Self {
        Self { period: Duration::from_secs_f64(1.0 / hz), start: Instant::now(), next_index: 0, overruns: 0 }
    }

    pub fn period(&self) -> Duration {
        self.period
    }

    pub fn wait(&mut self) -> Tick {
        let now = Instant::now();
        let mut index = self.next_index;
        let mut deadline = self.start + self.period * index as u32;
        let mut skipped = 0;
        while deadline + self.period <= now {
            index += 1;
            skipped += 1;
            deadline = self.start + self.period * index as u32;
        }
        if skipped > 0 {
            self.overruns += 1;
        }
        if deadline > now {
            std::thread::sleep(deadline - now);
        }
        self.next_index = index + 1;
        Tick { index, lateness: Instant::now().saturating_duration_since(deadline), skipped }
    }

    pub fn elapsed(&self) -> Duration {
        self.start.elapsed()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ticks_land_on_the_grid_not_after_the_work() {
        let mut t = Ticker::new(200.0);
        let t0 = Instant::now();
        for _ in 0..20 {
            let tick = t.wait();
            std::thread::sleep(Duration::from_millis(2));
            assert!(tick.lateness < Duration::from_millis(3), "lateness {:?}", tick.lateness);
        }
        let elapsed = t0.elapsed();
        assert!(elapsed < Duration::from_millis(125), "elapsed {elapsed:?}");
        assert_eq!(t.overruns, 0);
    }

    #[test]
    fn overrun_skips_grid_points() {
        let mut t = Ticker::new(100.0);
        let first = t.wait();
        std::thread::sleep(Duration::from_millis(35));
        let second = t.wait();
        assert_eq!(first.index, 0);
        assert!(second.skipped >= 2, "skipped {}", second.skipped);
        assert!(second.index >= 3);
        assert_eq!(t.overruns, 1);
    }
}
