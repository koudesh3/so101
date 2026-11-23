//! The servo loop's grid (contract 5): cycle k starts at start + k * period, not one period after
//! the previous cycle's work finished. A cycle that would start late is skipped.

use std::time::{Duration, Instant};

pub struct Grid {
    period: Duration,
    start: Instant,
    next_index: u64,
    pub overruns: u64,
}

#[derive(Clone, Copy, Debug)]
pub struct Cycle {
    pub index: u64,
    pub lateness: Duration,
    // Grid points skipped because the previous work ran long.
    pub skipped: u64,
}

impl Grid {
    pub fn new(hz: f64) -> Self {
        Self { period: Duration::from_secs_f64(1.0 / hz), start: Instant::now(), next_index: 0, overruns: 0 }
    }

    pub fn period(&self) -> Duration {
        self.period
    }

    pub fn wait(&mut self) -> Cycle {
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
        Cycle { index, lateness: Instant::now().saturating_duration_since(deadline), skipped }
    }

    pub fn elapsed(&self) -> Duration {
        self.start.elapsed()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Known flaky under heavy parallel load (seen once on 2026-11-22, 15/15 passes alone): it
    // asserts real OS wake-up lateness under 3 ms.
    #[test]
    fn cycles_land_on_the_grid_not_after_the_work() {
        let mut t = Grid::new(200.0);
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
        let mut t = Grid::new(100.0);
        let first = t.wait();
        std::thread::sleep(Duration::from_millis(35));
        let second = t.wait();
        assert_eq!(first.index, 0);
        assert!(second.skipped >= 2, "skipped {}", second.skipped);
        assert!(second.index >= 3);
        assert_eq!(t.overruns, 1);
    }
}
