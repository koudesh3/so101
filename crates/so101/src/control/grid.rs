//! The servo loop's grid (contract 5): cycle k starts at start + k * period, not one period after
//! the previous cycle's work finished. A cycle that would start late is skipped.

use std::sync::Arc;
use std::time::Duration;

use crate::clock::{Clock, Timestamp};

pub struct Grid {
    clock: Arc<dyn Clock>,
    period: Duration,
    start: Timestamp,
    next_index: u64,
    pub overruns: u64,
}

#[derive(Clone, Copy, Debug)]
pub struct Cycle {
    pub index: u64,
    pub lateness: Duration,
    /// Grid points skipped because the previous work ran long.
    pub skipped: u64,
}

impl Grid {
    pub fn new(hz: f64, clock: Arc<dyn Clock>) -> Self {
        let start = clock.now();
        Self { clock, period: Duration::from_secs_f64(1.0 / hz), start, next_index: 0, overruns: 0 }
    }

    pub fn period(&self) -> Duration {
        self.period
    }

    pub fn wait(&mut self) -> Cycle {
        let now = self.clock.now();
        let at = |index: u64| self.start + self.period * index as u32;
        let mut index = self.next_index;
        let mut skipped = 0;
        while at(index) + self.period <= now {
            index += 1;
            skipped += 1;
        }
        if skipped > 0 {
            self.overruns += 1;
        }
        let deadline = at(index);
        self.clock.sleep_until(deadline);
        self.next_index = index + 1;
        Cycle { index, lateness: self.clock.now().since(deadline), skipped }
    }

    pub fn elapsed(&self) -> Duration {
        self.clock.now().since(self.start)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clock::{ReplayClock, ReplaySpeed};

    fn stepped() -> Arc<ReplayClock> {
        Arc::new(ReplayClock::new(Timestamp(0), ReplaySpeed::AsFastAsPossible))
    }

    #[test]
    fn cycles_land_on_the_grid_not_after_the_work() {
        let clock = stepped();
        let mut grid = Grid::new(200.0, clock.clone());
        for k in 0..20 {
            let cycle = grid.wait();
            assert_eq!(cycle.index, k);
            assert_eq!(cycle.lateness, Duration::ZERO);
            assert_eq!(clock.now(), Timestamp(k * 5_000_000), "cycle {k} starts on the grid");
            clock.sleep_until(clock.now() + Duration::from_millis(2)); // the work
        }
        assert_eq!(grid.overruns, 0);
    }

    #[test]
    fn overrun_skips_grid_points() {
        let clock = stepped();
        let mut grid = Grid::new(100.0, clock.clone());
        let first = grid.wait();
        clock.sleep_until(clock.now() + Duration::from_millis(35)); // work that overruns 3 cycles
        let second = grid.wait();
        assert_eq!(first.index, 0);
        assert_eq!(second.skipped, 2);
        assert_eq!(second.index, 3);
        assert_eq!(grid.overruns, 1);
    }
}
