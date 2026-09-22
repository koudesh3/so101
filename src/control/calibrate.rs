//! The range sweep: poll positions until told to stop, keeping each joint's extremes.

use crate::robot::arm::Arm;
use crate::error::Result;
use crate::robot::joint::{Ticks, COUNT};
use crate::control::ticker::Ticker;

pub struct Sweep {
    pub min: Ticks,
    pub max: Ticks,
}

pub fn sweep(arm: &mut Arm, hz: f64, mut keep_going: impl FnMut(&Sweep) -> bool) -> Result<Sweep> {
    let mut sweep = Sweep { min: [i32::MAX; COUNT], max: [i32::MIN; COUNT] };
    let mut ticker = Ticker::new(hz);
    loop {
        ticker.wait();
        let ticks = arm.read_ticks()?;
        for i in 0..COUNT {
            sweep.min[i] = sweep.min[i].min(ticks[i]);
            sweep.max[i] = sweep.max[i].max(ticks[i]);
        }
        if !keep_going(&sweep) {
            return Ok(sweep);
        }
    }
}
