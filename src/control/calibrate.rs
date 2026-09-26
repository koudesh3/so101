//! The range sweep: poll positions until told to stop, keeping each joint's extremes.

use crate::control::ticker::Ticker;
use crate::error::{Error, Result};
use crate::robot::arm::Arm;
use crate::robot::joint::{Joint, Ticks, COUNT};
use crate::wire::registers::{CENTER_TICK, TICKS_PER_TURN};

pub struct Sweep {
    pub min: Ticks,
    pub max: Ticks,
}

pub fn sweep(arm: &mut Arm, hz: f64, mut keep_going: impl FnMut(&Sweep) -> bool) -> Result<Sweep> {
    let mut sweep = Sweep { min: [i32::MAX; COUNT], max: [i32::MIN; COUNT] };
    let mut ticker = Ticker::new(hz);
    loop {
        ticker.wait();
        // note: a voltage flag no longer fails the read (see Bus::take_flags),
        // so hand-driven sweeps on a marginal 5 V supply keep every sample.
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

// The middle pose should land near the middle of each swept range. A joint whose
// recentred tick (2048) sits near one end, or whose reading wrapped past the
// encoder's zero, was held near a hard stop when Enter was pressed.
pub fn check_centred(sweep: &Sweep) -> Result<()> {
    let mut bad = Vec::new();
    for joint in Joint::ALL {
        if joint == Joint::WristRoll {
            continue; // turns freely; any pose is a middle
        }
        let (lo, hi) = (sweep.min[joint.index()], sweep.max[joint.index()]);
        if hi - lo > TICKS_PER_TURN - 200 {
            bad.push(format!("{} wrapped past the encoder zero ({lo}..{hi})", joint.name()));
            continue;
        }
        let at = (CENTER_TICK - lo) as f64 / (hi - lo).max(1) as f64;
        if !(0.2..=0.8).contains(&at) {
            bad.push(format!("{} held at {:.0}% of its range, not near the middle ({lo}..{hi})", joint.name(), at * 100.0));
        }
    }
    if bad.is_empty() {
        Ok(())
    } else {
        Err(Error::Calibration(format!("redo with a true middle pose: {}", bad.join("; "))))
    }
}
