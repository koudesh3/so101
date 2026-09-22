//! 1:1 mirroring. Each tick: read both arms, map leader values onto the
//! follower's calibration, limit the step, write the follower's goals.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::robot::arm::Arm;
use crate::error::Result;
use crate::robot::joint::{Ticks, Values, COUNT};
use crate::control::ticker::{Tick, Ticker};

#[derive(Clone, Copy, Debug)]
pub struct TeleopOptions {
    pub hz: f64,
    // Largest goal change per tick, in ticks. None disables the limit.
    pub max_step_ticks: Option<i32>,
    // Stop after this many ticks. None runs until `stop` is set.
    pub max_ticks: Option<u64>,
}

#[derive(Clone, Debug)]
pub struct TickReport {
    pub tick: Tick,
    pub leader_ticks: Ticks,
    pub leader_values: Values,
    pub follower_ticks: Ticks,
    pub follower_values: Values,
    pub goal_ticks_sent: Ticks,
    pub work_time: Duration,
    // Set when this tick failed on the bus. The loop continues; the previous goal holds.
    pub error: Option<String>,
}

#[derive(Debug, Default, Clone)]
pub struct TeleopStats {
    pub ticks: u64,
    pub overruns: u64,
    pub errors: u64,
    pub worst_work_time: Duration,
}

// Runs until `stop` is set or `max_ticks` is reached. Follower torque is
// switched off on every exit path.
pub fn run(
    leader: &mut Arm,
    follower: &mut Arm,
    options: TeleopOptions,
    stop: Arc<AtomicBool>,
    mut on_tick: impl FnMut(&TickReport),
) -> Result<TeleopStats> {
    leader.torque_off()?;
    follower.torque_on_holding()?;

    let loop_result = run_loop(leader, follower, options, &stop, &mut on_tick);
    let torque_result = follower.torque_off();

    match (loop_result, torque_result) {
        (Err(e), _) => Err(e),
        (Ok(_), Err(e)) => Err(e),
        (Ok(stats), Ok(())) => Ok(stats),
    }
}

fn run_loop(
    leader: &mut Arm,
    follower: &mut Arm,
    options: TeleopOptions,
    stop: &AtomicBool,
    on_tick: &mut impl FnMut(&TickReport),
) -> Result<TeleopStats> {
    let mut ticker = Ticker::new(options.hz);
    let mut stats = TeleopStats::default();
    let mut last_follower_ticks: Option<Ticks> = None;

    while !stop.load(Ordering::Relaxed) {
        if options.max_ticks.is_some_and(|max| stats.ticks >= max) {
            break;
        }
        let tick = ticker.wait();
        let work_started = Instant::now();
        let outcome = one_tick(leader, follower, options, last_follower_ticks);
        let work_time = work_started.elapsed();
        stats.ticks += 1;
        stats.worst_work_time = stats.worst_work_time.max(work_time);

        let report = match outcome {
            Ok(mut r) => {
                last_follower_ticks = Some(r.follower_ticks);
                r.tick = tick;
                r.work_time = work_time;
                r
            }
            Err(e) => {
                stats.errors += 1;
                TickReport {
                    tick,
                    leader_ticks: [0; COUNT],
                    leader_values: [0.0; COUNT],
                    follower_ticks: last_follower_ticks.unwrap_or([0; COUNT]),
                    follower_values: [0.0; COUNT],
                    goal_ticks_sent: [0; COUNT],
                    work_time,
                    error: Some(e.to_string()),
                }
            }
        };
        on_tick(&report);
    }
    stats.overruns = ticker.overruns;
    Ok(stats)
}

fn one_tick(leader: &mut Arm, follower: &mut Arm, options: TeleopOptions, last_follower_ticks: Option<Ticks>) -> Result<TickReport> {
    let follower_ticks = follower.read_ticks()?;
    let leader_ticks = leader.read_ticks()?;

    let leader_values = leader.ticks_to_values(&leader_ticks);
    let follower_values = follower.ticks_to_values(&follower_ticks);

    let goal_ticks = follower.values_to_ticks(&leader_values);
    let reference = Some(follower_ticks).or(last_follower_ticks);
    let goal_ticks_sent = follower.write_goal_ticks(goal_ticks, reference, options.max_step_ticks)?;

    Ok(TickReport {
        tick: Tick { index: 0, lateness: Duration::ZERO, skipped: 0 },
        leader_ticks,
        leader_values,
        follower_ticks,
        follower_values,
        goal_ticks_sent,
        work_time: Duration::ZERO,
        error: None,
    })
}
