//! 1:1 mirroring. Each tick: read both arms, map leader values onto the
//! follower's calibration, limit the step, write the follower's goals.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::robot::arm::{Arm, Telemetry};
use crate::wire::protocol::ServoError;
use crate::robot::joint::Joint;
use crate::error::{Error, Result};
use crate::robot::joint::{Ticks, Values, COUNT};
use crate::control::ticker::{Tick, Ticker};

#[derive(Clone, Copy, Debug)]
pub struct TeleopOptions {
    pub hz: f64,
    // Largest goal change per tick, in ticks. None disables the limit.
    pub max_step_ticks: Option<i32>,
    // Stop after this many ticks. None runs until `stop` is set.
    pub max_ticks: Option<u64>,
    // Also read one follower servo's full telemetry block per tick, rotating
    // through the six. Costs one extra round trip per tick.
    pub telemetry: bool,
    // Per joint: command where the leader will be this far ahead, from its
    // measured velocity, to cancel the follower's own response lag. 0 disables.
    pub lead: [Duration; COUNT],
}

#[derive(Clone, Debug)]
pub struct TickReport {
    pub tick: Tick,
    pub leader_ticks: Ticks,
    pub leader_values: Values,
    pub follower_ticks: Ticks,
    pub follower_values: Values,
    pub goal_ticks_sent: Ticks,
    // The goal actually sent, back in the shared coordinate. Differs from
    // `leader_values` where the range clamp or the step limit bit.
    pub goal_values_sent: Values,
    // Advisory status bits the follower raised this tick, per joint.
    pub flags: [ServoError; COUNT],
    // Joints being held at their stall point after an overload trip.
    pub released: [bool; COUNT],
    // One follower servo's telemetry, when `TeleopOptions::telemetry` is on.
    pub telemetry: Option<Telemetry>,
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
    // Status bits raised before this run (a trip at the end of the last one,
    // say) describe nothing the loop did. Drop them so they cannot latch a hold.
    let _ = follower.take_flags();
    let _ = leader.take_flags();

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
    let mut last_goal: Option<Ticks> = None;
    let mut releases: [Option<Release>; COUNT] = [None; COUNT];
    let mut stalls = StallWatch::default();
    let mut predictor = Predictor::default();
    let mut consecutive_errors = 0u32;
    let fail_after = ((FAILSAFE_AFTER.as_secs_f64() * options.hz).ceil() as u32).max(3);

    while !stop.load(Ordering::Relaxed) {
        if options.max_ticks.is_some_and(|max| stats.ticks >= max) {
            break;
        }
        let tick = ticker.wait();
        let work_started = Instant::now();
        let outcome = one_tick(leader, follower, options, last_goal, &mut releases, &mut stalls, &mut predictor, stats.ticks);
        let work_time = work_started.elapsed();
        stats.ticks += 1;
        stats.worst_work_time = stats.worst_work_time.max(work_time);

        let report = match outcome {
            Ok(mut r) => {
                consecutive_errors = 0;
                last_follower_ticks = Some(r.follower_ticks);
                last_goal = Some(r.goal_ticks_sent);
                r.tick = tick;
                r.work_time = work_time;
                r
            }
            Err(e) => {
                stats.errors += 1;
                consecutive_errors += 1;
                if consecutive_errors >= fail_after {
                    // Fail-safe: never leave the follower holding a stale goal
                    // behind a fault. `run` switches its torque off on the way out.
                    return Err(Error::Other(format!(
                        "follower failed {consecutive_errors} ticks in a row ({e}); stopping with torque off"
                    )));
                }
                TickReport {
                    tick,
                    leader_ticks: [0; COUNT],
                    leader_values: [0.0; COUNT],
                    follower_ticks: last_follower_ticks.unwrap_or([0; COUNT]),
                    follower_values: [0.0; COUNT],
                    goal_ticks_sent: [0; COUNT],
                    goal_values_sent: [0.0; COUNT],
                    flags: [ServoError(0); COUNT],
                    released: releases.map(|r| r.is_some()),
                    telemetry: None,
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

// How long a follower may fail every tick before teleop stops.
const FAILSAFE_AFTER: Duration = Duration::from_millis(500);
// A joint is blocked when its goal is this far away...
const STALL_GAP_TICKS: i32 = 60;
// ...and it has moved no more than this per tick...
const STALL_STILL_TICKS: i32 = 2;
// ...for this long. Faster than the servo's own protection, which let the
// gripper squeeze at its full ceiling for ~2 s on hardware before tripping.
const STALL_TIME: Duration = Duration::from_millis(250);
// While blocked, the gripper keeps this much of a squeeze past the contact
// point: a chosen grip force instead of the servo's maximum. Other joints hold.
const GRIP_SQUEEZE_TICKS: i32 = 30;

#[derive(Default)]
struct StallWatch {
    prev: Option<Ticks>,
    stuck_s: [f64; COUNT],
    // Whether the servo's protection flag was up last tick. The flag stays up
    // for as long as the servo is in protection, so only its rising edge is a
    // new event; a flag still up from a hold says nothing about the next move.
    flagged: [bool; COUNT],
}

// A joint that is blocked (stall seen, or the servo tripped its overload or
// current protection): its goal is held at the contact point, plus a small
// squeeze on the gripper, until the leader moves back past that point.
#[derive(Clone, Copy, Debug)]
struct Release {
    at: i32,
    // Which way the command was pushing when it tripped: +1, -1 or 0.
    pushing: i32,
}

fn one_tick(
    leader: &mut Arm,
    follower: &mut Arm,
    options: TeleopOptions,
    last_goal: Option<Ticks>,
    releases: &mut [Option<Release>; COUNT],
    stalls: &mut StallWatch,
    predictor: &mut Predictor,
    tick_count: u64,
) -> Result<TickReport> {
    let follower_ticks = follower.read_ticks()?;
    let flags = follower.take_flags();
    let leader_ticks = leader.read_ticks()?;
    let _ = leader.take_flags();

    let leader_values = leader.ticks_to_values(&leader_ticks);
    let follower_values = follower.ticks_to_values(&follower_ticks);

    let target = predictor.ahead(Instant::now(), leader_values, options.lead);
    let mut goal_ticks = follower.values_to_ticks(&target);
    let period = 1.0 / options.hz;
    for i in 0..COUNT {
        // The goal that drove it into the stall is the previous one, not this
        // tick's: the leader may already be backing off on the tick it trips.
        // No previous goal means nothing this run has pushed yet: no hold.
        let gap = last_goal.map_or(0, |g| g[i] - follower_ticks[i]);
        let pushing_hard = gap.abs() > STALL_GAP_TICKS;
        let step = stalls.prev.map(|p| follower_ticks[i] - p[i]);
        let moved = step.map_or(i32::MAX, i32::abs);
        stalls.stuck_s[i] = if pushing_hard && moved <= STALL_STILL_TICKS { stalls.stuck_s[i] + period } else { 0.0 };
        // A trip only counts while the command pushes against something the
        // joint cannot get past. A flag on a joint at its goal, or one still
        // making progress toward it, is stale or noise, and holding there would
        // pin a free joint.
        let progress = step.map_or(i32::MAX, |d| d * gap.signum());
        let blocked = pushing_hard && progress <= STALL_STILL_TICKS;
        let flagged = flags[i].has(ServoError::OVERLOAD) || flags[i].has(ServoError::OVERCURRENT);
        let tripped = blocked && flagged && !stalls.flagged[i];
        stalls.flagged[i] = flagged;
        let stalled = stalls.stuck_s[i] >= STALL_TIME.as_secs_f64();
        if releases[i].is_none() && (tripped || stalled) {
            releases[i] = Some(Release { at: follower_ticks[i], pushing: gap.signum() });
        }
        if let Some(r) = releases[i] {
            if (goal_ticks[i] - r.at).signum() != r.pushing || r.pushing == 0 {
                releases[i] = None; // the leader backed off: follow again
                stalls.stuck_s[i] = 0.0;
            } else {
                let squeeze = if Joint::ALL[i] == Joint::Gripper { GRIP_SQUEEZE_TICKS } else { 0 };
                goal_ticks[i] = r.at + r.pushing * squeeze;
            }
        }
    }
    stalls.prev = Some(follower_ticks);
    // The step limit is a slew limit on the command: each goal may move at most
    // `max_step` from the previous goal (seeded from the follower's present
    // position, so nothing jumps at start). Limiting it against the present
    // position instead keeps the servo's error, and so its effort and speed,
    // pinned at `max_step`: measured on hardware, ~37 deg/s against a leader at
    // ~140 deg/s, and up to 1.5 s of lag.
    let reference = last_goal.unwrap_or(follower_ticks);
    let goal_ticks_sent = follower.write_goal_ticks(goal_ticks, Some(reference), options.max_step_ticks)?;
    let goal_values_sent = follower.ticks_to_values(&goal_ticks_sent);

    // A side channel: a failed telemetry read must not cost the tick.
    let telemetry = if options.telemetry {
        let joint = Joint::ALL[(tick_count % COUNT as u64) as usize];
        follower.read_telemetry_block(joint).ok()
    } else {
        None
    };

    Ok(TickReport {
        tick: Tick { index: 0, lateness: Duration::ZERO, skipped: 0 },
        leader_ticks,
        leader_values,
        follower_ticks,
        follower_values,
        goal_ticks_sent,
        goal_values_sent,
        flags,
        released: releases.map(|r| r.is_some()),
        telemetry,
        work_time: Duration::ZERO,
        error: None,
    })
}

// Leader velocity, smoothed, for commanding a little ahead of the hand.
#[derive(Default)]
struct Predictor {
    last: Option<(Instant, Values)>,
    velocity: Values,
}

impl Predictor {
    // Smoothing weight for each new velocity sample (0..1).
    const ALPHA: f64 = 0.35;
    // Speeds below this (deg/s, or %/s for the gripper) are hand tremor and
    // encoder noise: predicting from them only makes a still follower shiver.
    // Soft, so lead fades in rather than switching on.
    const DEADBAND: f64 = 12.0;

    fn ahead(&mut self, now: Instant, values: Values, lead: [Duration; COUNT]) -> Values {
        if let Some((t, prev)) = self.last {
            let dt = now.duration_since(t).as_secs_f64();
            if dt > 1e-4 {
                for i in 0..COUNT {
                    let v = (values[i] - prev[i]) / dt;
                    self.velocity[i] += Self::ALPHA * (v - self.velocity[i]);
                }
            }
        }
        self.last = Some((now, values));
        std::array::from_fn(|i| {
            let v = self.velocity[i];
            let beyond = (v.abs() - Self::DEADBAND).max(0.0) * v.signum();
            values[i] + beyond * lead[i].as_secs_f64()
        })
    }
}
