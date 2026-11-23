//! Teleop: the follower mirrors the leader through each arm's calibration. Every step publishes
//! both arms' `JointState`, the follower's `JointTarget` and a `Tick` (contract 2).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::arm::servo::error::{Error, Result};
use crate::arm::servo::protocol::ServoError;
use crate::arm::{Arm, Counts, Joint, JointState, JointTarget, Pose, Source, JOINT_COUNT};
use crate::control::grid::Grid;
use crate::{ArmRole, Clip, Event, Limit, Message, MonotonicClock, Tick, Timestamp};

#[derive(Clone, Copy, Debug)]
pub struct TeleopOptions {
    pub hz: f64,
    /// The most a goal may move per tick, in counts. None disables the limit.
    pub max_step_counts: Option<i32>,
    /// Stop after this many ticks. None runs until `stop` is set.
    pub max_ticks: Option<u64>,
    /// Also read one follower actuator's status per tick, rotating through the six. Costs one
    /// extra round trip per tick.
    pub read_actuators: bool,
    /// Per joint: command where the leader will be this far ahead, from its measured velocity,
    /// to cancel the follower's own response lag. Zero disables.
    pub lead: [Duration; JOINT_COUNT],
}

#[derive(Debug, Default, Clone)]
pub struct TeleopStats {
    pub ticks: u64,
    pub overruns: u64,
    pub errors: u64,
    pub worst_work_time: Duration,
}

/// `on_step` gets each step's events, or why the step failed (the previous goal then holds).
/// Follower torque is switched off on every exit path.
pub fn run(
    leader: &mut Arm,
    follower: &mut Arm,
    options: TeleopOptions,
    stop: Arc<AtomicBool>,
    mut on_step: impl FnMut(std::result::Result<&[Event], &str>),
) -> Result<TeleopStats> {
    leader.torque_off()?;
    follower.torque_on_holding()?;
    // Status bits raised before this run (a trip at the end of the last one,
    // say) describe nothing the loop did. Drop them so they cannot latch a hold.
    let _ = follower.take_flags();
    let _ = leader.take_flags();

    let loop_result = run_loop(leader, follower, options, &stop, &mut on_step);
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
    on_step: &mut impl FnMut(std::result::Result<&[Event], &str>),
) -> Result<TeleopStats> {
    let mut grid = Grid::new(options.hz);
    let mut stats = TeleopStats::default();
    let mut state = LoopState::default();
    let mut consecutive_errors = 0u32;
    let fail_after = ((FAILSAFE_AFTER.as_secs_f64() * options.hz).ceil() as u32).max(3);

    while !stop.load(Ordering::Relaxed) {
        if options.max_ticks.is_some_and(|max| stats.ticks >= max) {
            break;
        }
        let slot = grid.wait();
        let work_started = Instant::now();
        let outcome = one_step(leader, follower, options, &mut state, slot.index, stats.ticks);
        let work_time = work_started.elapsed();
        stats.ticks += 1;
        stats.worst_work_time = stats.worst_work_time.max(work_time);

        match outcome {
            Ok(events) => {
                consecutive_errors = 0;
                on_step(Ok(&events));
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
                on_step(Err(&e.to_string()));
            }
        }
    }
    stats.overruns = grid.overruns;
    Ok(stats)
}

// How long a follower may fail every tick before teleop stops.
const FAILSAFE_AFTER: Duration = Duration::from_millis(500);
// A joint is blocked when its goal is this far away...
const STALL_GAP_COUNTS: i32 = 60;
// ...and it has moved no more than this per tick...
const STALL_STILL_COUNTS: i32 = 2;
// ...for this long. Faster than the servo's own protection, which let the
// gripper squeeze at its full ceiling for ~2 s on hardware before tripping.
const STALL_TIME: Duration = Duration::from_millis(250);
// While held, the gripper keeps this much of a squeeze past the contact
// point: a chosen grip force instead of the servo's maximum. Other joints hold.
const GRIP_SQUEEZE_COUNTS: i32 = 30;
// One count, in the pose's units: a requested value further than this from where the range
// clamp put it was clipped by the range.
const ONE_COUNT_DEG: f32 = 360.0 / 4096.0;

#[derive(Default)]
struct LoopState {
    last_goal: Option<Counts>,
    holds: [Option<Hold>; JOINT_COUNT],
    stalls: StallWatch,
    predictor: Predictor,
}

#[derive(Default)]
struct StallWatch {
    prev: Option<Counts>,
    stuck_s: [f64; JOINT_COUNT],
    // Whether the servo's protection flag was up last tick. The flag stays up
    // for as long as the servo is in protection, so only its rising edge is a
    // new event; a flag still up from a hold says nothing about the next move.
    flagged: [bool; JOINT_COUNT],
}

// A blocked joint (stall seen, or the servo tripped its overload or current protection): its
// goal is held at the contact point, plus a small squeeze on the gripper, until the leader moves
// back past that point.
#[derive(Clone, Copy, Debug)]
struct Hold {
    at: i32,
    pushing: Direction,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Direction {
    Up,
    Down,
    Neither,
}

impl Direction {
    fn of(delta: i32) -> Self {
        match delta.signum() {
            1 => Direction::Up,
            -1 => Direction::Down,
            _ => Direction::Neither,
        }
    }

    fn sign(self) -> i32 {
        match self {
            Direction::Up => 1,
            Direction::Down => -1,
            Direction::Neither => 0,
        }
    }
}

fn one_step(
    leader: &mut Arm,
    follower: &mut Arm,
    options: TeleopOptions,
    state: &mut LoopState,
    slot: u64,
    tick: u64,
) -> Result<Vec<Event>> {
    let follower_counts = follower.read_counts()?;
    let t_follower = MonotonicClock::read();
    let flags = follower.take_flags();
    let leader_counts = leader.read_counts()?;
    let t_leader = MonotonicClock::read();
    let _ = leader.take_flags();

    let leader_pose = leader.to_pose(&leader_counts);
    let follower_pose = follower.to_pose(&follower_counts);

    let requested = state.predictor.ahead(Instant::now(), leader_pose, options.lead);
    let in_range = follower.to_counts(&requested);
    let mut goal = in_range;
    let period = 1.0 / options.hz;
    let (holds, stalls) = (&mut state.holds, &mut state.stalls);
    for i in 0..JOINT_COUNT {
        // The goal that drove it into the stall is the previous one, not this
        // tick's: the leader may already be backing off on the tick it trips.
        // No previous goal means nothing this run has pushed yet: no hold.
        let gap = state.last_goal.map_or(0, |g| g[i] - follower_counts[i]);
        let pushing_hard = gap.abs() > STALL_GAP_COUNTS;
        let step = stalls.prev.map(|p| follower_counts[i] - p[i]);
        let moved = step.map_or(i32::MAX, i32::abs);
        stalls.stuck_s[i] = if pushing_hard && moved <= STALL_STILL_COUNTS { stalls.stuck_s[i] + period } else { 0.0 };
        // A trip only counts while the command pushes against something the
        // joint cannot get past. A flag on a joint at its goal, or one still
        // making progress toward it, is stale or noise, and holding there would
        // pin a free joint.
        let progress = step.map_or(i32::MAX, |d| d * gap.signum());
        let blocked = pushing_hard && progress <= STALL_STILL_COUNTS;
        let flagged = flags[i].has(ServoError::OVERLOAD) || flags[i].has(ServoError::OVERCURRENT);
        let tripped = blocked && flagged && !stalls.flagged[i];
        stalls.flagged[i] = flagged;
        let stalled = stalls.stuck_s[i] >= STALL_TIME.as_secs_f64();
        if holds[i].is_none() && (tripped || stalled) {
            holds[i] = Some(Hold { at: follower_counts[i], pushing: Direction::of(gap) });
        }
        if let Some(h) = holds[i] {
            if Direction::of(goal[i] - h.at) != h.pushing || h.pushing == Direction::Neither {
                holds[i] = None; // the leader backed off: follow again
                stalls.stuck_s[i] = 0.0;
            } else {
                let squeeze = if Joint::ALL[i] == Joint::Gripper { GRIP_SQUEEZE_COUNTS } else { 0 };
                goal[i] = h.at + h.pushing.sign() * squeeze;
            }
        }
    }
    stalls.prev = Some(follower_counts);
    // The step limit is a slew limit on the command: each goal may move at most
    // `max_step` from the previous goal (seeded from the follower's present
    // position, so nothing jumps at start). Limiting it against the present
    // position instead keeps the servo's error, and so its effort and speed,
    // pinned at `max_step`: measured on hardware, ~37 deg/s against a leader at
    // ~140 deg/s, and up to 1.5 s of lag.
    let reference = state.last_goal.unwrap_or(follower_counts);
    let sent = follower.write_goal_counts(goal, Some(reference), options.max_step_counts)?;
    let t_sent = MonotonicClock::read();
    state.last_goal = Some(sent);
    let action = follower.to_pose(&sent);

    let range_pose = follower.to_pose(&in_range);
    let clipped = Joint::ALL
        .into_iter()
        .filter_map(|joint| {
            let i = joint.index();
            let limit = if holds[i].is_some() {
                Limit::Stall
            } else if sent[i] != goal[i] {
                Limit::Step
            } else if (range_pose.0[i] - requested.0[i]).abs() > ONE_COUNT_DEG {
                Limit::Range
            } else {
                return None;
            };
            Some(Clip { joint, limit })
        })
        .collect();

    let slot = slot as u32;
    let mut events = vec![
        state_event(ArmRole::Follower, t_follower, follower_pose, follower_counts, slot),
        state_event(ArmRole::Leader, t_leader, leader_pose, leader_counts, slot),
        Event::new(
            Message::JointTarget(
                ArmRole::Follower,
                JointTarget { t: t_sent, pose: action, source: Source::Teleop, counts: Some(sent) },
            ),
            slot,
            t_sent,
        ),
        Event::new(
            Message::Tick(Tick {
                t: t_sent,
                slot,
                state: follower_pose,
                wrist: None,
                scene: None,
                action,
                source: Source::Teleop,
                requested,
                clipped,
            }),
            tick as u32,
            t_sent,
        ),
    ];

    // A side channel: a failed status read must not cost the step.
    if options.read_actuators {
        let joint = Joint::ALL[(tick % JOINT_COUNT as u64) as usize];
        if let Ok(status) = follower.read_actuator(joint) {
            let t = status.t;
            events.push(Event::new(Message::Actuator(ArmRole::Follower, status), slot, t));
        }
    }
    Ok(events)
}

fn state_event(role: ArmRole, t: Timestamp, pose: Pose, counts: Counts, slot: u32) -> Event {
    Event::new(Message::JointState(role, JointState { t, pose, counts: Some(counts) }), slot, t)
}

// Leader velocity, smoothed, for commanding a little ahead of the hand.
#[derive(Default)]
struct Predictor {
    last: Option<(Instant, Pose)>,
    velocity: [f32; JOINT_COUNT],
}

impl Predictor {
    // Smoothing weight for each new velocity sample (0..1).
    const ALPHA: f32 = 0.35;
    // Speeds below this (deg/s, or %/s for the gripper) are hand tremor and
    // encoder noise: predicting from them only makes a still follower shiver.
    // Soft, so lead fades in rather than switching on.
    const DEADBAND: f32 = 12.0;

    fn ahead(&mut self, now: Instant, pose: Pose, lead: [Duration; JOINT_COUNT]) -> Pose {
        if let Some((t, prev)) = self.last {
            let dt = now.duration_since(t).as_secs_f32();
            if dt > 1e-4 {
                for i in 0..JOINT_COUNT {
                    let v = (pose.0[i] - prev.0[i]) / dt;
                    self.velocity[i] += Self::ALPHA * (v - self.velocity[i]);
                }
            }
        }
        self.last = Some((now, pose));
        Pose(std::array::from_fn(|i| {
            let v = self.velocity[i];
            let beyond = (v.abs() - Self::DEADBAND).max(0.0) * v.signum();
            pose.0[i] + beyond * lead[i].as_secs_f32()
        }))
    }
}
