//! Teleop: the follower mirrors the leader through each arm's calibration. Every step publishes
//! both arms' `JointState`, the follower's `JointTarget` and a `Tick` (contract 2).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::arm::{Arm, Counts, Fault, Joint, JointState, JointTarget, Pose, Source, JOINT_COUNT};
use crate::arm::{Error, Result};
use crate::control::Grid;
use crate::control::{ArmRole, Clip, Event, Limit, Message, Tick};

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

// Leader velocity, smoothed, for commanding a little ahead of the hand.
#[derive(Default)]
struct Predictor {
    last: Option<(Instant, Pose)>,
    velocity: [f32; JOINT_COUNT],
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

/// `on_step` gets each step's events, or why the step failed (the previous goal then holds).
/// Follower torque is switched off on every exit path.
pub fn run(
    leader: &mut impl Arm,
    follower: &mut impl Arm,
    options: TeleopOptions,
    stop: Arc<AtomicBool>,
    mut on_step: impl FnMut(std::result::Result<&[Event], &str>),
) -> Result<TeleopStats> {
    leader.torque_off()?;
    follower.hold()?;
    // Faults raised before this run (a trip at the end of the last one, say)
    // describe nothing the loop did. Drop them so they cannot latch a hold.
    let _ = follower.take_faults();
    let _ = leader.take_faults();

    let loop_result = run_loop(leader, follower, options, &stop, &mut on_step);
    let torque_result = follower.torque_off();

    match (loop_result, torque_result) {
        (Err(e), _) => Err(e),
        (Ok(_), Err(e)) => Err(e),
        (Ok(stats), Ok(())) => Ok(stats),
    }
}

fn run_loop(
    leader: &mut impl Arm,
    follower: &mut impl Arm,
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

fn one_step(
    leader: &mut impl Arm,
    follower: &mut impl Arm,
    options: TeleopOptions,
    state: &mut LoopState,
    slot: u64,
    tick: u64,
) -> Result<Vec<Event>> {
    let follower_state = follower.read_state()?;
    let faults = follower.take_faults();
    let leader_state = leader.read_state()?;
    let _ = leader.take_faults();
    let follower_counts = counts(&follower_state)?;
    let (follower_pose, leader_pose) = (follower_state.pose, leader_state.pose);

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
        let flagged = faults[i].iter().any(|f| matches!(f, Fault::Overload | Fault::Overcurrent));
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
    let from = state.last_goal.unwrap_or(follower_counts);
    let sent: Counts = match options.max_step_counts {
        Some(max) => std::array::from_fn(|i| from[i] + (goal[i] - from[i]).clamp(-max, max)),
        None => goal,
    };
    follower.write_goal(&sent)?;
    let t_sent = crate::clock::MonotonicClock::read();
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
        state_event(ArmRole::Follower, follower_state, slot),
        state_event(ArmRole::Leader, leader_state, slot),
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

fn state_event(role: ArmRole, state: JointState, slot: u32) -> Event {
    let t = state.t;
    Event::new(Message::JointState(role, state), slot, t)
}

/// Teleop's stall logic works in encoder counts, so it needs an arm that reports them.
fn counts(state: &JointState) -> Result<Counts> {
    state.counts.ok_or_else(|| "teleop needs an arm that reports encoder counts".into())
}

#[cfg(test)]
mod tests {
    use super::*;

    const LEAD: [Duration; JOINT_COUNT] = [Duration::from_millis(40); JOINT_COUNT];

    // Feed `steps` poses, 10 ms apart, moving pan at `deg_per_s`; return the last prediction.
    fn predict(deg_per_s: f32, steps: u32, lead: [Duration; JOINT_COUNT]) -> (Pose, Pose) {
        let mut p = Predictor::default();
        let t0 = Instant::now();
        let mut last = (Pose::default(), Pose::default());
        for i in 0..steps {
            let mut pose = Pose::default();
            pose.0[0] = deg_per_s * i as f32 * 0.01;
            last = (pose, p.ahead(t0 + Duration::from_millis(10 * i as u64), pose, lead));
        }
        last
    }

    #[test]
    fn a_still_hand_gets_no_lead() {
        let (pose, ahead) = predict(0.0, 50, LEAD);
        assert_eq!(ahead, pose);
    }

    #[test]
    fn tremor_below_the_deadband_gets_no_lead() {
        let (pose, ahead) = predict(Predictor::DEADBAND * 0.9, 50, LEAD);
        assert_eq!(ahead, pose);
    }

    #[test]
    fn steady_motion_leads_by_the_speed_above_the_deadband() {
        let (pose, ahead) = predict(100.0, 50, LEAD);
        let want = (100.0 - Predictor::DEADBAND) * 0.040;
        let got = ahead.0[0] - pose.0[0];
        assert!((got - want).abs() < 0.01, "lead {got:.3} deg, wanted {want:.3}");
        assert_eq!(ahead.0[1..], pose.0[1..], "only the moving joint leads");
    }

    #[test]
    fn zero_lead_passes_the_pose_through() {
        let (pose, ahead) = predict(100.0, 50, [Duration::ZERO; JOINT_COUNT]);
        assert_eq!(ahead, pose);
    }

    #[test]
    fn direction_follows_the_sign_of_the_gap() {
        assert_eq!(Direction::of(60), Direction::Up);
        assert_eq!(Direction::of(-1), Direction::Down);
        assert_eq!(Direction::of(0), Direction::Neither);
        assert_eq!(Direction::Down.sign(), -1);
    }
}
