//! Teleop: the follower mirrors the leader through each arm's calibration. Every step publishes
//! both arms' `JointState`, the follower's `JointTarget` and a `Tick` (contract 2).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::arm::{Arm, Joint, JointState, JointTarget, Pose, Source, JOINT_COUNT};
use crate::arm::{Error, Result};
use crate::clock::{Clock, Timestamp};
use crate::control::Grid;
use crate::control::{ArmRole, Clip, Event, Message, Tick};

// How long a follower may fail every tick before teleop stops.
const FAILSAFE_AFTER: Duration = Duration::from_millis(500);

#[derive(Clone, Copy, Debug)]
pub struct TeleopOptions {
    pub hz: f64,
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
    predictor: Predictor,
}

// Leader velocity, smoothed, for commanding a little ahead of the hand.
#[derive(Default)]
struct Predictor {
    last: Option<(Timestamp, Pose)>,
    velocity: [f32; JOINT_COUNT],
}

impl Predictor {
    // Smoothing weight for each new velocity sample (0..1).
    const ALPHA: f32 = 0.35;
    // Speeds below this (deg/s, or %/s for the gripper) are hand tremor and
    // encoder noise: predicting from them only makes a still follower shiver.
    // Soft, so lead fades in rather than switching on.
    const DEADBAND: f32 = 12.0;

    fn ahead(&mut self, now: Timestamp, pose: Pose, lead: [Duration; JOINT_COUNT]) -> Pose {
        if let Some((t, prev)) = self.last {
            let dt = now.since(t).as_secs_f32();
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
    clock: Arc<dyn Clock>,
    stop: Arc<AtomicBool>,
    mut on_step: impl FnMut(std::result::Result<&[Event], &str>),
) -> Result<TeleopStats> {
    leader.torque_off()?;
    follower.hold()?;

    let loop_result = run_loop(leader, follower, options, &clock, &stop, &mut on_step);
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
    clock: &Arc<dyn Clock>,
    stop: &AtomicBool,
    on_step: &mut impl FnMut(std::result::Result<&[Event], &str>),
) -> Result<TeleopStats> {
    let mut grid = Grid::new(options.hz, clock.clone());
    let mut stats = TeleopStats::default();
    let mut state = LoopState::default();
    let mut consecutive_errors = 0u32;
    let fail_after = ((FAILSAFE_AFTER.as_secs_f64() * options.hz).ceil() as u32).max(3);

    while !stop.load(Ordering::Relaxed) {
        if options.max_ticks.is_some_and(|max| stats.ticks >= max) {
            break;
        }
        let cycle = grid.wait();
        let work_started = Instant::now();
        let outcome = one_step(leader, follower, options, clock.as_ref(), &mut state, cycle.index, stats.ticks);
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
    clock: &dyn Clock,
    state: &mut LoopState,
    cycle: u64,
    tick: u64,
) -> Result<Vec<Event>> {
    let follower_state = follower.read_state()?;
    let leader_state = leader.read_state()?;
    let requested = state.predictor.ahead(clock.now(), leader_state.pose, options.lead);
    let sent = follower.command(&requested)?;
    let t_sent = clock.now();
    let clipped = Joint::ALL
        .into_iter()
        .filter_map(|joint| sent.limits[joint.index()].map(|limit| Clip { joint, limit }))
        .collect();

    let cycle = cycle as u32;
    let follower_pose = follower_state.pose;
    let mut events = vec![
        state_event(ArmRole::Follower, follower_state, cycle),
        state_event(ArmRole::Leader, leader_state, cycle),
        Event::new(
            Message::JointTarget(
                ArmRole::Follower,
                JointTarget { t: t_sent, pose: sent.pose, source: Source::Teleop, counts: sent.counts },
            ),
            cycle,
            t_sent,
        ),
        Event::new(
            Message::Tick(Tick {
                t: t_sent,
                cycle,
                state: follower_pose,
                wrist: None,
                scene: None,
                action: sent.pose,
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
            events.push(Event::new(Message::Actuator(ArmRole::Follower, status), cycle, t));
        }
    }
    Ok(events)
}

fn state_event(role: ArmRole, state: JointState, cycle: u32) -> Event {
    let t = state.t;
    Event::new(Message::JointState(role, state), cycle, t)
}

#[cfg(test)]
mod tests {
    use super::*;

    const LEAD: [Duration; JOINT_COUNT] = [Duration::from_millis(40); JOINT_COUNT];

    // Feed `steps` poses, 10 ms apart, moving pan at `deg_per_s`; return the last prediction.
    fn predict(deg_per_s: f32, steps: u32, lead: [Duration; JOINT_COUNT]) -> (Pose, Pose) {
        let mut p = Predictor::default();
        let t0 = Timestamp(0);
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
}
