//! The control loop: on every cycle of the grid, read the follower, ask the pilot for a target,
//! command it, and publish what happened.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::arm::{Arm, Error, Joint, JointTarget, Result, JOINT_COUNT};
use crate::clock::Clock;
use crate::control::{ArmRole, Clip, Event, EventBus, Grid, Message, Pilot, Tick};

// How long a follower may fail every cycle before the loop stops.
const FAILSAFE_AFTER: Duration = Duration::from_millis(500);

#[derive(Clone, Copy, Debug)]
pub struct RunOptions {
    pub hz: f64,
    /// Stop after this many ticks. None runs until `stop` is set.
    pub max_ticks: Option<u64>,
    /// Also read one follower actuator's status per tick, rotating through the six. Costs one
    /// extra round trip per tick.
    pub read_actuators: bool,
}

#[derive(Debug, Default, Clone)]
pub struct RunStats {
    pub ticks: u64,
    pub overruns: u64,
    pub errors: u64,
    /// Real compute time, whatever the clock: what the loop costs the host.
    pub worst_work_time: Duration,
}

/// Drives `follower` with `pilot` until `stop` is set or `max_ticks` have run, publishing every
/// event on `bus`. `on_error` hears each failed step; the previous goal then holds. Follower
/// torque is switched off on every exit path.
pub fn run(
    follower: &mut impl Arm,
    pilot: &mut impl Pilot,
    options: RunOptions,
    clock: Arc<dyn Clock>,
    stop: &AtomicBool,
    bus: &EventBus,
    mut on_error: impl FnMut(&Error),
) -> Result<RunStats> {
    follower.hold()?;
    let loop_result = run_cycles(follower, pilot, options, &clock, stop, bus, &mut on_error);
    let torque_result = follower.torque_off();
    match (loop_result, torque_result) {
        (Err(e), _) => Err(e),
        (Ok(_), Err(e)) => Err(e),
        (Ok(stats), Ok(())) => Ok(stats),
    }
}

fn run_cycles(
    follower: &mut impl Arm,
    pilot: &mut impl Pilot,
    options: RunOptions,
    clock: &Arc<dyn Clock>,
    stop: &AtomicBool,
    bus: &EventBus,
    on_error: &mut impl FnMut(&Error),
) -> Result<RunStats> {
    let mut grid = Grid::new(options.hz, clock.clone());
    let mut stats = RunStats::default();
    let mut consecutive_errors = 0u32;
    let fail_after = ((FAILSAFE_AFTER.as_secs_f64() * options.hz).ceil() as u32).max(3);

    while !stop.load(Ordering::Relaxed) {
        if options.max_ticks.is_some_and(|max| stats.ticks >= max) {
            break;
        }
        let cycle = grid.wait();
        let work_started = Instant::now();
        let outcome = one_cycle(follower, pilot, options, clock.as_ref(), cycle.index, stats.ticks);
        stats.worst_work_time = stats.worst_work_time.max(work_started.elapsed());
        stats.ticks += 1;
        match outcome {
            Ok(events) => {
                consecutive_errors = 0;
                for event in events {
                    bus.publish(event);
                }
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
                on_error(&e);
            }
        }
    }
    stats.overruns = grid.overruns;
    Ok(stats)
}

/// One cycle's events, published together only if the whole cycle succeeds.
fn one_cycle(
    follower: &mut impl Arm,
    pilot: &mut impl Pilot,
    options: RunOptions,
    clock: &dyn Clock,
    cycle: u64,
    tick: u64,
) -> Result<Vec<Event>> {
    let follower_state = follower.read_state()?;
    let (requested, observed) = pilot.target(clock.now(), &follower_state)?;
    let sent = follower.command(&requested)?;
    let t_sent = clock.now();
    let source = pilot.source();
    let clipped = Joint::ALL
        .into_iter()
        .filter_map(|joint| sent.limits[joint.index()].map(|limit| Clip { joint, limit }))
        .collect();

    let cycle = cycle as u32;
    let follower_pose = follower_state.pose;
    let t_state = follower_state.t;
    let mut events = vec![Event::new(Message::JointState(ArmRole::Follower, follower_state), cycle, t_state)];
    events.extend(observed.into_iter().map(|m| {
        let t = m.time();
        Event::new(m, cycle, t)
    }));
    events.push(Event::new(
        Message::JointTarget(
            ArmRole::Follower,
            JointTarget { t: t_sent, pose: sent.pose, source: source.clone(), counts: sent.counts },
        ),
        cycle,
        t_sent,
    ));
    events.push(Event::new(
        Message::Tick(Tick {
            t: t_sent,
            cycle,
            state: follower_pose,
            wrist: None,
            scene: None,
            action: sent.pose,
            source,
            requested,
            clipped,
        }),
        tick as u32,
        t_sent,
    ));

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
