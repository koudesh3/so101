//! The control loop: on every cycle of the grid, read the follower, ask the pilot for a target,
//! command it, and publish what happened.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::arm::{Arm, Error, Joint, Result};
use crate::clock::Clock;
use crate::control::{ArmRole, Clip, Event, EventBus, Grid, Health, JointTarget, Junk, Message, Pilot, Tick};

// How long a follower may fail every cycle before the loop stops.
const FAILSAFE_AFTER: Duration = Duration::from_millis(500);
// A cycle that starts later than this after its grid time counts as late. On the Spark, 99.9% of
// cycles start within 0.45 ms (the Hanoi run, 2026-11-23).
pub const LATE_AFTER: Duration = Duration::from_millis(1);

#[derive(Clone, Copy, Debug)]
pub struct RunOptions {
    pub hz: f64,
    /// Stop after this many ticks. None runs until `stop` is set.
    pub max_ticks: Option<u64>,
    /// Read every follower actuator's status (load, voltage, current, ...) each cycle, in the same
    /// read as its position. About 0.8 ms more bus time per cycle.
    pub read_actuators: bool,
}

#[derive(Debug, Default, Clone)]
pub struct RunStats {
    pub ticks: u64,
    /// Grid points that passed while earlier work ran long: cycles that never happened.
    pub skipped: u64,
    /// Cycles that started more than `LATE_AFTER` after their grid time.
    pub late: u64,
    pub worst_lateness: Duration,
    pub errors: u64,
    /// Real compute time, whatever the clock: what the loop costs the host.
    pub worst_work_time: Duration,
    /// Actuator readings the health filter judged junk (with `read_actuators`).
    pub junk: Junk,
}

/// Drives `follower` with `pilot` until `stop` is set or `max_ticks` have run, publishing every
/// event on `bus`. `on_error` hears each failed step; the previous goal then holds. With
/// `read_actuators`, a servo that stays too hot or a supply that sags stops the run (`Health`).
/// Follower torque is switched off on every exit path.
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
    let mut health = Health::default();
    let fail_after = ((FAILSAFE_AFTER.as_secs_f64() * options.hz).ceil() as u32).max(3);

    while !stop.load(Ordering::Relaxed) {
        if options.max_ticks.is_some_and(|max| stats.ticks >= max) {
            break;
        }
        let cycle = grid.wait();
        stats.skipped += cycle.skipped;
        stats.late += u64::from(cycle.lateness > LATE_AFTER);
        stats.worst_lateness = stats.worst_lateness.max(cycle.lateness);
        let work_started = Instant::now();
        let outcome = one_cycle(follower, pilot, options, clock.as_ref(), cycle.index, stats.ticks);
        stats.worst_work_time = stats.worst_work_time.max(work_started.elapsed());
        stats.ticks += 1;
        match outcome {
            Ok(events) => {
                consecutive_errors = 0;
                let mut trip = None;
                for e in &events {
                    if let Message::Actuator(ArmRole::Follower, a) = &e.message {
                        trip = trip.or(health.add(a));
                    }
                }
                // Published first, so a recording keeps the readings that tripped it.
                for event in events {
                    bus.publish(event);
                }
                stats.junk = health.junk();
                if let Some(trip) = trip {
                    return Err(Error::Other(format!("health: {trip}; stopping with torque off")));
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
    Ok(stats)
}

/// Gives the calling thread real-time (SCHED_FIFO) priority, so the OS can't preempt the loop for
/// ordinary work: at normal priority a cycle started 2-4.5 ms late every few minutes on the Spark.
/// Linux only, and the user needs an rtprio limit (`/etc/security/limits.d`); call it on the
/// thread that will call `run`. Returns why not, if it can't.
pub fn request_realtime(priority: i32) -> std::result::Result<(), String> {
    #[cfg(target_os = "linux")]
    {
        let param = libc::sched_param { sched_priority: priority };
        // SAFETY: plain syscall on the calling thread with a valid sched_param.
        if unsafe { libc::sched_setscheduler(0, libc::SCHED_FIFO, &param) } == 0 {
            Ok(())
        } else {
            Err(format!("SCHED_FIFO {priority} refused: {}", std::io::Error::last_os_error()))
        }
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = priority;
        Err("real-time priority is Linux only".into())
    }
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
    let (follower_state, actuators) = if options.read_actuators {
        follower.read_state_and_actuators()?
    } else {
        (follower.read_state()?, Vec::new())
    };
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
    events.extend(actuators.into_iter().map(|a| {
        let t = a.t;
        Event::new(Message::Actuator(ArmRole::Follower, a), cycle, t)
    }));
    Ok(events)
}
