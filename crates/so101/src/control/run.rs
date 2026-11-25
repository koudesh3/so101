//! The control loop, on a grid of 120 Hz cycles. Every cycle reads the follower and writes one
//! goal; every 4th cycle is also a tick (30 Hz), where the pilot decides a target from what it
//! sees. The cycles in between step the goal linearly from the last target to the new one, so the
//! follower's guards still see every write. Each tick is published when its window ends, with the
//! goal actually sent (contract 2).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::arm::{Arm, Error, Joint, Limit, Pose, Result, JOINT_COUNT};
use crate::camera::LatestFrame;
use crate::clock::Clock;
use crate::control::{
    ArmRole, Clip, Event, EventBus, Grid, Health, JointTarget, Junk, Message, Observation, Pilot, Source, Tick,
};

// How long a follower may fail every cycle before the loop stops.
const FAILSAFE_AFTER: Duration = Duration::from_millis(500);
// A cycle that starts later than this after its grid time counts as late. On the Spark, 99.9% of
// cycles start within 0.45 ms (the Hanoi run, 2026-11-23).
pub const LATE_AFTER: Duration = Duration::from_millis(1);
/// Cycles per tick: 120 Hz cycles, 30 Hz ticks (the cameras' and the policy's rate).
pub const CYCLES_PER_TICK: u64 = 4;

#[derive(Clone, Copy, Debug)]
pub struct RunOptions {
    pub hz: f64,
    /// Stop after this many cycles. None runs until `stop` is set.
    pub max_cycles: Option<u64>,
    /// Read every follower actuator's status (load, voltage, current, ...) each cycle, in the same
    /// read as its position. About 0.8 ms more bus time per cycle.
    pub read_actuators: bool,
}

/// The cameras' newest frames, which each tick records and its pilot sees.
#[derive(Clone, Default)]
pub struct Cameras {
    pub wrist: Option<Arc<LatestFrame>>,
    pub scene: Option<Arc<LatestFrame>>,
}

#[derive(Debug, Default, Clone)]
pub struct RunStats {
    pub cycles: u64,
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

// One tick's window: the goal steps from `from` to the pilot's target over its cycles; the tick
// is published when the window ends, with what was sent.
struct Window {
    from: Pose,
    to: Pose,
    first_cycle: u64,
    // Its place on the grid of ticks: the cycle index / 4.
    number: u32,
    tick: Tick,
    sent: Option<Pose>,
    clipped: [Option<Limit>; JOINT_COUNT],
    published: bool,
}

/// Drives `follower` with `pilot` until `stop` is set or `max_cycles` have run, publishing every
/// event on `bus`. `on_error` hears each failed cycle; the previous goal then holds. With
/// `read_actuators`, a servo that stays too hot or a supply that sags stops the run (`Health`).
/// Follower torque is switched off on every exit path.
#[allow(clippy::too_many_arguments)]
pub fn run(
    follower: &mut impl Arm,
    pilot: &mut impl Pilot,
    options: RunOptions,
    cameras: &Cameras,
    clock: Arc<dyn Clock>,
    stop: &AtomicBool,
    bus: &EventBus,
    mut on_error: impl FnMut(&Error),
) -> Result<RunStats> {
    follower.hold()?;
    let loop_result = run_cycles(follower, pilot, options, cameras, &clock, stop, bus, &mut on_error);
    let torque_result = follower.torque_off();
    match (loop_result, torque_result) {
        (Err(e), _) => Err(e),
        (Ok(_), Err(e)) => Err(e),
        (Ok(stats), Ok(())) => Ok(stats),
    }
}

#[allow(clippy::too_many_arguments)]
fn run_cycles(
    follower: &mut impl Arm,
    pilot: &mut impl Pilot,
    options: RunOptions,
    cameras: &Cameras,
    clock: &Arc<dyn Clock>,
    stop: &AtomicBool,
    bus: &EventBus,
    on_error: &mut impl FnMut(&Error),
) -> Result<RunStats> {
    let mut grid = Grid::new(options.hz, clock.clone());
    let mut stats = RunStats::default();
    let mut consecutive_errors = 0u32;
    let fail_after = ((FAILSAFE_AFTER.as_secs_f64() * options.hz).ceil() as u32).max(3);
    let mut health = Health::default();
    let mut window: Option<Window> = None;

    while !stop.load(Ordering::Relaxed) {
        if options.max_cycles.is_some_and(|max| stats.cycles >= max) {
            break;
        }
        let cycle = grid.wait();
        stats.skipped += cycle.skipped;
        stats.late += u64::from(cycle.lateness > LATE_AFTER);
        stats.worst_lateness = stats.worst_lateness.max(cycle.lateness);
        // Ticks sit on every 4th grid point, so a skipped one leaves a gap in their numbers.
        let is_tick = cycle.index.is_multiple_of(CYCLES_PER_TICK);
        let work_started = Instant::now();
        let outcome =
            one_cycle(follower, pilot, options, cameras, clock.as_ref(), cycle.index, is_tick, &mut window, &mut stats);
        stats.worst_work_time = stats.worst_work_time.max(work_started.elapsed());
        stats.cycles += 1;
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
                        "follower failed {consecutive_errors} cycles in a row ({e}); stopping with torque off"
                    )));
                }
                on_error(&e);
            }
        }
    }
    // A window cut short (stopped, or its last cycles skipped or failed) still records its tick.
    if let Some(event) = window.as_mut().and_then(|w| finish(w, &mut stats)) {
        bus.publish(event);
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
#[allow(clippy::too_many_arguments)]
fn one_cycle(
    follower: &mut impl Arm,
    pilot: &mut impl Pilot,
    options: RunOptions,
    cameras: &Cameras,
    clock: &dyn Clock,
    index: u64,
    is_tick: bool,
    window: &mut Option<Window>,
    stats: &mut RunStats,
) -> Result<Vec<Event>> {
    let cycle = index as u32;
    let (state, actuators) = if options.read_actuators {
        follower.read_state_and_actuators()?
    } else {
        (follower.read_state()?, Vec::new())
    };
    let mut events = Vec::new();
    if is_tick {
        // The last window's tick, if its own last cycle never came.
        if let Some(event) = window.as_mut().and_then(|w| finish(w, stats)) {
            events.push(event);
        }
        let seen = Observation {
            t: clock.now(),
            follower: &state,
            wrist: cameras.wrist.as_ref().and_then(|c| c.get()),
            scene: cameras.scene.as_ref().and_then(|c| c.get()),
        };
        let (requested, observed) = pilot.target(&seen)?;
        let from = window.as_ref().and_then(|w| w.sent).unwrap_or(state.pose);
        events.extend(observed.into_iter().map(|m| {
            let t = m.time();
            Event::new(m, cycle, t)
        }));
        *window = Some(Window {
            from,
            to: requested,
            first_cycle: index,
            number: (index / CYCLES_PER_TICK) as u32,
            tick: tick_seen(&seen, cycle, pilot.source(), requested),
            sent: None,
            clipped: [None; JOINT_COUNT],
            published: false,
        });
    }
    let Some(w) = window.as_mut() else {
        // No target yet (the first tick's pilot failed): nothing to write.
        events.insert(0, Event::new(Message::JointState(ArmRole::Follower, state.clone()), cycle, state.t));
        return Ok(events);
    };
    let step = ((index - w.first_cycle + 1) as f32 / CYCLES_PER_TICK as f32).min(1.0);
    let goal = Pose(std::array::from_fn(|i| w.from.0[i] + (w.to.0[i] - w.from.0[i]) * step));
    let sent = follower.command(&goal)?;
    let t_sent = clock.now();
    w.sent = Some(sent.pose);
    for joint in Joint::ALL {
        let i = joint.index();
        w.clipped[i] = most_severe(w.clipped[i], sent.limits[i]);
    }

    let t_state = state.t;
    events.insert(0, Event::new(Message::JointState(ArmRole::Follower, state), cycle, t_state));
    events.push(Event::new(
        Message::JointTarget(
            ArmRole::Follower,
            JointTarget { t: t_sent, pose: sent.pose, source: w.tick.source.clone(), counts: sent.counts },
        ),
        cycle,
        t_sent,
    ));
    if index - w.first_cycle + 1 >= CYCLES_PER_TICK {
        events.extend(finish(w, stats));
    }
    events.extend(actuators.into_iter().map(|a| {
        let t = a.t;
        Event::new(Message::Actuator(ArmRole::Follower, a), cycle, t)
    }));
    Ok(events)
}

// The tick as decided; `action` and `clipped` are filled in when its window ends.
fn tick_seen(seen: &Observation, cycle: u32, source: Source, requested: Pose) -> Tick {
    Tick {
        t: seen.t,
        cycle,
        state: seen.follower.pose,
        wrist: seen.wrist.map(|f| f.id),
        scene: seen.scene.map(|f| f.id),
        action: requested,
        source,
        requested,
        clipped: Vec::new(),
    }
}

// Publishes the window's tick once: what was last sent, and every limit any of its cycles hit.
fn finish(w: &mut Window, stats: &mut RunStats) -> Option<Event> {
    if w.published {
        return None;
    }
    let sent = w.sent?;
    w.published = true;
    let mut tick = w.tick.clone();
    tick.action = sent;
    tick.clipped = Joint::ALL
        .into_iter()
        .filter_map(|joint| w.clipped[joint.index()].map(|limit| Clip { joint, limit }))
        .collect();
    stats.ticks += 1;
    let t = tick.t;
    Some(Event::new(Message::Tick(tick), w.number, t))
}

// Stall, then Collision, then Step, then Range (phase1.md).
fn most_severe(a: Option<Limit>, b: Option<Limit>) -> Option<Limit> {
    let rank = |l: Limit| match l {
        Limit::Stall => 3,
        Limit::Collision => 2,
        Limit::Step => 1,
        Limit::Range => 0,
    };
    match (a, b) {
        (Some(x), Some(y)) => Some(if rank(y) > rank(x) { y } else { x }),
        (x, y) => x.or(y),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_most_severe_limit_wins() {
        assert_eq!(most_severe(Some(Limit::Range), Some(Limit::Stall)), Some(Limit::Stall));
        assert_eq!(most_severe(Some(Limit::Collision), Some(Limit::Step)), Some(Limit::Collision));
        assert_eq!(most_severe(None, Some(Limit::Step)), Some(Limit::Step));
        assert_eq!(most_severe(None, None), None);
    }
}
