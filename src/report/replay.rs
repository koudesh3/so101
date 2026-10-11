//! Turn a byte log back into the dashboard's event stream, so a recorded
//! episode plays on the page with no arms attached (`dashboard.html?replay=`).
//!
//! Teleop's bus traffic has a fixed shape per tick: a follower sync read of
//! Present_Position starts it, then the leader's sync read, the follower's
//! sync write of goals, and (with telemetry on) one follower block read. The
//! replies carry everything a live tick event does, so the events come out
//! in the same shapes `teleop --serve` publishes. Timing is the log's own:
//! work time is the span of a tick's frames, lateness is not recorded.

use std::time::Duration;

use crate::control::teleop::TickReport;
use crate::control::ticker::Tick;
use crate::report::dashboard::{hello_json, tick_json, wire_json};
use crate::report::visualize::decode;
use crate::robot::arm::{Telemetry, TELEMETRY_LEN, TELEMETRY_START};
use crate::robot::calibration::ArmCalibration;
use crate::robot::joint::{Joint, Ticks, COUNT};
use crate::wire::protocol::{Instruction, ServoError};
use crate::wire::registers;
use crate::wire::transport::{Direction, LogRecord};

/// What the host last asked on a port, which says what a reply means.
#[derive(Clone, Copy, PartialEq)]
enum Asked {
    Other,
    Positions,
    Telemetry(Joint),
}

struct Building {
    start_us: u64,
    end_us: u64,
    leader: [Option<i32>; COUNT],
    follower: [Option<i32>; COUNT],
    goals: Option<Ticks>,
    flags: [ServoError; COUNT],
    telemetry: Option<Telemetry>,
}

impl Building {
    fn new(t_us: u64) -> Self {
        Self {
            start_us: t_us,
            end_us: t_us,
            leader: [None; COUNT],
            follower: [None; COUNT],
            goals: None,
            flags: [ServoError(0); COUNT],
            telemetry: None,
        }
    }
}

/// Every event for the log, one JSON object per entry: "hello", then the
/// ticks and wire frames in time order, then "end". Frames before the first
/// tick (setup: pings, configuration writes, the initial hold) are left out.
pub fn events_from_log(records: &[LogRecord], cal: [&ArmCalibration; 2]) -> Vec<String> {
    let decoded = decode(records);
    let mut ticks: Vec<Building> = Vec::new();
    let mut wires: Vec<(u64, String)> = Vec::new();
    let mut asked = [Asked::Other; 2];

    for f in &decoded.frames {
        let port = if f.port == "follower" { 1 } else { 0 };
        let p = &f.frame.params;
        if f.dir == Direction::HostToBus {
            let instruction = f.frame.instruction();
            let reads_positions = instruction == Some(Instruction::SyncRead) && p.first() == Some(&registers::PRESENT_POSITION.address);
            if port == 1 && reads_positions {
                ticks.push(Building::new(f.t_us));
            }
            asked[port] = if reads_positions {
                Asked::Positions
            } else if instruction == Some(Instruction::Read) && p.len() >= 2 && p[0] == TELEMETRY_START && p[1] == TELEMETRY_LEN {
                Joint::from_id(f.frame.id).map_or(Asked::Other, Asked::Telemetry)
            } else {
                Asked::Other
            };
            if let Some(tick) = ticks.last_mut() {
                if port == 1 && instruction == Some(Instruction::SyncWrite) && p.first() == Some(&registers::GOAL_POSITION.address) {
                    let width = p[1] as usize;
                    let mut goals = [0; COUNT];
                    for block in p[2..].chunks_exact(1 + width) {
                        if let Some(j) = Joint::from_id(block[0]) {
                            goals[j.index()] = registers::GOAL_POSITION.decode(&block[1..]);
                        }
                    }
                    tick.goals = Some(goals);
                }
            }
        } else if let Some(tick) = ticks.last_mut() {
            let error = ServoError(f.frame.fourth_byte);
            match (asked[port], Joint::from_id(f.frame.id)) {
                (Asked::Positions, Some(j)) if p.len() >= 2 => {
                    let value = registers::PRESENT_POSITION.decode(p);
                    if port == 0 {
                        tick.leader[j.index()] = Some(value);
                    } else {
                        tick.follower[j.index()] = Some(value);
                        tick.flags[j.index()].0 |= error.0;
                    }
                }
                (Asked::Telemetry(j), Some(id)) if id == j => tick.telemetry = Telemetry::from_block(j, p),
                _ => {}
            }
        }
        if let Some(tick) = ticks.last_mut() {
            tick.end_us = f.t_us;
            wires.push((f.t_us, wire_json(f.t_us, port as u8, if f.dir == Direction::HostToBus { 0 } else { 1 }, &f.bytes)));
        }
    }

    // Before the loop, teleop reads the follower's pose once to hold it as
    // torque comes on. That read has no leader read after it: it is setup,
    // not a tick, and neither it nor anything before it is replayed.
    let first = ticks.iter().position(|b| b.leader.iter().any(Option::is_some)).unwrap_or(ticks.len());
    ticks.drain(..first);
    if let Some(start) = ticks.first().map(|b| b.start_us) {
        wires.retain(|(t, _)| *t >= start);
    }

    // The rate the loop actually ran at: the median gap between tick starts.
    let mut gaps: Vec<u64> = ticks.windows(2).map(|w| w[1].start_us - w[0].start_us).collect();
    gaps.sort_unstable();
    let hz = gaps.get(gaps.len() / 2).map_or(30.0, |&g| (1e6 / g.max(1) as f64).round());

    let ports = ["leader".to_string(), "follower".to_string()];
    let mut out = vec![hello_json(hz, 0, false, &ports, cal)];
    let mut wires = wires.into_iter().peekable();
    let mut last = [[0; COUNT], [0; COUNT], [0; COUNT]];
    let mut errors = 0;
    for (index, b) in ticks.iter().enumerate() {
        // A reply lost to a timeout keeps the joint's previous reading, as the
        // dashboard would have shown it.
        let fill = |got: [Option<i32>; COUNT], prev: Ticks| std::array::from_fn(|i| got[i].unwrap_or(prev[i]));
        let complete = b.leader.iter().chain(&b.follower).all(Option::is_some) && b.goals.is_some();
        let (leader, follower, goals) = (fill(b.leader, last[0]), fill(b.follower, last[1]), b.goals.unwrap_or(last[2]));
        last = [leader, follower, goals];
        if !complete {
            errors += 1;
        }
        let report = TickReport {
            tick: Tick { index: index as u64, lateness: Duration::ZERO, skipped: 0 },
            leader_ticks: leader,
            leader_values: cal[0].ticks_to_values(&leader),
            follower_ticks: follower,
            follower_values: cal[1].ticks_to_values(&follower),
            goal_ticks_sent: goals,
            goal_values_sent: cal[1].ticks_to_values(&goals),
            flags: b.flags,
            released: [false; COUNT],
            telemetry: b.telemetry,
            work_time: Duration::from_micros(b.end_us - b.start_us),
            error: (!complete).then(|| "incomplete tick in the log".to_string()),
        };
        // Each tick's frames come after the tick, as they do live.
        while let Some((_, w)) = wires.next_if(|(t, _)| *t <= b.end_us) {
            out.push(w);
        }
        out.push(tick_json(b.end_us, &report));
    }
    out.extend(wires.map(|(_, w)| w));
    out.push(serde_json::json!({"k": "end", "ticks": ticks.len(), "overruns": 0, "errors": errors}).to_string());
    out
}
