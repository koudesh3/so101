//! `ServoArm`: the SO-101 on its STS3215 servos, the real `Arm`. Which servo is which joint, the
//! follower's gains, reading and writing through `Servos`, decoding actuator status. Positions here
//! are in the calibrated coordinate; it's the only code above `servos` that names a register.

use std::sync::Arc;
use std::time::Duration;

use super::calibration::Calibration;
use super::clearance::ClearanceMap;
use super::servos::{self, ServoError, Servos};
use super::{ActuatorStatus, Arm, Counts, Error, Fault, Joint, JointState, Limit, Pose, Result, Sent, JOINT_COUNT};
use crate::clock::{Clock, MonotonicClock, Timestamp};

/// Servo IDs, in `Joint::ALL` order.
pub const IDS: [u8; JOINT_COUNT] = [1, 2, 3, 4, 5, 6];

// Present_Position (56) through Present_Current (69..70): one 15-byte read.
pub const STATUS_BLOCK_START: u8 = 56;

pub const STATUS_BLOCK_LEN: u8 = 15;

// The follower's position-loop gains. LeRobot halves P from the factory 32 to
// 16 to avoid shake; higher P is stiffer and quicker to close a gap.
#[derive(Clone, Copy, Debug)]
pub struct FollowerGains {
    pub p: i32,
    pub d: i32,
    // Drive ceiling per joint, 0..1000 = 0..100%, written to Max_Torque (EEPROM, so it survives a
    // brownout reboot) and Torque_Limit. It caps force and, with it, top speed: the gripper at 500
    // measured ~1600 counts/s against ~3100 at full drive. Stall protection does not depend on it.
    pub max_torque: [i32; JOINT_COUNT],
}

pub struct ServoArm {
    label: String,
    servos: Servos,
    calibration: Calibration,
    clock: Arc<dyn Clock>,
    protection: Protection,
    step_limit: Option<i32>,
    power: PowerGuard,
    camera_guard: Option<(ClearanceMap, f32)>,
}

impl Joint {
    pub fn id(self) -> u8 {
        IDS[self.index()]
    }

    pub fn from_id(id: u8) -> Option<Joint> {
        Joint::ALL.iter().copied().find(|j| j.id() == id)
    }
}

impl FollowerGains {
    pub const LEROBOT: FollowerGains = FollowerGains { p: 16, d: 32, max_torque: [1000, 1000, 1000, 1000, 1000, 500] };
    /// About 1.2x the most each joint used in Hanoi moves at step limit 20 (2026-11-23: pan 74%,
    /// lift 66%, elbow 54%, wrist flex 66%, roll 59%; holding never above 9%). Normal moves are
    /// unchanged; a jam or collision can't draw full current.
    pub const MEASURED_MAX_TORQUE: [i32; JOINT_COUNT] = [900, 800, 700, 800, 700, 500];
}

impl ServoArm {
    pub fn new(label: &str, servos: Servos, calibration: Calibration) -> Self {
        Self {
            label: label.to_string(),
            servos,
            calibration,
            clock: Arc::new(MonotonicClock),
            protection: Protection::default(),
            step_limit: None,
            power: PowerGuard::default(),
            camera_guard: None,
        }
    }

    /// Stamps readings on `clock` instead of the host's monotonic clock.
    pub fn with_clock(mut self, clock: Arc<dyn Clock>) -> Self {
        self.clock = clock;
        self
    }

    /// The most a goal may move per write, in counts (LeRobot's `max_relative_target`). `None`
    /// disables it.
    pub fn set_step_limit(&mut self, max_counts: Option<i32>) {
        self.step_limit = max_counts;
    }

    /// Keep each goal's wrist camera at least `margin_mm` from the arm, by `map`. A goal that would
    /// come closer, and closer than the last goal was, keeps the map's joints where they were.
    pub fn set_camera_guard(&mut self, map: ClearanceMap, margin_mm: f32) {
        self.camera_guard = Some((map, margin_mm));
    }

    /// Whether the follower's total current has been over budget, so goals move at most
    /// `GUARD_STEP_COUNTS` per write. Needs actuator reads (`read_state_and_actuators`).
    pub fn current_guard_active(&self) -> bool {
        self.power.active
    }

    pub fn clock(&self) -> Arc<dyn Clock> {
        self.clock.clone()
    }

    pub fn label(&self) -> &str {
        &self.label
    }

    pub fn port_name(&self) -> &str {
        self.servos.name()
    }

    pub fn calibration(&self) -> &Calibration {
        &self.calibration
    }

    pub fn check_present(&mut self) -> Result<()> {
        for joint in Joint::ALL {
            let identity = self.servos.identify(joint.id())?;
            if !identity.is_sts3215() {
                return Err(Error::UnexpectedModel { id: joint.id(), model: identity.model });
            }
        }
        Ok(())
    }

    // What LeRobot writes to every arm at connect: position readings that never
    // wrap. Returns how many servos had the wrap bit set and were changed.
    pub fn configure_readings(&mut self) -> Result<usize> {
        let mut changed = 0;
        for id in IDS {
            let phase = self.servos.read(id, servos::PHASE)?;
            if phase & servos::PHASE_ANGLE_FEEDBACK != 0 {
                self.servos.write_eeprom(id, servos::PHASE, phase & !servos::PHASE_ANGLE_FEEDBACK)?;
                changed += 1;
            }
        }
        Ok(changed)
    }

    pub fn configure_follower(&mut self) -> Result<()> {
        self.configure_follower_with(FollowerGains::LEROBOT)
    }

    // The register set LeRobot writes to a follower at connect time.
    pub fn configure_follower_with(&mut self, gains: FollowerGains) -> Result<()> {
        for id in IDS {
            self.servos.write(id, servos::TORQUE_ENABLE, 0)?;
            self.servos.write_eeprom(id, servos::RETURN_DELAY, 0)?;
            self.servos.write_eeprom(id, servos::OPERATING_MODE, 0)?;
            self.servos.write_eeprom(id, servos::P_COEFFICIENT, gains.p)?;
            self.servos.write_eeprom(id, servos::I_COEFFICIENT, 0)?;
            self.servos.write_eeprom(id, servos::D_COEFFICIENT, gains.d)?;
            self.servos.write_unlocked(id, servos::MAXIMUM_ACCELERATION, 254)?;
            self.servos.write(id, servos::ACCELERATION, 254)?;
        }
        for (id, max_torque) in IDS.into_iter().zip(gains.max_torque) {
            self.servos.write_eeprom(id, servos::MAX_TORQUE, max_torque)?;
            // The RAM copy, in case Max_Torque only loads at power-on.
            self.servos.write(id, servos::TORQUE_LIMIT, max_torque)?;
        }
        // Gripper burnout guard, LeRobot's values. Closing the leader trigger
        // drives the claw into itself; without these it stalls at full torque.
        let grip = Joint::Gripper.id();
        self.servos.write_eeprom(grip, servos::PROTECTION_CURRENT, 250)?; // factory default 0 = off
        self.servos.write_eeprom(grip, servos::OVERLOAD_TORQUE, 25)?; // overload trips at 25%
        Ok(())
    }

    pub fn torque_off(&mut self) -> Result<()> {
        self.servos.sync_write(servos::TORQUE_ENABLE, &IDS.map(|id| (id, 0)))
    }

    // Seeds every goal with the present position first so nothing jumps.
    pub fn torque_on_holding(&mut self) -> Result<()> {
        let present = self.read_counts()?;
        self.servos.sync_write(servos::GOAL_POSITION, &pairs(present))?;
        self.servos.sync_write(servos::TORQUE_ENABLE, &IDS.map(|id| (id, 1)))?;
        self.await_every_servo()
    }

    // note: a servo can miss the first read after its torque comes on. Servo 1 did on hardware
    // (2026-11-23, several runs): the first cycle's sync read, 0.8 ms after torque-on, got no
    // reply from it, so servo 2's reply read as the wrong servo. Read until all six answer.
    fn await_every_servo(&mut self) -> Result<()> {
        let deadline = self.clock.now() + TORQUE_ON_SETTLE;
        loop {
            match self.read_counts() {
                Ok(_) => return Ok(()),
                Err(e) if self.clock.now() >= deadline => return Err(e),
                Err(_) => self.clock.sleep_until(self.clock.now() + Duration::from_millis(2)),
            }
        }
    }

    // Declare the current pose to be centre. Returns the Homing_Offset each servo chose.
    pub fn recentre(&mut self) -> Result<Counts> {
        self.torque_off()?;
        let mut offsets = [0; JOINT_COUNT];
        for (i, id) in IDS.iter().enumerate() {
            self.servos.write(*id, servos::TORQUE_ENABLE, servos::TORQUE_RECENTER)?;
            offsets[i] = self.read_after_eeprom_write(*id, servos::HOMING_OFFSET)?;
        }
        Ok(offsets)
    }

    // note: after Torque_Enable = 128 the servo stores its new offset in EEPROM
    // and can stay silent for several ms. Retry a silent read for up to ~200 ms;
    // any other error returns at once.
    fn read_after_eeprom_write(&mut self, id: u8, reg: servos::Register) -> Result<i32> {
        let mut last = Error::NoReply { id };
        for _ in 0..20 {
            std::thread::sleep(Duration::from_millis(10));
            match self.servos.read(id, reg) {
                Ok(v) => return Ok(v),
                Err(e @ Error::NoReply { .. }) => last = e,
                Err(e) => return Err(e),
            }
        }
        Err(last)
    }

    pub fn read_counts(&mut self) -> Result<Counts> {
        let v = self.servos.sync_read(&IDS, servos::PRESENT_POSITION)?;
        Ok(std::array::from_fn(|i| v[i]))
    }

    pub fn read_pose(&mut self) -> Result<Pose> {
        let counts = self.read_counts()?;
        Ok(self.calibration.to_pose(&counts))
    }

    pub fn to_pose(&self, counts: &Counts) -> Pose {
        self.calibration.to_pose(counts)
    }

    pub fn to_counts(&self, pose: &Pose) -> Counts {
        self.calibration.to_counts(pose)
    }

    pub fn write_goal(&mut self, goal: &Counts) -> Result<()> {
        self.servos.sync_write(servos::GOAL_POSITION, &pairs(*goal))
    }

    pub fn read_actuators(&mut self) -> Result<Vec<ActuatorStatus>> {
        Ok(self.read_state_and_actuators()?.1)
    }

    /// One block read covers every field.
    pub fn read_actuator(&mut self, joint: Joint) -> Result<ActuatorStatus> {
        let block = self.servos.read_block(joint.id(), STATUS_BLOCK_START, STATUS_BLOCK_LEN)?;
        decode_actuator(joint, self.clock.now(), &block)
            .ok_or_else(|| Error::Corrupt(format!("short status block from servo {}", joint.id())))
    }

    pub fn take_flags(&mut self) -> [ServoError; JOINT_COUNT] {
        let mut out = [ServoError(0); JOINT_COUNT];
        for (id, e) in self.servos.take_flags() {
            if let Some(j) = Joint::from_id(id) {
                out[j.index()].0 |= e.0;
            }
        }
        out
    }

    // Raw register access for tests and diagnostics.
    pub fn read_register(&mut self, joint: Joint, reg: servos::Register) -> Result<i32> {
        self.servos.read(joint.id(), reg)
    }
}

impl Arm for ServoArm {
    fn read_state(&mut self) -> Result<JointState> {
        let counts = self.read_counts()?;
        self.protection.present = Some(counts);
        Ok(JointState { t: self.clock.now(), pose: self.to_pose(&counts), counts: Some(counts) })
    }

    fn command(&mut self, target: &Pose) -> Result<Sent> {
        let present = match self.protection.present.take() {
            Some(counts) => counts,
            None => self.read_counts()?,
        };
        let faults = self.take_flags().map(faults);
        let in_range = self.calibration.to_counts(target);
        let range_pose = self.calibration.to_pose(&in_range);
        let now = self.clock.now();
        let step_limit = if self.power.active {
            Some(self.step_limit.map_or(GUARD_STEP_COUNTS, |s| s.min(GUARD_STEP_COUNTS)))
        } else {
            self.step_limit
        };
        let previous = self.protection.last_goal;
        let (mut sent, held, stepped) = self.protection.apply(present, &faults, in_range, now, step_limit);
        let mut kept = [false; JOINT_COUNT];
        if let (Some((map, margin_mm)), Some(previous)) = (&self.camera_guard, previous) {
            let clearance = |counts: &Counts| map.clearance_mm(&self.calibration.to_pose(counts));
            let next = clearance(&sent);
            if next < *margin_mm && next < clearance(&previous) {
                for joint in map.joints() {
                    let i = joint.index();
                    kept[i] = sent[i] != previous[i];
                    sent[i] = previous[i];
                }
                self.protection.last_goal = Some(sent);
            }
        }
        self.write_goal(&sent)?;
        let limits = std::array::from_fn(|i| {
            if held[i] {
                Some(Limit::Stall)
            } else if kept[i] {
                Some(Limit::Collision)
            } else if stepped[i] {
                Some(Limit::Step)
            } else if (range_pose.0[i] - target.0[i]).abs() > ONE_COUNT_DEG {
                Some(Limit::Range)
            } else {
                None
            }
        });
        Ok(Sent { pose: self.to_pose(&sent), counts: Some(sent), limits })
    }

    /// One sync read of the status block, which starts at the position: about 0.8 ms more bus
    /// time per call than `read_state` at 1 Mbaud.
    fn read_state_and_actuators(&mut self) -> Result<(JointState, Vec<ActuatorStatus>)> {
        let blocks = self.servos.sync_read_block(&IDS, STATUS_BLOCK_START, STATUS_BLOCK_LEN)?;
        let t = self.clock.now();
        let mut counts = [0; JOINT_COUNT];
        let mut actuators = Vec::with_capacity(JOINT_COUNT);
        for (joint, block) in Joint::ALL.into_iter().zip(&blocks) {
            let status = decode_actuator(joint, t, block)
                .ok_or_else(|| Error::Corrupt(format!("short status block from servo {}", joint.id())))?;
            counts[joint.index()] = servos::PRESENT_POSITION.decode(block);
            actuators.push(status);
        }
        self.protection.present = Some(counts);
        self.power.update(actuators.iter().filter_map(|a| a.current_a).sum(), t);
        Ok((JointState { t, pose: self.to_pose(&counts), counts: Some(counts) }, actuators))
    }

    fn torque_off(&mut self) -> Result<()> {
        ServoArm::torque_off(self)
    }

    fn hold(&mut self) -> Result<()> {
        self.torque_on_holding()?;
        // Faults raised before now (a trip at the end of the last run, say) describe nothing
        // this run did. Drop them so they cannot latch a hold.
        let _ = self.take_flags();
        self.protection = Protection::default();
        self.power = PowerGuard::default();
        Ok(())
    }
}

fn pairs(counts: Counts) -> [(u8, i32); JOINT_COUNT] {
    std::array::from_fn(|i| (IDS[i], counts[i]))
}

fn decode_actuator(joint: Joint, t: Timestamp, block: &[u8]) -> Option<ActuatorStatus> {
    if block.len() < STATUS_BLOCK_LEN as usize {
        return None;
    }
    let at = |r: servos::Register| r.decode(&block[(r.address - STATUS_BLOCK_START) as usize..]);
    let status = ServoError(at(servos::STATUS) as u8);
    Some(ActuatorStatus {
        t,
        joint,
        velocity_deg_s: Some(at(servos::PRESENT_SPEED) as f32 * 360.0 / servos::COUNTS_PER_TURN as f32),
        // The register is in tenths of a percent.
        load_percent: Some(at(servos::PRESENT_LOAD) as f32 / 10.0),
        voltage_v: Some(at(servos::PRESENT_VOLTAGE) as f32 / 10.0),
        temperature_c: Some(at(servos::PRESENT_TEMPERATURE) as f32),
        // 6.5 mA per unit.
        current_a: Some(at(servos::PRESENT_CURRENT) as f32 * 0.0065),
        faults: faults(status),
    })
}

pub(crate) fn faults(status: ServoError) -> Vec<Fault> {
    [
        (ServoError::VOLTAGE, Fault::Voltage),
        (ServoError::ANGLE_SENSOR, Fault::Sensor),
        (ServoError::OVERHEAT, Fault::Overheat),
        (ServoError::OVERCURRENT, Fault::Overcurrent),
        (ServoError::ANGLE_LIMIT, Fault::AngleLimit),
        (ServoError::OVERLOAD, Fault::Overload),
    ]
    .into_iter()
    .filter(|(bit, _)| status.has(*bit))
    .map(|(_, f)| f)
    .collect()
}

// A joint is blocked when its goal is this far away...
const STALL_GAP_COUNTS: i32 = 60;
// ...and it has moved no more than this per write...
const STALL_STILL_COUNTS: i32 = 2;
// ...for this long. Faster than the servo's own protection, which let the
// gripper squeeze at its full ceiling for ~2 s on hardware before tripping.
const STALL_TIME: Duration = Duration::from_millis(250);
// While held, the gripper keeps this much of a squeeze past the contact
// point: a chosen grip force instead of the servo's maximum. Other joints hold.
const GRIP_SQUEEZE_COUNTS: i32 = 30;
// One count, in the pose's units: a commanded value further than this from where the range clamp
// put it was clipped by the range.
const ONE_COUNT_DEG: f32 = 360.0 / 4096.0;
// The follower's 12 V 2 A supply browned out once (2026-11-23). Measured Hanoi moves at step limit
// 20 drew 0.58 A at the 99.9th percentile and 1.25 A at most, so this leaves room for normal work.
const CURRENT_BUDGET_A: f32 = 1.5;
// Over budget this long engages the guard; back under it this long releases it.
const GUARD_AFTER: Duration = Duration::from_millis(50);
const GUARD_RELEASE_AFTER: Duration = Duration::from_millis(250);
// The step limit while the guard is on: a quarter of the default.
const GUARD_STEP_COUNTS: i32 = 10;
// How long torque-on waits for every servo to answer a read before giving up.
const TORQUE_ON_SETTLE: Duration = Duration::from_millis(100);

/// The follower's per-write protections, in the counts they were tuned in on hardware: the stall
/// hold, then the step limit. The range clamp happens before, in the calibration.
#[derive(Default)]
struct Protection {
    present: Option<Counts>,
    last_goal: Option<Counts>,
    last_write: Option<Timestamp>,
    holds: [Option<Hold>; JOINT_COUNT],
    stalls: StallWatch,
}

/// Tightens the step limit while the follower's summed current stays over `CURRENT_BUDGET_A`.
#[derive(Default)]
struct PowerGuard {
    active: bool,
    // When the total last crossed to the other side of the budget from `active`.
    crossed_at: Option<Timestamp>,
}

#[derive(Default)]
struct StallWatch {
    prev: Option<Counts>,
    stuck_s: [f64; JOINT_COUNT],
    // Whether the servo's protection flag was up last write. The flag stays up
    // for as long as the servo is in protection, so only its rising edge is a
    // new event; a flag still up from a hold says nothing about the next move.
    flagged: [bool; JOINT_COUNT],
}

// A blocked joint (stall seen, or the servo tripped its overload or current protection): its
// goal is held at the contact point, plus a small squeeze on the gripper, until the commanded
// pose moves back past that point.
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

impl PowerGuard {
    fn update(&mut self, total_a: f32, now: Timestamp) {
        let wants = total_a > CURRENT_BUDGET_A;
        if wants == self.active {
            self.crossed_at = None;
            return;
        }
        let since = *self.crossed_at.get_or_insert(now);
        let wait = if self.active { GUARD_RELEASE_AFTER } else { GUARD_AFTER };
        if now.since(since) >= wait {
            self.active = wants;
            self.crossed_at = None;
        }
    }
}

impl Protection {
    /// The goal to write, which joints are held, and which the step limit moved.
    fn apply(
        &mut self,
        present: Counts,
        faults: &[Vec<Fault>; JOINT_COUNT],
        mut goal: Counts,
        now: Timestamp,
        step_limit: Option<i32>,
    ) -> (Counts, [bool; JOINT_COUNT], [bool; JOINT_COUNT]) {
        let dt = self.last_write.map_or(0.0, |t| now.since(t).as_secs_f64());
        self.last_write = Some(now);
        let (holds, stalls) = (&mut self.holds, &mut self.stalls);
        for i in 0..JOINT_COUNT {
            // The goal that drove it into the stall is the previous one, not this
            // write's: the command may already be backing off on the write it trips.
            // No previous goal means nothing this run has pushed yet: no hold.
            let gap = self.last_goal.map_or(0, |g| g[i] - present[i]);
            let pushing_hard = gap.abs() > STALL_GAP_COUNTS;
            let step = stalls.prev.map(|p| present[i] - p[i]);
            let moved = step.map_or(i32::MAX, i32::abs);
            stalls.stuck_s[i] = if pushing_hard && moved <= STALL_STILL_COUNTS { stalls.stuck_s[i] + dt } else { 0.0 };
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
                holds[i] = Some(Hold { at: present[i], pushing: Direction::of(gap) });
            }
            if let Some(h) = holds[i] {
                if Direction::of(goal[i] - h.at) != h.pushing || h.pushing == Direction::Neither {
                    holds[i] = None; // the command backed off: follow again
                    stalls.stuck_s[i] = 0.0;
                } else {
                    let squeeze = if Joint::ALL[i] == Joint::Gripper { GRIP_SQUEEZE_COUNTS } else { 0 };
                    goal[i] = h.at + h.pushing.sign() * squeeze;
                }
            }
        }
        stalls.prev = Some(present);
        // The step limit is a slew limit on the command: each goal may move at most
        // `max_step` from the previous goal (seeded from the present position, so
        // nothing jumps at start). Limiting it against the present position instead
        // keeps the servo's error, and so its effort and speed, pinned at `max_step`:
        // measured on hardware, ~37 deg/s against a leader at ~140 deg/s, and up to
        // 1.5 s of lag.
        let from = self.last_goal.unwrap_or(present);
        let sent: Counts = match step_limit {
            Some(max) => std::array::from_fn(|i| from[i] + (goal[i] - from[i]).clamp(-max, max)),
            None => goal,
        };
        self.last_goal = Some(sent);
        let held = std::array::from_fn(|i| self.holds[i].is_some());
        let stepped = std::array::from_fn(|i| sent[i] != goal[i]);
        (sent, held, stepped)
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_current_guard_engages_after_50_ms_over_budget_and_releases_after_250_ms_under() {
        let mut guard = PowerGuard::default();
        let at = |ms: u64| Timestamp(ms * 1_000_000);
        guard.update(1.9, at(0));
        guard.update(1.9, at(40));
        assert!(!guard.active, "a 40 ms spike is ignored");
        guard.update(0.4, at(45));
        guard.update(1.9, at(50));
        guard.update(1.9, at(99));
        assert!(!guard.active, "the dip restarted the wait");
        guard.update(1.9, at(100));
        assert!(guard.active);
        guard.update(0.4, at(110));
        guard.update(0.4, at(300));
        assert!(guard.active, "still within the release wait");
        guard.update(0.4, at(360));
        assert!(!guard.active);
    }

    #[test]
    fn direction_follows_the_sign_of_the_gap() {
        assert_eq!(Direction::of(60), Direction::Up);
        assert_eq!(Direction::of(-1), Direction::Down);
        assert_eq!(Direction::of(0), Direction::Neither);
        assert_eq!(Direction::Down.sign(), -1);
    }

    #[test]
    fn ids_round_trip() {
        for (i, joint) in Joint::ALL.iter().enumerate() {
            assert_eq!(joint.id(), i as u8 + 1);
            assert_eq!(Joint::from_id(joint.id()), Some(*joint));
        }
        assert_eq!(Joint::from_id(0), None);
        assert_eq!(Joint::from_id(7), None);
    }
}
