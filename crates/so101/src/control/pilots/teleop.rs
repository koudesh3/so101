//! Teleop: a `Pilot` that commands where the leader arm is, a little ahead of it to cancel the
//! follower's own response lag; in fine mode, a fraction of the leader's motion.

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use std::time::Duration;

use crate::arm::{Arm, Joint, JointState, Pose, Result, JOINT_COUNT};
use crate::clock::Timestamp;
use crate::control::{ArmRole, Message, Pilot, Source};

pub struct Teleop<L: Arm> {
    leader: L,
    /// Per joint: command where the leader will be this far ahead, from its measured velocity.
    /// Zero disables.
    lead: [Duration; JOINT_COUNT],
    predictor: Predictor,
    fine: Arc<FineControl>,
    scaling: Scaling,
    taps: DoubleTap,
}

// Two quick squeezes (or opens) of the leader's gripper: the fine-mode switch, so a hand on the
// leader can flip it. A tap leaves where the gripper rested by TAP_AWAY and comes back within
// TAP_HALF of it inside TAP_TIME; two taps inside PAIR_TIME make a double tap.
#[derive(Default)]
struct DoubleTap {
    rest: Option<f32>,
    // When the gripper left rest, if it is away now.
    away_since: Option<Timestamp>,
    // When the last complete tap began.
    last_tap: Option<Timestamp>,
    // A double tap's second release, until which nothing counts again.
    quiet_until: Option<Timestamp>,
}

impl DoubleTap {
    // Gripper percent.
    const TAP_AWAY: f32 = 10.0;
    const TAP_HALF: f32 = 5.0;
    const TAP_TIME: Duration = Duration::from_millis(400);
    const PAIR_TIME: Duration = Duration::from_millis(1200);
    const QUIET: Duration = Duration::from_millis(500);

    /// True on the sample that completes a double tap.
    fn update(&mut self, now: Timestamp, grip: f32) -> bool {
        if self.quiet_until.is_some_and(|until| now.since(until) == Duration::ZERO) {
            self.rest = Some(grip);
            return false;
        }
        let rest = *self.rest.get_or_insert(grip);
        let off = (grip - rest).abs();
        match self.away_since {
            None if off >= Self::TAP_AWAY => self.away_since = Some(now),
            None => {
                // Drifting slowly (or holding a new grip) moves where it rests.
                if off < Self::TAP_HALF {
                    self.rest = Some(rest + 0.05 * (grip - rest));
                } else {
                    self.rest = Some(grip);
                }
            }
            Some(left) if off <= Self::TAP_HALF => {
                self.away_since = None;
                if now.since(left) <= Self::TAP_TIME {
                    if self.last_tap.is_some_and(|first| now.since(first) <= Self::PAIR_TIME) {
                        self.last_tap = None;
                        self.quiet_until = Some(now + Self::QUIET);
                        return true;
                    }
                    self.last_tap = Some(left);
                } else {
                    self.last_tap = None;
                }
            }
            Some(left) => {
                if now.since(left) > Self::TAP_TIME {
                    // Held away: a grasp, not a tap; it rests here now.
                    self.away_since = None;
                    self.last_tap = None;
                    self.rest = Some(grip);
                }
            }
        }
        false
    }
}

/// Fine mode's switch and scale, shared with whatever flips it from outside the loop.
#[derive(Debug)]
pub struct FineControl {
    on: AtomicBool,
    scale: AtomicU32,
}

impl FineControl {
    pub const DEFAULT_SCALE: f32 = 1.0 / 3.0;
    const SCALES: std::ops::RangeInclusive<f32> = 0.05..=1.0;

    pub fn is_on(&self) -> bool {
        self.on.load(Ordering::Relaxed)
    }

    pub fn set_on(&self, on: bool) {
        self.on.store(on, Ordering::Relaxed);
    }

    /// Follower motion per unit of leader motion while fine mode is on.
    pub fn scale(&self) -> f32 {
        f32::from_bits(self.scale.load(Ordering::Relaxed))
    }

    /// Clamped to 0.05..=1.
    pub fn set_scale(&self, scale: f32) {
        let scale =
            if scale.is_nan() { Self::DEFAULT_SCALE } else { scale.clamp(*Self::SCALES.start(), *Self::SCALES.end()) };
        self.scale.store(scale.to_bits(), Ordering::Relaxed);
    }
}

impl Default for FineControl {
    fn default() -> Self {
        Self { on: AtomicBool::new(false), scale: AtomicU32::new(Self::DEFAULT_SCALE.to_bits()) }
    }
}

// Leader pose -> target. In fine mode the arm's joints move `scale` times the leader's motion from
// where both were when it began; the gripper stays 1:1, or a scaled grip can't close from where
// it started. Leaving fine mode keeps the target where it is; the leftover offset then shrinks as
// the leader moves, so nothing moves on its own.
#[derive(Default)]
struct Scaling {
    // (scale, leader, target) when fine mode began or its scale last changed.
    anchor: Option<(f32, Pose, Pose)>,
    offset: [f32; JOINT_COUNT],
    last: Option<(Pose, Pose)>,
}

impl Scaling {
    // Leader motion that goes to closing the offset, per unit moved: the follower moves at 0.5x
    // or 1.5x the leader until the arms line up again.
    const CATCH_UP: f32 = 0.5;
    const UNSCALED: Joint = Joint::Gripper;

    fn target(&mut self, leader: Pose, fine: Option<f32>) -> Pose {
        let last_target = self.last.map_or(leader, |(_, t)| t);
        if let Some((prev, _)) = self.last {
            for i in 0..JOINT_COUNT {
                let closes = (leader.0[i] - prev.0[i]).abs() * Self::CATCH_UP;
                self.offset[i] -= self.offset[i].clamp(-closes, closes);
            }
        }
        let followed = |offset: &[f32; JOINT_COUNT], i: usize| leader.0[i] + offset[i];
        let target = match fine {
            Some(scale) => {
                if !matches!(self.anchor, Some((s, _, _)) if s == scale) {
                    self.anchor = Some((scale, leader, last_target));
                }
                let (_, l0, t0) = self.anchor.expect("set above");
                Pose(std::array::from_fn(|i| {
                    if i == Self::UNSCALED.index() {
                        followed(&self.offset, i)
                    } else {
                        t0.0[i] + scale * (leader.0[i] - l0.0[i])
                    }
                }))
            }
            None => {
                if self.anchor.take().is_some() {
                    self.offset = std::array::from_fn(|i| last_target.0[i] - leader.0[i]);
                }
                Pose(std::array::from_fn(|i| followed(&self.offset, i)))
            }
        };
        self.last = Some((leader, target));
        target
    }
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

impl<L: Arm> Teleop<L> {
    /// Turns the leader's torque off, so a hand can move it.
    pub fn new(mut leader: L, lead: [Duration; JOINT_COUNT]) -> Result<Self> {
        leader.torque_off()?;
        Ok(Self {
            leader,
            lead,
            predictor: Predictor::default(),
            fine: Arc::default(),
            scaling: Scaling::default(),
            taps: DoubleTap::default(),
        })
    }

    /// The fine-mode switch, for flipping from another thread while the loop runs.
    pub fn fine_control(&self) -> Arc<FineControl> {
        self.fine.clone()
    }
}

impl<L: Arm> Pilot for Teleop<L> {
    fn source(&self) -> Source {
        Source::Teleop
    }

    fn target(&mut self, now: Timestamp, _follower: &JointState) -> Result<(Pose, Vec<Message>)> {
        let leader = self.leader.read_state()?;
        if self.taps.update(now, leader.pose.0[Joint::Gripper.index()]) {
            self.fine.set_on(!self.fine.is_on());
        }
        let ahead = self.predictor.ahead(now, leader.pose, self.lead);
        let target = self.scaling.target(ahead, self.fine.is_on().then(|| self.fine.scale()));
        Ok((target, vec![Message::JointState(ArmRole::Leader, leader)]))
    }
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

    fn pan(deg: f32) -> Pose {
        let mut pose = Pose::default();
        pose.0[0] = deg;
        pose
    }

    #[test]
    fn fine_mode_moves_a_fraction_of_the_leader_from_where_it_began() {
        let mut s = Scaling::default();
        assert_eq!(s.target(pan(10.0), None), pan(10.0));
        assert_eq!(s.target(pan(10.0), Some(0.25)), pan(10.0), "switching on doesn't move");
        assert_eq!(s.target(pan(18.0), Some(0.25)), pan(12.0));
        assert_eq!(s.target(pan(2.0), Some(0.25)), pan(8.0));
    }

    #[test]
    fn changing_the_scale_in_fine_mode_doesnt_jump() {
        let mut s = Scaling::default();
        s.target(pan(0.0), Some(0.5));
        assert_eq!(s.target(pan(10.0), Some(0.5)), pan(5.0));
        assert_eq!(s.target(pan(10.0), Some(0.25)), pan(5.0));
        assert_eq!(s.target(pan(14.0), Some(0.25)), pan(6.0));
    }

    #[test]
    fn leaving_fine_mode_holds_then_catches_up_as_the_leader_moves() {
        let mut s = Scaling::default();
        s.target(pan(0.0), Some(0.5));
        assert_eq!(s.target(pan(20.0), Some(0.5)), pan(10.0));
        assert_eq!(s.target(pan(20.0), None), pan(10.0), "switching off doesn't move");
        assert_eq!(s.target(pan(20.0), None), pan(10.0), "nor does waiting");
        // Offset 10 behind; moving the leader 4 closes 2 of it.
        assert_eq!(s.target(pan(24.0), None), pan(16.0));
        assert_eq!(s.target(pan(14.0), None), pan(11.0), "closing in either direction");
        for _ in 0..20 {
            s.target(pan(30.0), None);
            s.target(pan(0.0), None);
        }
        assert_eq!(s.target(pan(5.0), None), pan(5.0), "lined up again");
    }

    #[test]
    fn the_gripper_stays_one_to_one_in_fine_mode() {
        let grip = |deg: f32| {
            let mut pose = pan(10.0);
            pose.0[Joint::Gripper.index()] = deg;
            pose
        };
        let mut s = Scaling::default();
        s.target(grip(60.0), Some(0.25));
        assert_eq!(s.target(grip(0.0), Some(0.25)), grip(0.0), "closes all the way");
    }

    // Gripper samples every 10 ms from `start` ms; returns the times (ms) a double tap completed.
    fn taps(start: u64, grip: &[f32]) -> Vec<u64> {
        let mut d = DoubleTap::default();
        (0..grip.len())
            .filter(|&i| d.update(Timestamp(0) + Duration::from_millis(start + 10 * i as u64), grip[i]))
            .map(|i| start + 10 * i as u64)
            .collect()
    }

    // `ms` milliseconds at `v`, sampled every 10 ms.
    fn hold(v: f32, ms: usize) -> Vec<f32> {
        vec![v; ms / 10]
    }

    #[test]
    fn two_quick_squeezes_are_a_double_tap() {
        let g = [hold(40.0, 300), hold(20.0, 150), hold(40.0, 150), hold(20.0, 150), hold(40.0, 300)].concat();
        assert_eq!(taps(0, &g), vec![750]);
    }

    #[test]
    fn opening_twice_counts_too() {
        let g = [hold(40.0, 300), hold(60.0, 150), hold(40.0, 150), hold(60.0, 150), hold(40.0, 300)].concat();
        assert_eq!(taps(0, &g).len(), 1);
    }

    #[test]
    fn a_grasp_and_release_is_not_a_tap() {
        // Close on something for a second, open, close again quickly: only one quick tap.
        let g = [hold(40.0, 300), hold(15.0, 1000), hold(40.0, 150), hold(20.0, 150), hold(40.0, 300)].concat();
        assert!(taps(0, &g).is_empty());
    }

    #[test]
    fn small_wiggles_and_slow_taps_are_not_taps() {
        let wiggle = [hold(40.0, 300), hold(34.0, 100), hold(40.0, 100), hold(34.0, 100), hold(40.0, 300)].concat();
        assert!(taps(0, &wiggle).is_empty());
        let slow = [hold(40.0, 300), hold(20.0, 600), hold(40.0, 100), hold(20.0, 600), hold(40.0, 300)].concat();
        assert!(taps(0, &slow).is_empty());
    }

    #[test]
    fn taps_too_far_apart_are_not_a_pair() {
        let g = [hold(40.0, 300), hold(20.0, 150), hold(40.0, 1500), hold(20.0, 150), hold(40.0, 300)].concat();
        assert!(taps(0, &g).is_empty());
    }

    #[test]
    fn a_triple_tap_toggles_once() {
        let tap = [hold(20.0, 150), hold(40.0, 150)].concat();
        let g = [hold(40.0, 300), tap.clone(), tap.clone(), tap, hold(40.0, 300)].concat();
        assert_eq!(taps(0, &g).len(), 1);
    }

    #[test]
    fn fine_scale_is_clamped() {
        let fine = FineControl::default();
        assert!((fine.scale() - FineControl::DEFAULT_SCALE).abs() < 1e-6);
        fine.set_scale(0.0);
        assert_eq!(fine.scale(), 0.05);
        fine.set_scale(3.0);
        assert_eq!(fine.scale(), 1.0);
    }

    #[test]
    fn zero_lead_passes_the_pose_through() {
        let (pose, ahead) = predict(100.0, 50, [Duration::ZERO; JOINT_COUNT]);
        assert_eq!(ahead, pose);
    }
}
