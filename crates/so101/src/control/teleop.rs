//! Teleop: a `Pilot` that commands where the leader arm is, a little ahead of it to cancel the
//! follower's own response lag.

use std::time::Duration;

use crate::arm::{Arm, JointState, Pose, Result, Source, JOINT_COUNT};
use crate::clock::Timestamp;
use crate::control::{ArmRole, Message, Pilot};

pub struct Teleop<L: Arm> {
    leader: L,
    /// Per joint: command where the leader will be this far ahead, from its measured velocity.
    /// Zero disables.
    lead: [Duration; JOINT_COUNT],
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

impl<L: Arm> Teleop<L> {
    /// Turns the leader's torque off, so a hand can move it.
    pub fn new(mut leader: L, lead: [Duration; JOINT_COUNT]) -> Result<Self> {
        leader.torque_off()?;
        Ok(Self { leader, lead, predictor: Predictor::default() })
    }
}

impl<L: Arm> Pilot for Teleop<L> {
    fn source(&self) -> Source {
        Source::Teleop
    }

    fn target(&mut self, now: Timestamp, _follower: &JointState) -> Result<(Pose, Vec<Message>)> {
        let leader = self.leader.read_state()?;
        let target = self.predictor.ahead(now, leader.pose, self.lead);
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

    #[test]
    fn zero_lead_passes_the_pose_through() {
        let (pose, ahead) = predict(100.0, 50, [Duration::ZERO; JOINT_COUNT]);
        assert_eq!(ahead, pose);
    }
}
