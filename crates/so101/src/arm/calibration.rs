//! Per-joint tick ranges and the conversion to the coordinate both arms
//! share: degrees from the midpoint of the calibrated range, or percent of
//! the range for the gripper.

use std::collections::BTreeMap;
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::arm::registers::{CENTER_COUNT, COUNTS_PER_TURN};
use crate::arm::servos::{Error, Result};
use crate::arm::{Arm, Counts, Joint, Pose, Unit, JOINT_COUNT};
use crate::control::grid::Grid;

pub const DEGREES_PER_COUNT: f64 = 360.0 / COUNTS_PER_TURN as f64;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointCalibration {
    // The Homing_Offset the servo chose when it was recentred.
    pub homing_offset: i32,
    pub range_min: i32,
    pub range_max: i32,
}

impl JointCalibration {
    pub const IDENTITY: JointCalibration =
        JointCalibration { homing_offset: 0, range_min: 0, range_max: COUNTS_PER_TURN - 1 };

    pub fn midpoint(&self) -> f64 {
        (self.range_min + self.range_max) as f64 / 2.0
    }

    pub fn span(&self) -> i32 {
        self.range_max - self.range_min
    }

    pub fn to_value(&self, unit: Unit, counts: i32) -> f64 {
        match unit {
            Unit::Degrees => (counts as f64 - self.midpoint()) * DEGREES_PER_COUNT,
            Unit::Percent => {
                let span = self.span().max(1) as f64;
                ((counts - self.range_min) as f64 / span * 100.0).clamp(0.0, 100.0)
            }
        }
    }

    // Always lands inside the calibrated range.
    pub fn to_count(&self, unit: Unit, value: f64) -> i32 {
        let counts = match unit {
            Unit::Degrees => (value / DEGREES_PER_COUNT + self.midpoint()).round() as i32,
            Unit::Percent => {
                (self.range_min as f64 + value.clamp(0.0, 100.0) / 100.0 * self.span() as f64).round() as i32
            }
        };
        counts.clamp(self.range_min, self.range_max)
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "CalibrationFile", into = "CalibrationFile")]
pub struct Calibration {
    pub arm_id: String,
    pub joints: [JointCalibration; JOINT_COUNT],
}

impl Calibration {
    // Raw counts, centred at 2048, full turn. What the simulator and an
    // uncalibrated monitor use.
    pub fn identity(arm_id: &str) -> Self {
        Self { arm_id: arm_id.to_string(), joints: [JointCalibration::IDENTITY; JOINT_COUNT] }
    }

    pub fn from_sweep(arm_id: &str, homing_offsets: Counts, range_min: Counts, range_max: Counts) -> Self {
        let joints = std::array::from_fn(|i| JointCalibration {
            homing_offset: homing_offsets[i],
            range_min: range_min[i],
            range_max: range_max[i],
        });
        Self { arm_id: arm_id.to_string(), joints }
    }

    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path).map_err(|e| Error::Calibration(format!("{}: {e}", path.display())))?;
        serde_json::from_str(&text).map_err(|e| Error::Calibration(format!("{}: {e}", path.display())))
    }

    pub fn save(&self, path: &Path) -> Result<()> {
        let text = serde_json::to_string_pretty(self).map_err(|e| Error::Calibration(e.to_string()))?;
        std::fs::write(path, text).map_err(|e| Error::Calibration(format!("{}: {e}", path.display())))
    }

    pub fn joint(&self, joint: Joint) -> &JointCalibration {
        &self.joints[joint.index()]
    }

    pub fn to_pose(&self, counts: &Counts) -> Pose {
        Pose(std::array::from_fn(|i| {
            let joint = Joint::ALL[i];
            self.joint(joint).to_value(joint.unit(), counts[i]) as f32
        }))
    }

    /// Clamped to each joint's calibrated range.
    pub fn to_counts(&self, pose: &Pose) -> Counts {
        std::array::from_fn(|i| {
            let joint = Joint::ALL[i];
            self.joint(joint).to_count(joint.unit(), pose.0[i] as f64)
        })
    }

    pub fn check_travel(&self, min_span_counts: i32) -> Result<()> {
        let narrow: Vec<String> = Joint::ALL
            .iter()
            .map(|&j| (j, self.joint(j)))
            .filter(|(_, c)| c.span() < min_span_counts)
            .map(|(j, c)| format!("{} ({}..{})", j.name(), c.range_min, c.range_max))
            .collect();
        if narrow.is_empty() {
            Ok(())
        } else {
            Err(Error::Calibration(format!("too little travel in the sweep: {}", narrow.join(", "))))
        }
    }
}

// On-disk layout: joints keyed by name, each carrying its servo id.
#[derive(Serialize, Deserialize)]
struct CalibrationFile {
    arm_id: String,
    joints: BTreeMap<String, JointFile>,
}

#[derive(Serialize, Deserialize)]
struct JointFile {
    id: u8,
    homing_offset: i32,
    range_min: i32,
    range_max: i32,
}

impl From<Calibration> for CalibrationFile {
    fn from(cal: Calibration) -> Self {
        let joints = Joint::ALL
            .iter()
            .map(|&j| {
                let c = cal.joint(j);
                let entry = JointFile {
                    id: j.id(),
                    homing_offset: c.homing_offset,
                    range_min: c.range_min,
                    range_max: c.range_max,
                };
                (j.name().to_string(), entry)
            })
            .collect();
        Self { arm_id: cal.arm_id, joints }
    }
}

impl TryFrom<CalibrationFile> for Calibration {
    type Error = String;

    fn try_from(file: CalibrationFile) -> std::result::Result<Self, String> {
        let mut joints = [JointCalibration::IDENTITY; JOINT_COUNT];
        for joint in Joint::ALL {
            let entry = file.joints.get(joint.name()).ok_or_else(|| format!("missing joint {}", joint.name()))?;
            if entry.id != joint.id() {
                return Err(format!("{} has id {}, expected {}", joint.name(), entry.id, joint.id()));
            }
            if entry.range_min > entry.range_max {
                return Err(format!("{} range is inverted", joint.name()));
            }
            joints[joint.index()] = JointCalibration {
                homing_offset: entry.homing_offset,
                range_min: entry.range_min,
                range_max: entry.range_max,
            };
        }
        Ok(Self { arm_id: file.arm_id, joints })
    }
}

// The procedure that produces a calibration: sweep, then check the middle pose.

pub struct Sweep {
    pub min: Counts,
    pub max: Counts,
}

pub fn sweep(arm: &mut Arm, hz: f64, mut keep_going: impl FnMut(&Sweep) -> bool) -> Result<Sweep> {
    let mut sweep = Sweep { min: [i32::MAX; JOINT_COUNT], max: [i32::MIN; JOINT_COUNT] };
    let mut grid = Grid::new(hz);
    loop {
        grid.wait();
        // A voltage flag does not fail the read (Servos::take_flags), so hand-driven
        // sweeps on a marginal 5 V supply keep every sample.
        let counts = arm.read_counts()?;
        for (i, &t) in counts.iter().enumerate() {
            sweep.min[i] = sweep.min[i].min(t);
            sweep.max[i] = sweep.max[i].max(t);
        }
        if !keep_going(&sweep) {
            return Ok(sweep);
        }
    }
}

// The middle pose should land near the middle of each swept range. A joint whose
// recentred tick (2048) sits near one end, or whose reading wrapped past the
// encoder's zero, was held near a hard stop when Enter was pressed.
pub fn check_centred(sweep: &Sweep) -> Result<()> {
    let mut bad = Vec::new();
    for joint in Joint::ALL {
        if joint == Joint::WristRoll {
            continue; // turns freely; any pose is a middle
        }
        let (lo, hi) = (sweep.min[joint.index()], sweep.max[joint.index()]);
        if hi - lo > COUNTS_PER_TURN - 200 {
            bad.push(format!("{} wrapped past the encoder zero ({lo}..{hi})", joint.name()));
            continue;
        }
        let at = (CENTER_COUNT - lo) as f64 / (hi - lo).max(1) as f64;
        if !(0.2..=0.8).contains(&at) {
            bad.push(format!(
                "{} held at {:.0}% of its range, not near the middle ({lo}..{hi})",
                joint.name(),
                at * 100.0
            ));
        }
    }
    if bad.is_empty() {
        Ok(())
    } else {
        Err(Error::Calibration(format!("redo with a true middle pose: {}", bad.join("; "))))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arm::registers::CENTER_COUNT;

    fn cal() -> JointCalibration {
        JointCalibration { homing_offset: 0, range_min: 769, range_max: 3156 }
    }

    #[test]
    fn degrees_round_trip_inside_range() {
        let c = cal();
        for counts in [769, 1000, 1962, 2500, 3156] {
            let d = c.to_value(Unit::Degrees, counts);
            let back = c.to_count(Unit::Degrees, d);
            assert!((back - counts).abs() <= 1, "{counts} -> {d} -> {back}");
        }
        assert_eq!(c.to_value(Unit::Degrees, 1962), -0.5 * DEGREES_PER_COUNT);
    }

    #[test]
    fn to_count_clamps_to_range() {
        let c = cal();
        assert_eq!(c.to_count(Unit::Degrees, 500.0), 3156);
        assert_eq!(c.to_count(Unit::Degrees, -500.0), 769);
    }

    #[test]
    fn percent_round_trip() {
        let c = cal();
        assert_eq!(c.to_value(Unit::Percent, 769), 0.0);
        assert_eq!(c.to_value(Unit::Percent, 3156), 100.0);
        assert_eq!(c.to_count(Unit::Percent, 50.0), 1963);
    }

    #[test]
    fn identity_maps_centre_to_zero_degrees() {
        let cal = Calibration::identity("t");
        let pose = cal.to_pose(&[CENTER_COUNT; JOINT_COUNT]);
        assert!(pose[Joint::ElbowFlex].abs() < 0.1);
    }

    #[test]
    fn json_round_trip_keeps_the_named_file_layout() {
        let mut a = Calibration::identity("test");
        a.joints[Joint::ElbowFlex.index()].range_min = 100;
        let text = serde_json::to_string(&a).unwrap();
        assert!(text.contains("\"elbow_flex\""));
        assert!(text.contains("\"id\":3"));
        let b: Calibration = serde_json::from_str(&text).unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn file_with_a_missing_or_misnumbered_joint_is_rejected() {
        let mut text = serde_json::to_string(&Calibration::identity("t")).unwrap();
        text = text.replace("\"id\":3", "\"id\":4");
        assert!(serde_json::from_str::<Calibration>(&text).is_err());
        let text = r#"{"arm_id":"t","joints":{}}"#;
        assert!(serde_json::from_str::<Calibration>(text).is_err());
    }
}
