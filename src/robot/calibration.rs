//! Per-joint tick ranges and the conversion to the coordinate both arms
//! share: degrees from the midpoint of the calibrated range, or percent of
//! the range for the gripper.

use std::collections::BTreeMap;
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::error::{Error, Result};
use crate::robot::joint::{Joint, Ticks, Unit, Values, COUNT};
use crate::wire::registers::TICKS_PER_TURN;

pub const DEGREES_PER_TICK: f64 = 360.0 / TICKS_PER_TURN as f64;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointCalibration {
    // The Homing_Offset the servo chose when it was recentred.
    pub homing_offset: i32,
    pub range_min: i32,
    pub range_max: i32,
}

impl JointCalibration {
    pub const IDENTITY: JointCalibration =
        JointCalibration { homing_offset: 0, range_min: 0, range_max: TICKS_PER_TURN - 1 };

    pub fn midpoint(&self) -> f64 {
        (self.range_min + self.range_max) as f64 / 2.0
    }

    pub fn span(&self) -> i32 {
        self.range_max - self.range_min
    }

    pub fn to_value(&self, unit: Unit, ticks: i32) -> f64 {
        match unit {
            Unit::Degrees => (ticks as f64 - self.midpoint()) * DEGREES_PER_TICK,
            Unit::Percent => {
                let span = self.span().max(1) as f64;
                ((ticks - self.range_min) as f64 / span * 100.0).clamp(0.0, 100.0)
            }
        }
    }

    // Always lands inside the calibrated range.
    pub fn to_ticks(&self, unit: Unit, value: f64) -> i32 {
        let ticks = match unit {
            Unit::Degrees => (value / DEGREES_PER_TICK + self.midpoint()).round() as i32,
            Unit::Percent => {
                (self.range_min as f64 + value.clamp(0.0, 100.0) / 100.0 * self.span() as f64).round() as i32
            }
        };
        ticks.clamp(self.range_min, self.range_max)
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "CalibrationFile", into = "CalibrationFile")]
pub struct ArmCalibration {
    pub arm_id: String,
    pub joints: [JointCalibration; COUNT],
}

impl ArmCalibration {
    // Raw ticks, centred at 2048, full turn. What the simulator and an
    // uncalibrated monitor use.
    pub fn identity(arm_id: &str) -> Self {
        Self { arm_id: arm_id.to_string(), joints: [JointCalibration::IDENTITY; COUNT] }
    }

    pub fn from_sweep(arm_id: &str, homing_offsets: Ticks, range_min: Ticks, range_max: Ticks) -> Self {
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

    pub fn ticks_to_values(&self, ticks: &Ticks) -> Values {
        std::array::from_fn(|i| {
            let joint = Joint::ALL[i];
            self.joint(joint).to_value(joint.unit(), ticks[i])
        })
    }

    pub fn values_to_ticks(&self, values: &Values) -> Ticks {
        std::array::from_fn(|i| {
            let joint = Joint::ALL[i];
            self.joint(joint).to_ticks(joint.unit(), values[i])
        })
    }

    // Every joint must have covered at least `min_span_ticks` during its sweep.
    pub fn check_travel(&self, min_span_ticks: i32) -> Result<()> {
        let narrow: Vec<String> = Joint::ALL
            .iter()
            .map(|&j| (j, self.joint(j)))
            .filter(|(_, c)| c.span() < min_span_ticks)
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

impl From<ArmCalibration> for CalibrationFile {
    fn from(cal: ArmCalibration) -> Self {
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

impl TryFrom<CalibrationFile> for ArmCalibration {
    type Error = String;

    fn try_from(file: CalibrationFile) -> std::result::Result<Self, String> {
        let mut joints = [JointCalibration::IDENTITY; COUNT];
        for joint in Joint::ALL {
            let entry = file.joints.get(joint.name()).ok_or_else(|| format!("missing joint {}", joint.name()))?;
            if entry.id != joint.id() {
                return Err(format!("{} has id {}, expected {}", joint.name(), entry.id, joint.id()));
            }
            if entry.range_min > entry.range_max {
                return Err(format!("{} range is inverted", joint.name()));
            }
            joints[joint.index()] =
                JointCalibration { homing_offset: entry.homing_offset, range_min: entry.range_min, range_max: entry.range_max };
        }
        Ok(Self { arm_id: file.arm_id, joints })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wire::registers::CENTER_TICK;

    fn cal() -> JointCalibration {
        JointCalibration { homing_offset: 0, range_min: 769, range_max: 3156 }
    }

    #[test]
    fn degrees_round_trip_inside_range() {
        let c = cal();
        for ticks in [769, 1000, 1962, 2500, 3156] {
            let d = c.to_value(Unit::Degrees, ticks);
            let back = c.to_ticks(Unit::Degrees, d);
            assert!((back - ticks).abs() <= 1, "{ticks} -> {d} -> {back}");
        }
        assert_eq!(c.to_value(Unit::Degrees, 1962), -0.5 * DEGREES_PER_TICK);
    }

    #[test]
    fn to_ticks_clamps_to_range() {
        let c = cal();
        assert_eq!(c.to_ticks(Unit::Degrees, 500.0), 3156);
        assert_eq!(c.to_ticks(Unit::Degrees, -500.0), 769);
    }

    #[test]
    fn percent_round_trip() {
        let c = cal();
        assert_eq!(c.to_value(Unit::Percent, 769), 0.0);
        assert_eq!(c.to_value(Unit::Percent, 3156), 100.0);
        assert_eq!(c.to_ticks(Unit::Percent, 50.0), 1963);
    }

    #[test]
    fn identity_maps_centre_to_zero_degrees() {
        let cal = ArmCalibration::identity("t");
        let values = cal.ticks_to_values(&[CENTER_TICK; COUNT]);
        assert!((values[Joint::ElbowFlex.index()]).abs() < 0.1);
    }

    #[test]
    fn json_round_trip_keeps_the_named_file_layout() {
        let mut a = ArmCalibration::identity("test");
        a.joints[Joint::ElbowFlex.index()].range_min = 100;
        let text = serde_json::to_string(&a).unwrap();
        assert!(text.contains("\"elbow_flex\""));
        assert!(text.contains("\"id\":3"));
        let b: ArmCalibration = serde_json::from_str(&text).unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn file_with_a_missing_or_misnumbered_joint_is_rejected() {
        let mut text = serde_json::to_string(&ArmCalibration::identity("t")).unwrap();
        text = text.replace("\"id\":3", "\"id\":4");
        assert!(serde_json::from_str::<ArmCalibration>(&text).is_err());
        let text = r#"{"arm_id":"t","joints":{}}"#;
        assert!(serde_json::from_str::<ArmCalibration>(text).is_err());
    }
}
