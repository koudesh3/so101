//! `ClearanceMap`: how close the wrist camera comes to the rest of the arm, precomputed from the
//! arm's 3D model over a grid of joint angles (`tools/camera_clearance.py`) and looked up per goal.

use std::path::Path;

use crate::arm::{Error, Joint, Pose, Result};

/// Clearance in mm (0 = touching) at each point of a 3-joint grid, interpolated between points.
#[derive(Clone, Debug)]
pub struct ClearanceMap {
    joints: [Joint; 3],
    axes: [Axis; 3],
    mm: Vec<f32>,
}

/// One joint's grid: `count` points from `min`, `step` degrees apart.
#[derive(Clone, Copy, Debug, PartialEq, serde::Deserialize)]
pub struct Axis {
    pub min: f32,
    pub step: f32,
    pub count: usize,
}

#[derive(serde::Deserialize)]
struct MapFile {
    joints: Vec<String>,
    axes: Vec<Axis>,
    clearance_mm: Vec<f32>,
}

impl ClearanceMap {
    /// `mm` is row-major in the order of `joints`.
    pub fn new(joints: [Joint; 3], axes: [Axis; 3], mm: Vec<f32>) -> Result<Self> {
        if axes.iter().any(|a| a.count < 2 || a.step <= 0.0) {
            return Err(Error::Other("clearance map: each axis needs 2+ points and a positive step".into()));
        }
        let expected: usize = axes.iter().map(|a| a.count).product();
        if mm.len() != expected {
            return Err(Error::Other(format!("clearance map: {} values for a grid of {expected}", mm.len())));
        }
        Ok(Self { joints, axes, mm })
    }

    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path)?;
        let file: MapFile =
            serde_json::from_str(&text).map_err(|e| Error::Other(format!("clearance map {}: {e}", path.display())))?;
        let joint = |name: &String| {
            Joint::from_name(name).ok_or_else(|| Error::Other(format!("clearance map: unknown joint {name}")))
        };
        let [a, b, c] = file.joints.as_slice() else {
            return Err(Error::Other("clearance map: needs exactly 3 joints".into()));
        };
        let [x, y, z] = file.axes.as_slice() else {
            return Err(Error::Other("clearance map: needs exactly 3 axes".into()));
        };
        Self::new([joint(a)?, joint(b)?, joint(c)?], [*x, *y, *z], file.clearance_mm)
    }

    /// The joints the map depends on; the others don't move the camera relative to what it can hit.
    pub fn joints(&self) -> [Joint; 3] {
        self.joints
    }

    /// Trilinear between grid points; poses past the grid take its edge.
    pub fn clearance_mm(&self, pose: &Pose) -> f32 {
        let cell = |k: usize| {
            let axis = self.axes[k];
            let x = ((pose.0[self.joints[k].index()] - axis.min) / axis.step).clamp(0.0, (axis.count - 1) as f32);
            let i = (x.floor() as usize).min(axis.count - 2);
            (i, x - i as f32)
        };
        let [(i, fi), (j, fj), (k, fk)] = [cell(0), cell(1), cell(2)];
        let (nj, nk) = (self.axes[1].count, self.axes[2].count);
        let at = |a: usize, b: usize, c: usize| self.mm[(a * nj + b) * nk + c];
        let mut total = 0.0;
        for (da, wa) in [(0, 1.0 - fi), (1, fi)] {
            for (db, wb) in [(0, 1.0 - fj), (1, fj)] {
                for (dc, wc) in [(0, 1.0 - fk), (1, fk)] {
                    total += wa * wb * wc * at(i + da, j + db, k + dc);
                }
            }
        }
        total
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Clearance = 60 - wrist roll in degrees, whatever the elbow and wrist flex.
    fn falling_with_roll() -> ClearanceMap {
        let axes = [
            Axis { min: -90.0, step: 90.0, count: 3 },
            Axis { min: -90.0, step: 90.0, count: 3 },
            Axis { min: -180.0, step: 30.0, count: 13 },
        ];
        let mm = (0..3 * 3 * 13).map(|n| 60.0 - (-180.0 + 30.0 * (n % 13) as f32)).collect();
        ClearanceMap::new([Joint::ElbowFlex, Joint::WristFlex, Joint::WristRoll], axes, mm).unwrap()
    }

    fn roll(deg: f32) -> Pose {
        let mut pose = Pose::default();
        pose.0[Joint::WristRoll.index()] = deg;
        pose
    }

    #[test]
    fn interpolates_between_grid_points() {
        let map = falling_with_roll();
        assert!((map.clearance_mm(&roll(0.0)) - 60.0).abs() < 1e-4);
        assert!((map.clearance_mm(&roll(45.0)) - 15.0).abs() < 1e-4, "between the 30° and 60° points");
    }

    #[test]
    fn a_pose_past_the_grid_takes_its_edge() {
        assert!((falling_with_roll().clearance_mm(&roll(400.0)) - (60.0 - 180.0)).abs() < 1e-4);
    }

    #[test]
    fn rejects_a_grid_of_the_wrong_size() {
        let axis = Axis { min: 0.0, step: 1.0, count: 2 };
        assert!(
            ClearanceMap::new([Joint::ElbowFlex, Joint::WristFlex, Joint::WristRoll], [axis; 3], vec![0.0; 7]).is_err()
        );
    }
}
