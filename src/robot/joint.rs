//! The six joints of an SO-101 arm, the servo ID each answers on, and the
//! unit its position is expressed in.

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Joint {
    ShoulderPan,
    ShoulderLift,
    ElbowFlex,
    WristFlex,
    WristRoll,
    Gripper,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Unit {
    Degrees,
    Percent,
}

pub const COUNT: usize = 6;

// One value per joint, in `Joint::ALL` order.
pub type Ticks = [i32; COUNT];
pub type Values = [f64; COUNT];

// Servo IDs in `Joint::ALL` order.
pub const IDS: [u8; COUNT] = [1, 2, 3, 4, 5, 6];

impl Joint {
    pub const ALL: [Joint; COUNT] = [
        Joint::ShoulderPan,
        Joint::ShoulderLift,
        Joint::ElbowFlex,
        Joint::WristFlex,
        Joint::WristRoll,
        Joint::Gripper,
    ];

    pub fn index(self) -> usize {
        self as usize
    }

    pub fn id(self) -> u8 {
        IDS[self.index()]
    }

    pub fn from_id(id: u8) -> Option<Joint> {
        Joint::ALL.iter().copied().find(|j| j.id() == id)
    }

    pub fn name(self) -> &'static str {
        match self {
            Joint::ShoulderPan => "shoulder_pan",
            Joint::ShoulderLift => "shoulder_lift",
            Joint::ElbowFlex => "elbow_flex",
            Joint::WristFlex => "wrist_flex",
            Joint::WristRoll => "wrist_roll",
            Joint::Gripper => "gripper",
        }
    }

    pub fn from_name(name: &str) -> Option<Joint> {
        Joint::ALL.iter().copied().find(|j| j.name() == name)
    }

    pub fn unit(self) -> Unit {
        match self {
            Joint::Gripper => Unit::Percent,
            _ => Unit::Degrees,
        }
    }
}

impl Unit {
    pub fn symbol(self) -> &'static str {
        match self {
            Unit::Degrees => "°",
            Unit::Percent => "%",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_and_names_round_trip() {
        for (i, joint) in Joint::ALL.iter().enumerate() {
            assert_eq!(joint.index(), i);
            assert_eq!(joint.id(), i as u8 + 1);
            assert_eq!(Joint::from_id(joint.id()), Some(*joint));
            assert_eq!(Joint::from_name(joint.name()), Some(*joint));
        }
        assert_eq!(Joint::from_id(0), None);
        assert_eq!(Joint::from_id(7), None);
    }

    #[test]
    fn only_the_gripper_is_in_percent() {
        for joint in Joint::ALL {
            let expected = if joint == Joint::Gripper { Unit::Percent } else { Unit::Degrees };
            assert_eq!(joint.unit(), expected);
        }
    }
}
