//! `Health`: the follower's temperature and voltage, judged on the median of each joint's last
//! five readings (contract 8), so a single junk reading never trips it; junk is counted instead.

use std::fmt;

use crate::arm::{ActuatorStatus, Joint, JOINT_COUNT};

const WINDOW: usize = 5;
/// The servos cut themselves out at their Max_Temperature, 70 C; stop before they do.
pub const OVERHEAT_C: f32 = 65.0;
/// A supply sagging below this share of what it read at start is heading for a brownout, which
/// rebooted all six servos once (2026-11-23). 85% of 11.9 V is about 10.1 V.
pub const SAG_RATIO: f32 = 0.85;
// Contract 8: a temperature over 100 C can't be real.
const IMPOSSIBLE_C: f32 = 100.0;
// A reading this far from its joint's median is junk. Real readings move far slower (2026-11-24:
// steady 34-39 C and 11.8-11.9 V; junk at 45-150 C and 10.2-11.2 V, one reading at a time).
const JUNK_C: f32 = 10.0;
const JUNK_V: f32 = 0.5;

/// Why the follower isn't safe to keep driving.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Trip {
    Overheat { joint: Joint, celsius: f32 },
    Sag { joint: Joint, volts: f32, at_start: f32 },
}

/// Readings that were junk, by kind, over a run.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Junk {
    pub temperature: u64,
    pub voltage: u64,
    /// Over 100 C; also counted in `temperature`.
    pub impossible: u64,
}

#[derive(Default)]
pub struct Health {
    temperature: [Window; JOINT_COUNT],
    voltage: [Window; JOINT_COUNT],
    // The supply as read once every joint had a full window: the median of their medians.
    supply_at_start: Option<f32>,
    junk: Junk,
}

#[derive(Default, Clone)]
struct Window {
    readings: [f32; WINDOW],
    len: usize,
    next: usize,
}

impl Window {
    fn push(&mut self, v: f32) {
        self.readings[self.next] = v;
        self.next = (self.next + 1) % WINDOW;
        self.len = (self.len + 1).min(WINDOW);
    }

    fn full(&self) -> bool {
        self.len == WINDOW
    }

    fn median(&self) -> Option<f32> {
        (self.len > 0).then(|| median(&self.readings[..self.len]))
    }
}

impl Health {
    /// Takes one actuator reading; returns a trip if the joint's medians now cross a limit.
    pub fn add(&mut self, a: &ActuatorStatus) -> Option<Trip> {
        let i = a.joint.index();
        if let Some(c) = a.temperature_c {
            self.temperature[i].push(c);
            self.junk.impossible += u64::from(c > IMPOSSIBLE_C);
            self.junk.temperature += u64::from(self.temperature[i].median().is_some_and(|m| (c - m).abs() > JUNK_C));
        }
        if let Some(v) = a.voltage_v {
            self.voltage[i].push(v);
            self.junk.voltage += u64::from(self.voltage[i].median().is_some_and(|m| (v - m).abs() > JUNK_V));
            if self.supply_at_start.is_none() && self.voltage.iter().all(Window::full) {
                let medians: Vec<f32> = self.voltage.iter().filter_map(Window::median).collect();
                self.supply_at_start = Some(median(&medians));
            }
        }
        self.check(a.joint)
    }

    // Only on full windows: with fewer readings, one junk value can be the median.
    fn check(&self, joint: Joint) -> Option<Trip> {
        let i = joint.index();
        let full_median = |w: &Window| w.median().filter(|_| w.full());
        if let Some(celsius) = full_median(&self.temperature[i]).filter(|&c| c >= OVERHEAT_C) {
            return Some(Trip::Overheat { joint, celsius });
        }
        let at_start = self.supply_at_start?;
        let volts = full_median(&self.voltage[i])?;
        (volts < at_start * SAG_RATIO).then_some(Trip::Sag { joint, volts, at_start })
    }

    pub fn temperature_c(&self, joint: Joint) -> Option<f32> {
        self.temperature[joint.index()].median()
    }

    pub fn voltage_v(&self, joint: Joint) -> Option<f32> {
        self.voltage[joint.index()].median()
    }

    /// The lowest joint's median voltage: what the supply is doing, without single-reading junk.
    pub fn supply_v(&self) -> Option<f32> {
        Joint::ALL.iter().filter_map(|&j| self.voltage_v(j)).min_by(f32::total_cmp)
    }

    pub fn junk(&self) -> Junk {
        self.junk
    }
}

impl fmt::Display for Trip {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Trip::Overheat { joint, celsius } => {
                write!(f, "{} at {celsius:.0} C (median of 5), at or over {OVERHEAT_C:.0} C", joint.name())
            }
            Trip::Sag { joint, volts, at_start } => write!(
                f,
                "{} supply at {volts:.1} V (median of 5), under {:.0}% of the {at_start:.1} V it read at start",
                joint.name(),
                SAG_RATIO * 100.0
            ),
        }
    }
}

fn median(values: &[f32]) -> f32 {
    let mut v = values.to_vec();
    v.sort_by(f32::total_cmp);
    v[v.len() / 2]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clock::Timestamp;

    fn reading(joint: Joint, celsius: f32, volts: f32) -> ActuatorStatus {
        ActuatorStatus {
            t: Timestamp(0),
            joint,
            velocity_deg_s: None,
            load_percent: None,
            voltage_v: Some(volts),
            temperature_c: Some(celsius),
            current_a: None,
            faults: Vec::new(),
        }
    }

    // `n` rounds of every joint at the same temperature and voltage.
    fn steady(h: &mut Health, n: usize, celsius: f32, volts: f32) {
        for _ in 0..n {
            for j in Joint::ALL {
                assert_eq!(h.add(&reading(j, celsius, volts)), None);
            }
        }
    }

    #[test]
    fn a_single_junk_temperature_is_counted_not_tripped() {
        let mut h = Health::default();
        steady(&mut h, 5, 35.0, 11.9);
        assert_eq!(h.add(&reading(Joint::ShoulderPan, 150.0, 11.9)), None);
        steady(&mut h, 3, 35.0, 11.9);
        assert_eq!(h.junk(), Junk { temperature: 1, voltage: 0, impossible: 1 });
        assert_eq!(h.temperature_c(Joint::ShoulderPan), Some(35.0));
    }

    #[test]
    fn junk_in_the_first_reading_does_not_trip() {
        let mut h = Health::default();
        assert_eq!(h.add(&reading(Joint::Gripper, 150.0, 11.9)), None);
        steady(&mut h, 5, 35.0, 11.9);
    }

    #[test]
    fn a_servo_that_stays_hot_trips_on_the_third_reading() {
        let mut h = Health::default();
        steady(&mut h, 5, 40.0, 11.9);
        assert_eq!(h.add(&reading(Joint::ElbowFlex, 66.0, 11.9)), None);
        assert_eq!(h.add(&reading(Joint::ElbowFlex, 66.0, 11.9)), None);
        let trip = h.add(&reading(Joint::ElbowFlex, 66.0, 11.9));
        assert_eq!(trip, Some(Trip::Overheat { joint: Joint::ElbowFlex, celsius: 66.0 }));
    }

    #[test]
    fn a_single_voltage_dip_is_counted_not_tripped() {
        // The gripper's junk on 2026-11-24: one reading at 10.2 V among 11.9s.
        let mut h = Health::default();
        steady(&mut h, 5, 35.0, 11.9);
        assert_eq!(h.add(&reading(Joint::Gripper, 35.0, 10.2)), None);
        steady(&mut h, 2, 35.0, 11.9);
        assert_eq!(h.junk().voltage, 1);
        assert_eq!(h.supply_v(), Some(11.9));
    }

    #[test]
    fn a_supply_that_sags_and_stays_down_trips() {
        let mut h = Health::default();
        steady(&mut h, 5, 35.0, 11.9);
        let mut trip = None;
        for _ in 0..3 {
            trip = h.add(&reading(Joint::ShoulderLift, 35.0, 9.8));
        }
        assert!(matches!(trip, Some(Trip::Sag { joint: Joint::ShoulderLift, volts, .. }) if volts == 9.8), "{trip:?}");
    }

    #[test]
    fn a_lower_supply_from_the_start_is_its_own_baseline() {
        // A 5 V leader-style supply isn't a sag; only a drop from where it started is.
        let mut h = Health::default();
        steady(&mut h, 10, 35.0, 4.9);
        assert_eq!(h.supply_v(), Some(4.9));
    }
}
