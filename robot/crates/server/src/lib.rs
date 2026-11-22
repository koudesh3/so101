//! The robot's side of the browser apps in `ui/`: serves them (embedded at build time),
//! streams live teleop events to the dashboard, replays a byte log as events, and turns a
//! byte log into an offline HTML report.

pub mod live;
pub mod replay;
pub mod visualize;
