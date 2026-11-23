//! The SO-101, one folder per thing: the arm (with its servo bus), the camera, the policy, the
//! control loop, and the session log. `cli.rs` puts them together.

pub mod arm;
pub mod camera;
pub mod clock;
pub mod control;
pub mod policy;
pub mod session;

pub use arm::{ActuatorStatus, Counts, Fault, Joint, JointState, JointTarget, Pose, Source, Unit, JOINT_COUNT};
pub use camera::{Camera, Frame, FrameRef};
pub use clock::{Clock, MonotonicClock, ReplayClock, ReplaySpeed, Timestamp};
pub use control::event::{ArmRole, Event, EventBus, Message, RecorderDrop};
pub use control::tick::{Clip, Limit, Tick};
pub use policy::{CheckpointId, Chunk, Obs};
