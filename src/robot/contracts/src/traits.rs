//! The traits the runtime is built from. Phases 1 and 2 implement these; `Skill`, `Check` and
//! `Sequencer` arrive in Phase 3 with the step runner that calls them. Signatures are refined
//! when their first implementation lands.

use crate::{Camera, Chunk, Frame, JointState, JointTarget, Joints, Timestamp};

pub type DynError = Box<dyn std::error::Error + Send + Sync>;

/// The one writer to an arm (behind the safety wrapper, from Phase 1).
pub trait ArmDriver: Send {
    fn read_state(&mut self) -> Result<JointState, DynError>;
    fn write_target(&mut self, target: &JointTarget) -> Result<(), DynError>;
}

/// A camera, real (V4L2), replayed or synthetic. Blocks until the next frame.
pub trait FrameSource: Send {
    fn camera(&self) -> Camera;
    fn next_frame(&mut self) -> Result<Frame, DynError>;
}

/// Something that proposes targets: the leader arm, a script, the action queue.
pub trait TargetSource: Send {
    fn next_target(&mut self) -> Result<JointTarget, DynError>;
}

/// What a policy is given. What the loop actually used is logged as a `Tick`.
#[derive(Clone, Debug, PartialEq)]
pub struct Obs {
    pub t: Timestamp,
    pub joints: Joints,
    pub wrist: Option<Frame>,
}

/// A trained policy (contract 4):
///
/// - It takes and returns degrees (gripper 0-100), the units of `JointState` and `JointTarget`.
///   Nothing outside the policy ever sees normalized values.
/// - It owns its normalization: inside the ONNX graph per the spec's export, or in a JSON stats
///   file that names the model it belongs to.
/// - Its constructor refuses to load if the normalization doesn't match the model, so a model
///   with the wrong stats can't run.
/// - Every chunk it returns carries its checkpoint ID.
pub trait Policy: Send {
    fn checkpoint_id(&self) -> &str;
    fn infer(&mut self, obs: &Obs) -> Result<Chunk, DynError>;
}
