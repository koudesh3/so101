//! Policies: what they're given, what they return, and what they guarantee (contract 4).

use crate::joint::joints_from_pb;
use crate::pb::so101::v1 as pb;
use crate::{ContractError, DynError, Frame, Joints, Timestamp, JOINT_COUNT};

type Result<T> = std::result::Result<T, ContractError>;

/// What a policy is given. What the loop actually used is logged as a `Tick`.
#[derive(Clone, Debug, PartialEq)]
pub struct Obs {
    pub t: Timestamp,
    pub joints: Joints,
    pub wrist: Option<Frame>,
}

/// A predicted action chunk, in degrees.
#[derive(Clone, Debug, PartialEq)]
pub struct Chunk {
    pub t_obs: Timestamp,
    pub actions: Vec<Joints>,
    pub ckpt_id: String,
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
    fn infer(&mut self, obs: &Obs) -> std::result::Result<Chunk, DynError>;
}

impl From<&Chunk> for pb::Chunk {
    fn from(m: &Chunk) -> Self {
        pb::Chunk {
            t_obs_ns: m.t_obs.0,
            k: m.actions.len() as u32,
            actions: m.actions.iter().flat_map(|j| j.0).collect(),
            ckpt_id: m.ckpt_id.clone(),
        }
    }
}

impl TryFrom<pb::Chunk> for Chunk {
    type Error = ContractError;
    fn try_from(m: pb::Chunk) -> Result<Self> {
        if m.actions.len() != m.k as usize * JOINT_COUNT {
            return Err(ContractError::new(format!(
                "Chunk: k = {} needs {} values, got {}",
                m.k,
                m.k as usize * JOINT_COUNT,
                m.actions.len()
            )));
        }
        let actions =
            m.actions.chunks_exact(JOINT_COUNT).map(|r| joints_from_pb(r, "Chunk.actions")).collect::<Result<_>>()?;
        Ok(Chunk { t_obs: Timestamp(m.t_obs_ns), actions, ckpt_id: m.ckpt_id })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chunk_shape_is_checked() {
        let bad = pb::Chunk { t_obs_ns: 0, k: 2, actions: vec![0.0; 6], ckpt_id: "x".into() };
        assert!(Chunk::try_from(bad).is_err());
    }
}
