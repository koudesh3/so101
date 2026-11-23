//! A small synthetic session with every event type, for tests and the Python readability check.

use crate::{
    ActuatorStatus, Camera, CheckpointId, Chunk, Clip, Fault, Frame, FrameRef, Joint, JointState, JointTarget, Limit,
    Pose, Source, Tick, Timestamp,
};
use bytes::Bytes;

use crate::{ArmRole, Event, Message};

const MS: u64 = 1_000_000;

pub fn joints(base: f32) -> Pose {
    Pose([base, base + 1.5, -base, 0.25, 90.0, 42.0])
}

/// A fake JPEG: the SOI marker, then bytes that identify the camera and frame.
pub fn jpeg(camera: Camera, sequence: u32, len: usize) -> Bytes {
    let mut v = vec![0xFF, 0xD8];
    v.extend((0..len.saturating_sub(2)).map(|i| (i as u32 ^ sequence ^ camera as u32) as u8));
    Bytes::from(v)
}

pub fn frame(camera: Camera, sequence: u32, t_capture: Timestamp, len: usize) -> Frame {
    Frame {
        camera,
        sequence,
        t_capture,
        t_receive: t_capture + std::time::Duration::from_millis(33),
        jpeg: jpeg(camera, sequence, len),
    }
}

/// About a third of a second of every event type, starting at `t0`, in publish order.
pub fn session(t0: Timestamp) -> Vec<Event> {
    let t = |ms: u64| Timestamp(t0.0 + ms * MS);
    let mut out = Vec::new();
    for slot in 0..30u32 {
        let ms = slot as u64 * 10;
        let pos = joints(slot as f32 * 0.5);
        let counts = Some([2048 + slot as i32, 1900, 2100, 2048, 1024, 3000]);
        out.push(Event::new(
            Message::JointState(ArmRole::Follower, JointState { t: t(ms), pose: pos, counts }),
            slot,
            t(ms + 1),
        ));
        out.push(Event::new(
            Message::JointState(ArmRole::Leader, JointState { t: t(ms), pose: pos, counts: None }),
            slot,
            t(ms + 1),
        ));
        let joint = Joint::ALL[slot as usize % 6];
        out.push(Event::new(
            Message::Actuator(
                ArmRole::Follower,
                ActuatorStatus {
                    t: t(ms + 2),
                    joint,
                    velocity_deg_s: Some(-12.5),
                    load_percent: Some(0.0),
                    voltage_v: Some(12.1),
                    temperature_c: Some(31.0),
                    current_a: Some(0.11),
                    faults: if slot % 5 == 0 { vec![Fault::Overload] } else { vec![] },
                },
            ),
            slot,
            t(ms + 2),
        ));
        if slot % 3 == 0 {
            let tick = slot / 3;
            let wrist = frame(Camera::Wrist, tick, t(ms), 2000);
            let scene = frame(Camera::Scene, tick + 100, t(ms + 1), 3000);
            let wrist_id = wrist.id();
            let scene_id = scene.id();
            out.push(Event::frame(wrist));
            out.push(Event::frame(scene));
            out.push(Event::new(
                Message::JointTarget(
                    ArmRole::Follower,
                    JointTarget { t: t(ms + 3), pose: joints(1.0), source: Source::Teleop, counts: Some([2060; 6]) },
                ),
                tick,
                t(ms + 3),
            ));
            out.push(Event::new(
                Message::Tick(Tick {
                    t: t(ms + 3),
                    slot,
                    state: pos,
                    wrist: Some(wrist_id),
                    scene: Some(FrameRef { camera: Camera::Scene, sequence: scene_id.sequence }),
                    action: joints(1.0),
                    source: Source::Policy(CheckpointId("ckpt-0005000".into())),
                    requested: joints(1.4),
                    clipped: if tick % 2 == 0 {
                        vec![Clip { joint: Joint::ElbowFlex, limit: Limit::Step }]
                    } else {
                        vec![]
                    },
                }),
                tick,
                t(ms + 3),
            ));
        }
    }
    out.push(Event::new(
        Message::Chunk(Chunk {
            t_obs: t(5),
            period: std::time::Duration::from_nanos(33_333_333),
            actions: vec![joints(2.0), joints(2.5), joints(3.0)],
            checkpoint_id: CheckpointId("ckpt-0005000".into()),
        }),
        0,
        t(9),
    ));
    out.push(Event::new(
        Message::JointTarget(
            ArmRole::Follower,
            JointTarget { t: t(300), pose: joints(0.0), source: Source::Script("home".into()), counts: None },
        ),
        10,
        t(300),
    ));
    out
}
