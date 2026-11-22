//! A small synthetic session with every event type, for tests and the Python readability check.

use bytes::Bytes;
use contracts::{
    Camera, Chunk, Clip, Frame, FrameRef, Joint, JointState, JointTarget, Joints, Limit, ServoTelemetry, Source, Tick,
    Timestamp,
};

use crate::{Arm, Envelope, Event};

const MS: u64 = 1_000_000;

pub fn joints(base: f32) -> Joints {
    Joints([base, base + 1.5, -base, 0.25, 90.0, 42.0])
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
pub fn session(t0: Timestamp) -> Vec<Envelope> {
    let t = |ms: u64| Timestamp(t0.0 + ms * MS);
    let mut out = Vec::new();
    for slot in 0..30u32 {
        let ms = slot as u64 * 10;
        let pos = joints(slot as f32 * 0.5);
        let ticks = Some([2048 + slot as i32, 1900, 2100, 2048, 1024, 3000]);
        out.push(Envelope::new(Event::JointState(Arm::Follower, JointState { t: t(ms), pos, ticks }), slot, t(ms + 1)));
        out.push(Envelope::new(
            Event::JointState(Arm::Leader, JointState { t: t(ms), pos, ticks: None }),
            slot,
            t(ms + 1),
        ));
        let joint = Joint::ALL[slot as usize % 6];
        out.push(Envelope::new(
            Event::ServoTelemetry(
                Arm::Follower,
                ServoTelemetry {
                    t: t(ms + 2),
                    joint,
                    velocity_deg_s: Some(-12.5),
                    load_percent: Some(0.0),
                    voltage_v: Some(12.1),
                    temperature_c: Some(31.0),
                    current_raw: Some(17),
                    status: 0,
                    moving: slot % 2 == 0,
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
            out.push(Envelope::frame(wrist));
            out.push(Envelope::frame(scene));
            out.push(Envelope::new(
                Event::JointTarget(
                    Arm::Follower,
                    JointTarget { t: t(ms + 3), pos: joints(1.0), source: Source::Teleop, ticks: Some([2060; 6]) },
                ),
                tick,
                t(ms + 3),
            ));
            out.push(Envelope::new(
                Event::Tick(Tick {
                    t: t(ms + 3),
                    state: pos,
                    wrist: Some(wrist_id),
                    scene: Some(FrameRef { camera: Camera::Scene, sequence: scene_id.sequence }),
                    action: joints(1.0),
                    source: Source::Policy("ckpt-0005000".into()),
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
    out.push(Envelope::new(
        Event::Chunk(Chunk {
            t_obs: t(5),
            actions: vec![joints(2.0), joints(2.5), joints(3.0)],
            ckpt_id: "ckpt-0005000".into(),
        }),
        0,
        t(9),
    ));
    out.push(Envelope::new(
        Event::JointTarget(
            Arm::Follower,
            JointTarget { t: t(300), pos: joints(0.0), source: Source::Script("home".into()), ticks: None },
        ),
        10,
        t(300),
    ));
    out
}
