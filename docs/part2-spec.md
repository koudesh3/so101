# Part 2 project spec: chained ACT on Tower of Hanoi

Nov 21, 2026

## Goal and scope

Part 2 builds a single-arm robot that solves 4-disc Tower of Hanoi, 15 moves in about 5 minutes. ACT handles the contact-level skills, a sequencer handles everything else, and every part of the stack is logged so the dev log can follow whatever turns out to be interesting.

What the post shows, each as a measured result on this task:

1. How ACT learns one skill: success at 25, 50, and 100 demos.
2. The product rule: predicted chain success from per-step rates against measured.
3. What retries and seam data buy.
4. Last and optional: a deterministic sequencer against an Opus sequencer.

Out of scope: separate toy tasks, sim, RL, new model architectures, language-conditioned policies, and bimanual work.

## Setup

| Component | Choice | Role |
|---|---|---|
| Arm | One SO-101 follower + leader (Seeed) | Execution; teleop for demos and takeovers |
| Wrist camera | innomaker U20CAM-1080P on the gripper (USB 2.0 UVC, MJPEG and YUY2, up to 1080p at 30 fps, 130° diagonal / 103° horizontal, M12 lens) | ACT's only camera |
| Scene camera | icSpring USB camera (2993:0858, MJPEG only, 30 fps), on the Seeed Robot Arm Camera Mount, top-down | Done checks, Opus, eval video; recorded, not fed to ACT |
| Compute | DGX Spark, local | Training, ACT inference, recorder, eval |
| System 2 | Opus via API | Optional sequencer, last phase |
| Control stack | Rust teleop loop from Part 1 | Servo layer, safety clamps, recorder transport |
| Task | Tower of Hanoi set, 3 pegs, 4 discs used | The task |

## The task

Each of the 15 moves runs the same four pieces. Only grasp and place are learned, so ACT needs two skills, not fifteen.

| Piece | Who does it | Input | Ends when |
|---|---|---|---|
| 1. Transit to above source peg | Scripted: taught hover pose per peg | Peg ID | Pose reached |
| 2. Grasp top disc and lift | ACT skill `grasp` | Wrist image + joints | Gripper holding (position/current) and lifted |
| 3. Transit to above target peg | Scripted | Peg ID | Pose reached |
| 4. Lower, place, release, retreat | ACT skill `place` | Wrist image + joints | Disc on peg (scene check), gripper open, arm clear |

Why this split: a wrist-only ACT can't tell pegs apart, but it sees the local geometry well. Scripted transits give it a fixed start view, which is the canonical start state from the chaining spike. The variation ACT has to learn is real: four disc sizes, stack heights from 1 to 4, and slight placement error.

Moves: the sequencer computes the 15-move solution with the standard recursion and passes (from, to) per move. A wrong disc order is caught by the scene check, never by ACT.

Later experiment (phase 10): replace pieces 1–3 with one ACT conditioned on (from, to), and compare scripted against learned transit.

## Architecture

At run time, the scene camera serves the sequencer and the checks, the wrist camera serves ACT, and the leader arm can take over at any failure. Everything lands in one log store, which feeds training offline. Each new checkpoint returns to the runtime as an ONNX file.

### Modules

The system is one runtime boundary with five inputs and three outputs. Every edge inside it carries a named message from `contracts`, so any module can be swapped as long as it speaks the same messages.

Hardware never appears inside the boundary: the SO-101 is reached only through `drivers::arm::so101`, which reads LeRobot's calibration file and converts servo ticks to LeRobot's units (degrees, gripper 0–100). Joint state flows from the arm driver to the checks and skills through the bus; the diagram leaves those lines out for readability.

| Module | Trait it implements or owns | Implementations | Consumes | Produces |
|---|---|---|---|---|
| `contracts` | Message types, `Clock` | wall, replay | — | Shared types for Rust and Python |
| `drivers::arm` | `ArmDriver` | so101, replay, mock | JointTarget | JointState |
| `drivers::camera` | `FrameSource` | v4l2_usb (wrist, scene), replay, synthetic | USB video | Frame |
| `drivers::leader` | `TargetSource` | so101_leader, replay | Leader joints | JointTarget |
| `control::safety` | Wraps any `ArmDriver` | — | JointTarget | Clamped JointTarget, e-stop |
| `control::mux` | Picks the active `TargetSource` | — | Targets from leader, scripts, action queue | One JointTarget stream |
| `control::action_queue` | — | LeRobot's queue: n_action_steps per chunk; ensembling optional | Chunk | Per-tick JointTarget |
| `skills` | `Skill`, `Policy` | transit (scripted), grasp, place (ACT via ort), replay | RunStep, Obs | Chunk or JointTarget |
| `checks` | `Check` | holding_rule, scene_slot | Frame, JointState | CheckResult |
| `runtime::step_runner` | — | — | RunStep, CheckResult | StepResult |
| `runtime::sequencer` | `Sequencer` | hanoi_fsm; opus_agent in the last phase | Task goal, StepResult, scene Frame | Command |
| `runtime::bus` + `record` | Event sink | recorder, export | Every Event | Log store, versioned datasets |
| `ml/export` | Offline script | — | Checkpoint, logged observations | act.onnx, export parity report |

Rules that keep it swappable: messages in LeRobot's keys and units; one writer to the arm; safety as a wrapper that code above can't bypass; one clock with timestamps on everything; a conformance test suite per trait that mock, replay, and real implementations all pass.

Skills as a catalog. Each skill registers a name, a typed argument struct, and the checks that end it. The deterministic sequencer calls the catalog as plain Rust functions. An MCP server or Opus tool use can wrap the same catalog later without touching the skills.

## Training pipeline and the policy artifact

Recording and deployment share one Rust driver, LeRobot does everything between them unchanged, and the trained policy comes back as one ONNX file the Rust runtime loads with ort, with no Python at run time. PyTorch stays only as the offline reference in the export parity test.

The plugin is a thin Robot subclass that calls the Rust driver through PyO3. To LeRobot it looks like any SO-101, so lerobot-record and lerobot-train run unchanged, and the dataset format is LeRobot's own.

### What crosses the boundary

| Artifact | Written by | Read by | Format |
|---|---|---|---|
| Calibration | lerobot-calibrate, once | Rust driver, plugin | LeRobot calibration JSON; the only calibration source |
| Observations and actions at record time | Rust driver via plugin | lerobot-record | `<joint>.pos` keys (shoulder_pan … gripper), degrees, gripper 0–100; cameras keyed `wrist`, `scene` |
| Dataset | lerobot-record | lerobot-train, your probes | LeRobotDataset v3: parquet, mp4, meta/info.json (features, fps), meta/stats.json |
| Checkpoint | lerobot-train | Export script | model.safetensors, config.json, pre- and post-processor JSON with normalizer stats |
| Policy artifact | Export script | Rust runtime | ONNX: state[6] and wrist image in, chunk[k][6] in degrees out, normalization inside |

### Export, step by step

1. LeRobot has no built-in export command yet; an open issue proposes one and notes that torch.onnx.export works for ACT (issue #3146).
2. Normalization lives outside the weights, in the pre- and post-processor pipelines (backward compatibility notes). Wrap preprocessor, model, and postprocessor in one module before export, so the ONNX file takes raw degrees and returns degrees.
3. Export only the inference path: ACT sets the latent z to zero at inference, so the CVAE encoder is dropped.
4. Load it in Rust with ort, the ONNX Runtime wrapper, using the CUDA or TensorRT execution provider. Check whether prebuilt GPU binaries cover the Spark's platform; if not, build ONNX Runtime from source.
5. The action queue stays in Rust, outside the model. Use LeRobot's default: run n_action_steps of each chunk with temporal ensembling off, the same as lerobot-eval.

One rate for record and deploy. ACT's chunk steps are dataset frames, and the dataset's fps lives in meta/info.json. Record and run the policy loop at 30 fps. The servo layer can still run faster underneath.

### Parity tests

| Test | Compares | Passes when | Phase |
|---|---|---|---|
| Static read | LeRobot's SO-101 driver vs yours, same held pose | Joint values match within encoder resolution | 1 |
| Gains | Servo registers after each driver's configure | P 16, I 0, D 32 on every motor | 1 |
| Image | One frame through both preprocessing paths | Identical tensors | 1 |
| Export | ONNX vs PyTorch on 100 logged observations | Max action difference under a small tolerance you set, e.g. 0.01° | 2 |

### Inference in the Rust runtime

When the action queue runs empty, the Rust process takes the latest wrist frame and joint state, runs the ONNX policy with ort on the Spark's GPU (normalization is inside the graph), and refills the queue with the new chunk. No Python and no IPC sit on the control path.

The shapes follow the ACT paper's setup: a 480×640 image becomes a 15×20×512 feature map, flattened to 300 tokens. With one camera, the encoder sees 302 tokens: the zero latent, the state, and the image tokens. Chunk size k is a config value (100 by default), and LeRobot's layer defaults differ from the paper's (its decoder has 1 layer, matching a bug in the original code); read both from the checkpoint's config.json rather than hard-coding them.

Inference budget. With LeRobot's default the policy runs once every n_action_steps ticks (100 by default, about 3.3 s at 30 fps), so inference doesn't have to fit inside one 33 ms tick. Its latency shows up as a pause at each chunk boundary instead. The original ACT ran inference in about 0.01 s on an RTX 2080 Ti; measure it on the Spark and log every call.

The policy as a Rust type (sketch, not compiled; check `ort`'s current API for input and output handling):

```rust
pub struct OnnxPolicy {
    session: ort::session::Session,  // act.onnx with the CUDA or TensorRT execution provider
    k: usize,                         // chunk size, recorded at export
}

impl Policy for OnnxPolicy {
    fn infer(&self, obs: &Obs) -> Result<Chunk> {
        let image = to_rgb_f32_nchw(&obs.wrist);   // (1, 3, 480, 640); normalization is inside the graph
        let state = obs.joints.to_array();          // (1, 6), degrees
        let out = self.session.run(inputs![
            "observation.images.wrist" => image,
            "observation.state" => state,
        ])?;
        let actions = out["action"].extract::<f32>()?; // (1, k, 6), degrees
        Ok(Chunk::new(obs.t, actions))
    }
}
```

`Policy` is the trait from the module map. A replay policy that reads chunks from a log implements it too, so the runtime can be tested without a GPU or a checkpoint.

## Interfaces

Each boundary is a versioned message type, so any layer can be swapped or replayed from logs.

| Boundary | Direction | Message | Rate | Notes |
|---|---|---|---|---|
| Servo layer ↔ everything | Out | JointState {t, pos[6], vel[6], current[6], temp[6]} | Servo rate | Clamps applied here; nothing above can bypass them |
| | In | JointTarget {t, pos[6], source} | 30 Hz | source = teleop, script, or policy ID |
| Cameras → recorder, policy | Out | Frame {t_capture, t_receive, cam_id, jpeg} | 30 fps | Both timestamps, so you can measure staleness |
| Policy | In → out | Obs {t, wrist_frame, joints, skill, args} → Chunk {t_obs, k, actions[k][6], ckpt_id} | Once per n_action_steps ticks | In-process via ort; the action queue in Rust pops one action per tick; every chunk is logged |
| Step runner | In → out | RunStep {skill, args, timeout, max_tries} → StepResult {outcome, tries, duration, checks_fired} | Per step | outcome: done, retryable, fatal, timeout |
| Checks | Out | CheckResult {name, value, confidence, source_frame_ids} | 5–10 Hz | Deterministic first; frame IDs let any check be re-run offline |
| Skill catalog | Sequencer → runtime | list_skills, run_step, check_scene, go_home, request_human | Per boundary | Plain Rust calls for the deterministic sequencer; an MCP server or Opus wraps the same catalog later |
| Teleop takeover | Human → runtime | Takeover {reason} → episode recorded with origin = takeover | On demand | Becomes seam or correction data |

## Logging spec

Nothing gets thrown away. Every artifact is immutable, has an ID, and records the git SHA and config that produced it, so any plot in the dev log can be traced back to raw data.

| Artifact | What's recorded | Rate | Format | Size (our estimate) |
|---|---|---|---|---|
| Teleop sessions | Both cameras, leader and follower joints, currents, temps, operator notes | Full rate | Raw session log | Mostly video, a few GB per hour |
| Datasets | Episodes with tags: skill, disc size, from/to, stack height, origin (demo, takeover, seam), outcome | Per episode | LeRobot format, versioned; a version never changes once made | Same as above |
| Training runs | Config, dataset version, seed, SHA; L1 and KL losses, learning rate, grad norm | Every step | Local tracker (W&B offline, MLflow, or Parquet) | Small |
| Checkpoints | Weights; optimizer state only in the latest | Every 5k steps, all kept | Safetensors | About 0.3 GB each for an 80M model in fp32, ~6 GB per 100k-step run |
| Checkpoint probes | Predicted vs true action chunks on a fixed held-out episode set; z statistics; cross-attention maps on fixed frames | Every checkpoint | Arrays + rendered PNGs | Small |
| Rollouts | Both camera videos, joint traces, every predicted chunk, executed targets, check outputs, step outcomes, failure class, checkpoint ID | Full rate | Same schema as teleop | Like teleop |
| Sequencer | Every decision; for Opus also prompt, image IDs, response, latency, tokens | Per call | JSONL | Small |
| Dev notes | Daily notes, hypotheses before each run, surprises after | Daily | Markdown in the repo | — |

Why the checkpoint probes matter: saving all checkpoints and running a fixed probe on each lets you replay how ACT learned after the fact, including when the latent collapses and when grasp timing locks in, without re-training.

## Plan

Each phase leaves a working system and produces one dev-log result. Later phases only start once the earlier one meets its exit criterion. Phases 0–3 need no hardware; stop after phase 3 until the Hanoi set and cameras are set up.

- **Phase 0: contracts and log.** The contracts crate (message types, Clock, and the traits for arm, camera, target source and policy; skill, check and sequencer arrive in Phase 3), the event bus, and a log writer and reader. Design: [phase0.md](phase0.md).
  - Exit: a log written and read back gives identical records; the replay clock reproduces a run's timing.
- **Phase 1: runtime on fakes.** Tick loop at 30 fps with the mock arm (follows targets with a lag), a replay camera reading dataset video, the action queue, mux, and safety wrapper.
  - Also, from the 2026-11-23 hardware measurements ([phase0.md](phase0.md)): the servo grid locked at 120 Hz with the skip-on-overrun rule; the teleop lead owned by the teleop source, with a filtered velocity estimate (it adds 1.2-2.1× jitter today); a per-joint shake measure (follower vs target, 5-30 Hz, from the position stream); servo health checks on the median of the last 5 telemetry readings, with impossible values counted; the byte log kept as an optional bus-trace channel, off by default, since every timing measurement so far came from it.
  - Exit: unit tests on the action queue and the safety clamps (joint limits, rate limits, e-stop); a tick-jitter histogram from a 10-minute run.
- **Phase 2: policy artifact.** Build ONNX Runtime with CUDA on the Spark from source; no prebuilt aarch64 GPU binaries existed as of April 2026. Train a wrist-camera ACT on the public lerobot/svla_so101_pickplace dataset, export it, and load it with ort.
  - Exit: export parity passes on 100 observations, compared in degrees in and degrees out, with normalization wrapped into the exported graph (the spike's graph left it outside; see [phase0.md](phase0.md), contract 4); the image boundary test passes (one log through the runtime path, JPEG decoded in Rust, and the training path, MP4 decoded in Python: channel order, value range and size match exactly, and the pixel difference stays within a set tolerance); inference latency is logged; open-loop replay of a held-out episode through the Rust runtime matches PyTorch's predictions.
  - Dev log: the Rust/Python boundary and what a policy file is.
- **Phase 3: deterministic solver on a fake world.** The skill catalog, step runner, and hanoi_fsm, run against stub skills and checks with set success rates; fault injection for a missed grasp, a dropped disc, and a disc on the wrong peg.
  - Transit waypoints are stored as targets (what was commanded), not as where the follower ended up, and "arrived" uses a per-joint tolerance from measurement (within 0.7° at P 32 in a reaching pose).
  - Exit: a 4-disc solve end to end on stubs; each injected fault produces the specified retry, stop, or takeover request; Monte Carlo runs match the success budget's predictions.
- **Stop here until the hardware is set up.**
- **Phase 4: infrastructure.** Real driver behind the plugin, both cameras, scene camera locked on the mount with exposure and white balance fixed, hover poses taught for 3 pegs. The Rust driver is exposed to LeRobot as `lerobot_robot_*` and `lerobot_teleoperator_*` plugins, calibration comes from lerobot-calibrate, gains are P 32, I 0, D 32 from one file in the repo, applied and read back by the runtime and passed to `act` (measured 2026-11-23: LeRobot's default P 16 stops up to 2.3° short; [phase0.md](phase0.md), contract 8), and everything records at 30 fps. Camera framing spike before any demos: mount the wrist camera, teleoperate one grasp and one place, and check in both 640×480 and 640×360 that the gripper jaws, the top disc and the peg tip stay in view through contact; tilt the camera if the disc hides the peg. Then measure each camera's field of view with a tape measure (for the post) and calibrate the scene camera with a ChArUco board.
  - Exit: record a teleop episode with lerobot-record through the plugin, replay it on the arm, rebuild every logged signal from disk, and pass the static-read, gains, and image parity tests. The runtime refuses to start when a camera's read-back settings differ from the training data's, the converter refuses to mix calibrations, and the runtime refuses a policy trained at different servo gains ([phase0.md](phase0.md), contracts 6-8).
  - Dev log: the stack and the logging design.
- **Phase 5: first skill, grasp.** Train at 25, 50, and 100 demos and probe every checkpoint; export each checkpoint and run it in Rust.
  - Exit: ≥ 85% first-try over 40 trials across all disc sizes and stack heights.
  - Dev log: the learning curve and what the checkpoint probes show.
- **Phase 6: second skill, checks, retries.** `place`, the done and failure checks on the scene camera, and retries, with the phase 3 step runner now calling real skills.
  - Exit: one full move (transit, grasp, transit, place) at ≥ 90% with retries over 40 trials; check accuracy measured against hand labels.
  - Dev log: first-try vs with-retry success; how often the done check is wrong.
- **Phase 7: 3-disc Hanoi (7 moves), deterministic sequencer.**
  - Exit: 20 full runs, logged.
  - Dev log: predicted chain success (product of per-step rates) against measured; where the seams cost.
- **Phase 8: seam data, then 4-disc Hanoi (15 moves, ~5 min).** One round of takeover data at failed seams, retrain, then run 4 discs.
  - Exit: 20 full 4-disc runs.
  - Dev log: before and after seam data; the 5-minute result.
- **Phase 9: Opus as sequencer (optional, last).** Wrap the skill catalog as tools (an MCP server or API tool use); same checks; add perturbations (a human moves a disc mid-run).
  - Exit: 20 runs each, deterministic and Opus, with and without perturbation.
  - Dev log: success, recoveries, wrong advances, latency, cost.
- **Phase 10: optional experiments, only if time allows:** an offline depth ablation, run first because it needs no robot time (a monocular depth model over every logged wrist frame, then ACT trained on RGB vs RGB + depth from the same episodes and seed, scored on grasp by stack height); wrist vs wrist + scene camera for ACT; learned transit conditioned on (from, to); per-stage camera dropout; lighting, first as an offline test that relights held-out wrist frames and measures how far predicted actions drift, then brightness jitter on single-light demos against the same number of mixed-light demos, scored under a lamp neither saw; field of view, the 640×480 crop against 640×360 full width; motor currents added to the state; and the CVAE (z) on against off, with several seeds.

## Success budget

For 4 discs at 50% end-to-end, each move needs about 95.5%, so each of grasp and place needs about 97.7% after retries. 85% first-try with one retry gets there, if every failure leaves the scene retryable. At 80% end-to-end, each skill needs about 99.3%, which takes 3 tries at 81% first-try.

| Target | Discs (moves) | Per move | Per skill (grasp, place) | First-try needed with r tries |
|---|---|---|---|---|
| 50% | 3 (7) | 90.6% | 95.2% | 78% with 2 tries |
| 50% | 4 (15) | 95.5% | 97.7% | 85% with 2 tries |
| 80% | 4 (15) | 98.5% | 99.3% | 81% with 3 tries |

These assume transits never fail and done checks are perfect. Every check error lowers the per-skill number, so measure check accuracy in phase 6 and fold it into the prediction. All figures are computed from pⁿ and 1 − (1 − p)ʳ.

## Risks and open questions

| Risk | Why it matters | Mitigation |
|---|---|---|
| Scene camera drifts | Auto-exposure and white balance change the image the checks compare against; a bumped mount shifts the view | Lock exposure and white balance in the camera driver, mark the mount, photograph the reference view |
| Disc hides the peg in the wrist view during place | ACT is blind at final alignment | Tilt the wrist camera; check in phase 6 before collecting 100 demos |
| Largest disc vs gripper opening | Grasp may be impossible on the base disc | Measure before phase 5 |
| Tight peg fit | Insertion becomes the hard step | Check clearance; sand or swap pegs if needed |
| Servo heat over 5-minute runs | Drift or shutdown late in a run | Log temps (already in JointState); add a cool-down rule, acting on the median of the last 5 readings (single readings are sometimes junk) |
| Shoulder lift rings at about 5 Hz | The arm shakes on fast moves; demos and policy runs both carry it | A short D sweep on shoulder lift before recording datasets; then freeze the gains (contract 8) |
| Servo gains drift between tools | LeRobot writes P 16 on every connect, our runtime P 32; datasets and runs would see different arms | One gains file, applied by both and logged in every header (contract 8) |
| Disc knocked off the table | Unrecoverable, breaks the budget | A lipped tray under the pegs |

### Open questions

- [ ] How many discs does the set have, and what are the largest and smallest diameters?
- [ ] What is the gripper's maximum opening?
- [ ] Which camera is on the wrist? The notes below have the Innomaker on the wrist and the icSpring on the scene, but on 2026-11-23 the icSpring was mounted on the wrist (its image turned with wrist roll) and the Innomaker wasn't visible to the Spark at all. Settle the roles, fix this section, and check the Innomaker's USB connection.
- [ ] Cameras: both are characterized on the Spark (see [the devlog entry](devlog/2026-11-21-camera-spike/)). Wrist camera (innomaker U20CAM-1080P): 29.5 fps, no drops in 10 minutes, about 33 ms latency (p99 37 ms), exposure and white balance lockable. Scene camera (icSpring): 29.8 fps with the same latency, but it drops to 16.6–20 fps under auto exposure, so it runs on manual exposure at 33 ms or less, applied by the recording script on every start. Both stream together on the same USB 2.0 hub with no drops. Pair their frames by timestamp, since their rates differ. 640×480 is a centered 4:3 crop on both; 640×360 keeps the full width. Set power-line frequency to 60 Hz. Glass-to-glass latency is about 100 ms at the median (p99 about 145 ms) for both cameras. The wrist camera keeps manual exposure and white balance through process restarts, driver reloads and unplugging; the driver still sets them explicitly on each start.
