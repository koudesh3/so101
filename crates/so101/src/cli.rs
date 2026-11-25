//! The `so101` command line: picks the arms, transports and session, and puts them together.

mod console;

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex};

use clap::{Parser, Subcommand};
use so101::arm::{
    check_centred, demo, sweep, ActuatorStatus, ByteTrace, Calibration, ClearanceMap, Error, FollowerGains, Joint,
    Limit, Pose, Result, SerialPort, ServoArm, Servos, SimulatedServos, Traced, Transport, IDS, JOINT_COUNT,
};
use so101::camera::{read_controls, Camera, FfmpegCamera};
use so101::control::{
    request_realtime, run, ArmRole, Event, EventBus, Grid, Message, RunOptions, Teleop, Tick, LATE_AFTER,
};
use so101::session::{Attachment, CameraInfo, Header};

use console::{Console, Recording, RecordingSetup};

// Under the kernel's threaded interrupt handlers (priority 50), so serial and USB I/O still run.
const REALTIME_PRIORITY: i32 = 40;

#[derive(Parser)]
#[command(name = "so101", about = "Calibrate, monitor and teleoperate an SO-101 arm pair from scratch")]
struct Cli {
    /// Record every byte on every serial line to this JSON-lines file.
    #[arg(long, global = true)]
    trace: Option<PathBuf>,
    /// Use simulated servos instead of serial ports.
    #[arg(long, global = true)]
    sim: bool,
    #[command(subcommand)]
    command: Command,
}

// Parsed once; teleop's many flags make it the biggest variant by far, which costs nothing here.
#[allow(clippy::large_enum_variant)]
#[derive(Subcommand)]
enum Command {
    /// Ping one servo and read its model number.
    Ping {
        #[arg(long)]
        port: Option<String>,
        #[arg(long, default_value_t = 1)]
        id: u8,
    },
    /// Ping every possible ID and list who answers.
    Scan {
        #[arg(long)]
        port: Option<String>,
    },
    /// Print live positions and actuator status until Ctrl-C.
    Monitor {
        #[arg(long)]
        port: Option<String>,
        #[arg(long)]
        calibration: Option<PathBuf>,
        #[arg(long, default_value_t = 5.0)]
        hz: f64,
    },
    /// Two-step calibration: middle pose, then a full sweep. Writes a JSON file.
    Calibrate {
        #[arg(long)]
        port: Option<String>,
        #[arg(long)]
        arm_id: String,
        #[arg(long)]
        out: PathBuf,
    },
    /// Mirror the leader on the follower, 1:1, until Ctrl-C.
    Teleop {
        #[arg(long)]
        leader: Option<String>,
        #[arg(long)]
        follower: Option<String>,
        #[arg(long)]
        leader_calibration: Option<PathBuf>,
        #[arg(long)]
        follower_calibration: Option<PathBuf>,
        // Defaults below were tuned on hardware (2026-09-26): 100 Hz, P 32,
        // gripper 80%, 40 ms lead on the big joints took follower lag from
        // 0.5-1.5 s to ~30 ms with no shake at rest.
        #[arg(long, default_value_t = 100.0)]
        hz: f64,
        /// Largest goal change per tick, in counts (0 disables). 20 at 120 Hz allows 2400
        /// counts/s (about 210°/s): measured to keep the follower under 0.6 A on its 12 V 2 A
        /// supply. 40 is full speed; use it on a 5 A supply.
        #[arg(long, default_value_t = 20)]
        max_step: i32,
        /// Stop after this many ticks (0 runs until Ctrl-C).
        #[arg(long, default_value_t = 0)]
        ticks: u64,
        /// The follower's gains and limits, the one source (contract 8): written to every servo
        /// on start and read back; teleop refuses to run if one didn't take.
        #[arg(long, default_value = "data/gains.json")]
        gains: PathBuf,
        /// Command this many ms ahead of the leader on pan, lift, elbow and
        /// gripper, from the leader's velocity (0 = off).
        #[arg(long, default_value_t = 40, value_parser = clap::value_parser!(u64).range(0..=150))]
        lead_ms: u64,
        /// Lead for wrist flex and wrist roll. Quick reversals there turn lead
        /// into overshoot, so it is off by default.
        #[arg(long, default_value_t = 0, value_parser = clap::value_parser!(u64).range(0..=150))]
        wrist_lead_ms: u64,
        /// Read every follower servo's load, voltage and current each tick, with its position
        /// (about 0.8 ms more bus time). The live printout shows the follower's power draw.
        #[arg(long, default_value_t = true, action = clap::ArgAction::Set)]
        actuators: bool,
        /// The wrist camera's clearance map (tools/camera_clearance.py): the follower stops short
        /// of the camera hitting its own arm. `none` turns the guard off.
        #[arg(long, default_value = "data/camera_clearance.json")]
        camera_clearance: String,
        /// How close the wrist camera may come to the arm, in mm, by the map.
        #[arg(long, default_value_t = 10.0)]
        camera_margin_mm: f32,
        /// Serve the console (cameras, load, record button) on this port, on 127.0.0.1 only;
        /// `just console` tunnels it to the laptop. 0 turns it off.
        #[arg(long, default_value_t = 8090)]
        console_port: u16,
        /// The wrist camera's V4L2 device; recorded into the session's MCAP.
        #[arg(long)]
        wrist_camera: Option<String>,
        #[arg(long, default_value = "640x480")]
        wrist_size: String,
        /// The scene camera's V4L2 device; recorded beside the MCAP as video.
        #[arg(long)]
        scene_camera: Option<String>,
        #[arg(long, default_value = "1280x720")]
        scene_size: String,
        /// v4l2 controls for a camera, applied before streaming: "DEVICE name=value ...", as in
        /// robot.conf's CAMERA_CONTROLS. Repeat for each camera.
        #[arg(long = "camera-controls")]
        camera_controls: Vec<String>,
        /// Where recordings go.
        #[arg(long, default_value = "data/logs")]
        record_dir: PathBuf,
    },
    /// Recorded sessions (MCAP files).
    Session {
        #[command(subcommand)]
        command: SessionCommand,
    },
}

#[derive(Subcommand)]
enum SessionCommand {
    /// Rewrite a log cut short (kill, power loss) into a complete file any reader can open.
    /// Keeps everything up to the last complete chunk. Never overwrites OUTPUT.
    Recover { input: PathBuf, output: PathBuf },
}

#[derive(Clone, Copy)]
enum Role {
    Single,
    Leader,
    Follower,
}

impl Role {
    fn label(self) -> &'static str {
        match self {
            Role::Single => "servos",
            Role::Leader => "leader",
            Role::Follower => "follower",
        }
    }

    fn simulated(self) -> SimulatedServos {
        match self {
            Role::Single => SimulatedServos::waving_arm(self.label()),
            Role::Leader => SimulatedServos::demo_leader(self.label()),
            Role::Follower => SimulatedServos::demo_follower(self.label()),
        }
    }
}

fn main() {
    let cli = Cli::parse();
    let trace = cli
        .trace
        .as_ref()
        .map(|p| ByteTrace::create(p).unwrap_or_else(|e| fail(&format!("cannot create trace {}: {e}", p.display()))));
    let open = |role: Role, port: Option<String>| open_servos(role, port, cli.sim, &trace);
    let result = match cli.command {
        Command::Ping { port, id } => cmd_ping(open(Role::Single, port), id),
        Command::Scan { port } => cmd_scan(open(Role::Single, port)),
        Command::Monitor { port, calibration, hz } => cmd_monitor(open(Role::Single, port), calibration, hz),
        Command::Calibrate { port, arm_id, out } => cmd_calibrate(open(Role::Single, port), &arm_id, &out),
        Command::Teleop {
            leader,
            follower,
            leader_calibration,
            follower_calibration,
            hz,
            max_step,
            ticks,
            gains,
            lead_ms,
            wrist_lead_ms,
            actuators,
            camera_clearance,
            camera_margin_mm,
            console_port,
            wrist_camera,
            wrist_size,
            scene_camera,
            scene_size,
            camera_controls,
            record_dir,
        } => {
            let leader_servos = open_servos(Role::Leader, leader, cli.sim, &trace);
            let follower_servos = open_servos(Role::Follower, follower, cli.sim, &trace);
            let cals = (leader_calibration, follower_calibration);
            let gains_file = gains;
            let gains = FollowerGains::load(&gains_file).unwrap_or_else(|e| fail(&e.to_string()));
            let lead = Joint::ALL.map(|j| {
                let ms = if matches!(j, Joint::WristFlex | Joint::WristRoll) { wrist_lead_ms } else { lead_ms };
                std::time::Duration::from_millis(ms)
            });
            let tuning = (gains, lead);
            let camera = (camera_clearance != "none").then(|| (PathBuf::from(camera_clearance), camera_margin_mm));
            let io = TeleopIo {
                gains_file,
                console_port: (console_port > 0).then_some(console_port),
                wrist: wrist_camera.map(|d| (d, wrist_size)),
                scene: scene_camera.map(|d| (d, scene_size)),
                camera_controls,
                record_dir,
            };
            cmd_teleop(
                leader_servos,
                follower_servos,
                cals,
                hz,
                max_step,
                ticks,
                tuning,
                cli.sim,
                actuators,
                camera,
                io,
            )
        }
        Command::Session { command: SessionCommand::Recover { input, output } } => cmd_session_recover(&input, &output),
    };
    if let Some(trace) = &trace {
        trace.flush();
    }
    if let Err(e) = result {
        fail(&e.to_string());
    }
}

fn fail(msg: &str) -> ! {
    eprintln!("error: {msg}");
    std::process::exit(1)
}

fn open_servos(role: Role, port: Option<String>, sim: bool, trace: &Option<Arc<ByteTrace>>) -> Servos {
    let label = role.label();
    let transport: Box<dyn Transport> = if sim {
        traced(role.simulated(), label, trace)
    } else {
        let path = port.unwrap_or_else(|| fail(&format!("--port for {label} is required (or pass --sim)")));
        let serial = SerialPort::open(&path).unwrap_or_else(|e| fail(&format!("open {path}: {e}")));
        traced(serial, label, trace)
    };
    Servos::new(transport)
}

fn traced<T: Transport + 'static>(t: T, label: &str, trace: &Option<Arc<ByteTrace>>) -> Box<dyn Transport> {
    match trace {
        Some(trace) => Box::new(Traced::new(t, label, trace.clone())),
        None => Box::new(t),
    }
}

fn load_calibration(path: Option<PathBuf>, label: &str, sim: bool) -> Result<Calibration> {
    match path {
        Some(p) => Calibration::load(&p),
        None if sim && label != "monitor" => Ok(demo::calibration(label)),
        None => {
            eprintln!("note: no calibration for {label}, using identity (raw counts, centred at 2048)");
            Ok(Calibration::identity(label))
        }
    }
}

fn ctrl_c_flag() -> Arc<AtomicBool> {
    let stop = Arc::new(AtomicBool::new(false));
    let s = stop.clone();
    let _ = ctrlc::set_handler(move || s.store(true, Ordering::Relaxed));
    stop
}

fn format_pose(pose: &Pose) -> String {
    Joint::ALL
        .iter()
        .map(|&j| format!("{}={:7.2}{}", j.name(), pose[j], j.unit().symbol()))
        .collect::<Vec<_>>()
        .join("  ")
}

fn cmd_ping(mut servos: Servos, id: u8) -> Result<()> {
    let servo = servos.identify(id)?;
    let model = if servo.is_sts3215() { "STS3215" } else { "unexpected" };
    println!(
        "servo {id} answered on {}: model {} ({model}), present position {} counts",
        servos.name(),
        servo.model,
        servo.present_position
    );
    Ok(())
}

fn cmd_scan(mut servos: Servos) -> Result<()> {
    let found = servos.scan();
    if found.is_empty() {
        println!("no servos answered on {}", servos.name());
        return Ok(());
    }
    println!("servos answering on {}: {:?}", servos.name(), found);
    let missing: Vec<u8> = IDS.iter().copied().filter(|id| !found.contains(id)).collect();
    if !missing.is_empty() {
        println!("missing from the expected {IDS:?}: {missing:?}");
    }
    Ok(())
}

fn cmd_monitor(servos: Servos, calibration: Option<PathBuf>, hz: f64) -> Result<()> {
    let cal = load_calibration(calibration, "monitor", false)?;
    let mut arm = ServoArm::new("arm", servos, cal);
    arm.check_present()?;
    let stop = ctrl_c_flag();
    let mut grid = Grid::new(hz, arm.clock());
    println!("monitoring {} at {hz} Hz; Ctrl-C to stop", arm.port_name());
    while !stop.load(Ordering::Relaxed) {
        grid.wait();
        let pose = arm.read_pose()?;
        let actuators = arm.read_actuators()?;
        println!(
            "\x1b[2J\x1b[H{:<14}{:>9}{:>9}{:>8}{:>7}{:>6}{:>7}  faults",
            "joint", "value", "°/s", "load %", "volt", "temp", "amps"
        );
        for a in &actuators {
            let f = |v: Option<f32>| v.unwrap_or(f32::NAN);
            println!(
                "{:<14}{:>8.1}{}{:>9.1}{:>8.1}{:>6.1}V{:>5.0}C{:>7.2}  {:?}",
                a.joint.name(),
                pose[a.joint],
                a.joint.unit().symbol(),
                f(a.velocity_deg_s),
                f(a.load_percent),
                f(a.voltage_v),
                f(a.temperature_c),
                f(a.current_a),
                a.faults,
            );
        }
    }
    Ok(())
}

fn cmd_calibrate(servos: Servos, arm_id: &str, out: &Path) -> Result<()> {
    let mut arm = ServoArm::new(arm_id, servos, Calibration::identity(arm_id));
    arm.check_present()?;
    let enter = stdin_lines();

    println!("Torque is off. Move every joint to the MIDDLE of its travel: halfway between its two hard stops.");
    println!("Roughly: base facing forward, upper arm straight up, forearm level and pointing forward,");
    println!("wrist in line with the forearm, gripper half open. Then press Enter.");
    arm.torque_off()?;
    enter.recv().map_err(|_| Error::Other("stdin closed".into()))?;
    let offsets = arm.recentre()?;
    println!("homing offsets written: {offsets:?}");

    println!("Now move every joint slowly through its FULL range (and open/close the gripper). Press Enter when done.");
    let sweep = sweep(&mut arm, 30.0, |sweep| {
        print!("\r");
        for (joint, (lo, hi)) in Joint::ALL.iter().zip(sweep.min.iter().zip(sweep.max)) {
            print!("{}:{lo}..{hi}  ", &joint.name()[..3]);
        }
        use std::io::Write as _;
        let _ = std::io::stdout().flush();
        enter.try_recv().is_err()
    })?;
    println!();
    let flags = arm.take_flags();
    if flags.iter().any(|f| f.0 != 0) {
        println!("note: servos raised advisory flags during the sweep: {flags:?}");
    }
    check_centred(&sweep)?;

    let cal = Calibration::from_sweep(arm_id, offsets, sweep.min, sweep.max);
    cal.check_travel(200)?;
    cal.save(out)?;
    println!("saved {}", out.display());
    Ok(())
}

fn stdin_lines() -> mpsc::Receiver<String> {
    let (tx, rx) = mpsc::channel();
    std::thread::spawn(move || {
        let mut line = String::new();
        while std::io::stdin().read_line(&mut line).map(|n| n > 0).unwrap_or(false) {
            if tx.send(line.clone()).is_err() {
                break;
            }
            line.clear();
        }
    });
    rx
}

#[allow(clippy::too_many_arguments)]
fn cmd_teleop(
    leader_servos: Servos,
    follower_servos: Servos,
    (leader_cal, follower_cal): (Option<PathBuf>, Option<PathBuf>),
    hz: f64,
    max_step: i32,
    ticks: u64,
    (gains, lead): (FollowerGains, [std::time::Duration; JOINT_COUNT]),
    sim: bool,
    actuators: bool,
    camera_guard: Option<(PathBuf, f32)>,
    io: TeleopIo,
) -> Result<()> {
    let calibration_files: Vec<(&str, PathBuf)> = [("leader", &leader_cal), ("follower", &follower_cal)]
        .into_iter()
        .filter_map(|(n, p)| Some((n, p.clone()?)))
        .collect();
    let mut leader = ServoArm::new("leader", leader_servos, load_calibration(leader_cal, "leader", sim)?);
    let mut follower = ServoArm::new("follower", follower_servos, load_calibration(follower_cal, "follower", sim)?);
    leader.check_present()?;
    follower.check_present()?;
    for arm in [&mut leader, &mut follower] {
        let changed = arm.configure_readings()?;
        if changed > 0 {
            println!("{}: cleared the position-wrap bit on {changed} servos", arm.label());
        }
    }
    follower.configure_follower_with(gains)?;
    let registers = follower.check_gains(&gains)?;
    println!(
        "gains from {}: P {} I {} D {}, gripper {}%; all six servos read back",
        io.gains_file.display(),
        gains.p,
        gains.i,
        gains.d,
        gains.max_torque[Joint::Gripper.index()] / 10
    );
    match camera_guard {
        Some((path, margin_mm)) => {
            let map = ClearanceMap::load(&path)?;
            follower.set_camera_guard(map, margin_mm);
            println!("camera guard on: {} keeps the wrist camera {margin_mm} mm from the arm", path.display());
        }
        None => println!("camera guard off"),
    }

    let options = RunOptions { hz, max_ticks: (ticks > 0).then_some(ticks), read_actuators: actuators };
    let stop = ctrl_c_flag();
    println!(
        "teleop at {hz} Hz, max step {max_step} counts, lead {} ms (wrists {} ms); Ctrl-C to stop (follower torque goes off on exit)",
        lead[Joint::ShoulderPan.index()].as_millis(),
        lead[Joint::WristFlex.index()].as_millis()
    );
    follower.set_step_limit((max_step > 0).then_some(max_step));
    let clock = follower.clock();
    let bus = Arc::new(EventBus::new());
    let config = BTreeMap::from([
        ("hz".to_string(), hz.to_string()),
        ("max_step".into(), max_step.to_string()),
        ("gains_file".into(), io.gains_file.display().to_string()),
        ("lead_ms".into(), lead.map(|d| d.as_millis().to_string()).join(",")),
        ("source".into(), "teleop".into()),
    ]);
    let recording = Arc::new(Recording::new(RecordingSetup {
        dir: io.record_dir.clone(),
        scene_video: io.scene.is_some(),
        clock: clock.clone(),
        header: session_header(config, calibration_files, registers, &io),
    }));
    bus.attach(recording.clone());
    let console = Arc::new(Console {
        wrist: Arc::default(),
        scene: Arc::default(),
        live: Arc::default(),
        recording: recording.clone(),
    });
    let printer = print_teleop(bus.subscribe(), hz, console.live.clone());
    let mut cameras = Vec::new();
    if let Some((device, size)) = &io.wrist {
        let (feed, bus) = (console.wrist.clone(), bus.clone());
        let started =
            FfmpegCamera::start(Camera::Wrist, device, size, &io.controls_for(device), clock.clone(), move |f| {
                feed.put(f.jpeg.clone());
                bus.publish(Event::frame(f));
            });
        cameras.push(report_camera("wrist", device, started));
    }
    if let Some((device, size)) = &io.scene {
        let (feed, recording) = (console.scene.clone(), recording.clone());
        let started =
            FfmpegCamera::start(Camera::Scene, device, size, &io.controls_for(device), clock.clone(), move |f| {
                feed.put(f.jpeg.clone());
                recording.scene_frame(&f);
            });
        cameras.push(report_camera("scene", device, started));
    }
    if let Some(port) = io.console_port {
        match console.serve(port) {
            Ok(()) => println!("console on 127.0.0.1:{port} (just console, then http://localhost:{port})"),
            Err(e) => println!("console off: cannot listen on 127.0.0.1:{port}: {e}"),
        }
    }
    let mut pilot = Teleop::new(leader, lead)?;
    match request_realtime(REALTIME_PRIORITY) {
        Ok(()) => println!("loop at real-time priority (SCHED_FIFO {REALTIME_PRIORITY})"),
        Err(why) => println!("loop at normal priority: {why}"),
    }
    let stats = run(&mut follower, &mut pilot, options, clock, &stop, &bus, |e| println!("ERROR {e}"));
    if recording.is_recording() {
        if let Err(e) = recording.stop() {
            println!("ERROR closing the recording: {e}");
        }
    }
    drop(cameras);
    drop(bus);
    let _ = printer.join();
    let stats = stats?;
    println!(
        "stopped after {} ticks: {} errors, {} skipped, {} started over {} ms late (worst {:.2} ms), worst tick work {:.2} ms",
        stats.ticks,
        stats.errors,
        stats.skipped,
        stats.late,
        LATE_AFTER.as_millis(),
        stats.worst_lateness.as_secs_f64() * 1e3,
        stats.worst_work_time.as_secs_f64() * 1e3
    );
    Ok(())
}

/// The console and cameras around teleop.
struct TeleopIo {
    /// The gains file, attached to every recording beside the registers read back.
    gains_file: PathBuf,
    console_port: Option<u16>,
    /// (device, size)
    wrist: Option<(String, String)>,
    scene: Option<(String, String)>,
    /// "DEVICE name=value ..." per camera.
    camera_controls: Vec<String>,
    record_dir: PathBuf,
}

impl TeleopIo {
    fn controls_for(&self, device: &str) -> Vec<String> {
        self.camera_controls
            .iter()
            .filter_map(|entry| {
                let mut words = entry.split_whitespace();
                (words.next() == Some(device)).then(|| words.map(str::to_string).collect::<Vec<_>>())
            })
            .flatten()
            .collect()
    }
}

fn report_camera(role: &str, device: &str, started: std::io::Result<FfmpegCamera>) -> Option<FfmpegCamera> {
    match started {
        Ok(camera) => {
            println!("{role} camera on {device}");
            Some(camera)
        }
        Err(e) => {
            println!("{role} camera off: {device}: {e}");
            None
        }
    }
}

/// A session header, made fresh at each recording's start: the commit, teleop's settings, both
/// calibrations (attached), each servo's gain registers as read back at teleop's start (nothing
/// else writes them while it runs) with the gains file attached, and each camera's device with
/// its controls read back.
fn session_header(
    config: BTreeMap<String, String>,
    calibrations: Vec<(&'static str, PathBuf)>,
    registers: [String; JOINT_COUNT],
    io: &TeleopIo,
) -> Box<dyn Fn() -> (Header, Vec<Attachment>) + Send + Sync> {
    // As applied: read once, at start, not again at each recording.
    let gains_file = std::fs::read(&io.gains_file).ok().map(|data| Attachment {
        name: "gains.json".into(),
        media_type: "application/json".into(),
        data,
    });
    let cameras: Vec<(&str, String, Vec<String>)> = [("wrist", &io.wrist), ("scene", &io.scene)]
        .into_iter()
        .filter_map(|(role, cam)| cam.as_ref().map(|(device, _)| (role, device.clone(), io.controls_for(device))))
        .collect();
    Box::new(move || {
        let mut header = Header::now();
        let git = |args: &[&str]| {
            std::process::Command::new("git")
                .args(args)
                .output()
                .ok()
                .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
        };
        header.git_commit = git(&["rev-parse", "HEAD"]).unwrap_or_default();
        header.git_dirty = git(&["status", "--porcelain", "--untracked-files=no"]).is_some_and(|s| !s.is_empty());
        header.config = config.clone();
        header
            .config
            .insert("scene_video".into(), "beside the MCAP: <name>-scene.mkv, frame times in <name>-scene.csv".into());
        let mut attachments: Vec<Attachment> = calibrations
            .iter()
            .filter_map(|(name, path)| std::fs::read(path).ok().map(|bytes| header.add_calibration(name, &bytes)))
            .collect();
        attachments.extend(gains_file.clone());
        for joint in Joint::ALL {
            header.servos.insert(joint.name().to_string(), registers[joint.index()].clone());
        }
        for (role, device, controls) in &cameras {
            let names: Vec<&str> = controls.iter().filter_map(|c| c.split('=').next()).collect();
            header.cameras.insert(
                role.to_string(),
                CameraInfo { device: device.clone(), settings: read_controls(device, &names) },
            );
        }
        (header, attachments)
    })
}

fn cmd_session_recover(input: &Path, output: &Path) -> Result<()> {
    let bytes = std::fs::read(input)?;
    // OUTPUT may be an existing recording, so never overwrite it.
    let file = std::fs::OpenOptions::new().write(true).create_new(true).open(output)?;
    let (_, summary) =
        so101::session::recover(&bytes, std::io::BufWriter::new(file)).map_err(|e| Error::Other(e.to_string()))?;
    match summary.truncated {
        Some(why) => {
            println!("recovered {} messages -> {} (input stopped early: {why})", summary.messages, output.display())
        }
        None => println!("input was complete; copied {} messages -> {}", summary.messages, output.display()),
    }
    Ok(())
}

fn report_holds(tick: &Tick, held: &mut [Option<Limit>; JOINT_COUNT]) {
    for joint in Joint::ALL {
        let now = tick
            .clipped
            .iter()
            .find(|c| c.joint == joint && matches!(c.limit, Limit::Stall | Limit::Collision))
            .map(|c| c.limit);
        match (held[joint.index()], now) {
            (None, Some(Limit::Stall)) => {
                println!("{} blocked: holding at contact; back the leader off to resume", joint.name())
            }
            (None, Some(_)) => {
                println!("{} stopped: the wrist camera would hit the arm; back the leader off", joint.name())
            }
            (Some(_), None) => println!("{} released: following again", joint.name()),
            _ => {}
        }
        held[joint.index()] = now;
    }
}

/// Prints what the teleop loop publishes: holds as they start and end, and both arms' poses once a
/// second. Ends when the bus is dropped.
fn print_teleop(
    events: mpsc::Receiver<Event>,
    hz: f64,
    live: Arc<Mutex<console::Live>>,
) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        let mut held = [None; JOINT_COUNT];
        let mut poses = [Pose::default(); 2];
        let mut power = Power::default();
        let every = (hz as u64).max(1);
        for event in events {
            match &event.message {
                Message::JointState(ArmRole::Leader, s) => poses[0] = s.pose,
                Message::JointState(ArmRole::Follower, s) => poses[1] = s.pose,
                Message::Actuator(ArmRole::Follower, a) => {
                    power.add(a);
                    live.lock().expect("live status").add(a);
                }
                Message::Tick(tick) => {
                    report_holds(tick, &mut held);
                    live.lock().expect("live status").held = held;
                    if (event.sequence as u64 + 1).is_multiple_of(every) {
                        println!(
                            "tick {:>6}\n   leader   {}\n   follower {}",
                            event.sequence + 1,
                            format_pose(&poses[0]),
                            format_pose(&poses[1])
                        );
                        if let Some(line) = power.take_line() {
                            println!("   power    {line}");
                        }
                    }
                }
                _ => {}
            }
        }
    })
}

/// The follower's draw on its supply, summed over the servos, for the live printout.
#[derive(Default)]
struct Power {
    current_a: [f32; JOINT_COUNT],
    peak_a: f32,
    lowest_v: Option<f32>,
}

impl Power {
    fn add(&mut self, a: &ActuatorStatus) {
        if let Some(amps) = a.current_a {
            self.current_a[a.joint.index()] = amps;
            self.peak_a = self.peak_a.max(self.current_a.iter().sum());
        }
        if let Some(v) = a.voltage_v {
            self.lowest_v = Some(self.lowest_v.map_or(v, |low| low.min(v)));
        }
    }

    /// Since the last line: lowest voltage, total current now, peak total.
    fn take_line(&mut self) -> Option<String> {
        let lowest_v = self.lowest_v.take()?;
        let line = format!(
            "lowest {lowest_v:.1} V, total {:.2} A now, {:.2} A peak",
            self.current_a.iter().sum::<f32>(),
            self.peak_a
        );
        self.peak_a = 0.0;
        Some(line)
    }
}
