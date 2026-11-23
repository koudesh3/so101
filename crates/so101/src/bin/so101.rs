use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};

use clap::{Parser, Subcommand};

use so101::arm::calibration::{self, Calibration};
use so101::arm::servos::Servos;
use so101::arm::servos::{Error, Result};
use so101::arm::sim::{demo, SimulatedServos};
use so101::arm::teleop::{self, TeleopOptions};
use so101::arm::uart::{ByteLog, Recorder, SerialTransport, Transport};
use so101::arm::{Arm, FollowerGains};
use so101::arm::{Pose, IDS, JOINT_COUNT};
use so101::control::grid::Grid;
use so101::{ArmRole, Joint, Limit, Message, Tick};

#[derive(Parser)]
#[command(name = "so101", about = "Calibrate, monitor and teleoperate an SO-101 arm pair from scratch")]
struct Cli {
    /// Record every byte on every serial line to this JSON-lines file.
    #[arg(long, global = true)]
    log: Option<PathBuf>,
    /// Use simulated servos instead of serial ports.
    #[arg(long, global = true)]
    sim: bool,
    #[command(subcommand)]
    command: Command,
}

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
        /// Largest goal change per tick, in counts (0 disables). At 100 Hz, 40
        /// allows 4000 counts/s: above the servo's top speed, below a lunge.
        #[arg(long, default_value_t = 40)]
        max_step: i32,
        /// Stop after this many ticks (0 runs until Ctrl-C).
        #[arg(long, default_value_t = 0)]
        ticks: u64,
        /// Follower position-loop P gain (servo register 21). LeRobot uses 16;
        /// the factory default is 32. Higher is stiffer and closes gaps faster.
        #[arg(long, default_value_t = 32, value_parser = clap::value_parser!(i32).range(4..=64))]
        p_gain: i32,
        /// Follower gripper drive ceiling in percent. 50 is LeRobot's (gentle,
        /// but halves gripper speed); 100 is full speed. Stall protection
        /// stays on either way.
        #[arg(long, default_value_t = 80, value_parser = clap::value_parser!(i32).range(20..=100))]
        grip_torque: i32,
        /// Command this many ms ahead of the leader on pan, lift, elbow and
        /// gripper, from the leader's velocity (0 = off).
        #[arg(long, default_value_t = 40, value_parser = clap::value_parser!(u64).range(0..=150))]
        lead_ms: u64,
        /// Lead for wrist flex and wrist roll. Quick reversals there turn lead
        /// into overshoot, so it is off by default.
        #[arg(long, default_value_t = 0, value_parser = clap::value_parser!(u64).range(0..=150))]
        wrist_lead_ms: u64,
        /// Also read one follower actuator's status per tick, rotating through the six.
        #[arg(long)]
        actuators: bool,
    },
    /// Session logs (MCAP).
    Log {
        #[command(subcommand)]
        command: LogCommand,
    },
}

#[derive(Subcommand)]
enum LogCommand {
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
    let log = cli
        .log
        .as_ref()
        .map(|p| ByteLog::create(p).unwrap_or_else(|e| fail(&format!("cannot create log {}: {e}", p.display()))));
    let open = |role: Role, port: Option<String>| open_servos(role, port, cli.sim, &log);
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
            p_gain,
            grip_torque,
            lead_ms,
            wrist_lead_ms,
            actuators,
        } => {
            let leader_servos = open_servos(Role::Leader, leader, cli.sim, &log);
            let follower_servos = open_servos(Role::Follower, follower, cli.sim, &log);
            let cals = (leader_calibration, follower_calibration);
            let gains = FollowerGains { p: p_gain, d: FollowerGains::LEROBOT.d, grip_torque: grip_torque * 10 };
            let lead = Joint::ALL.map(|j| {
                let ms = if matches!(j, Joint::WristFlex | Joint::WristRoll) { wrist_lead_ms } else { lead_ms };
                std::time::Duration::from_millis(ms)
            });
            let tuning = (gains, lead);
            cmd_teleop(leader_servos, follower_servos, cals, hz, max_step, ticks, tuning, cli.sim, actuators)
        }
        Command::Log { command: LogCommand::Recover { input, output } } => cmd_log_recover(&input, &output),
    };
    if let Some(log) = &log {
        log.flush();
    }
    if let Err(e) = result {
        fail(&e.to_string());
    }
}

fn fail(msg: &str) -> ! {
    eprintln!("error: {msg}");
    std::process::exit(1)
}

fn open_servos(role: Role, port: Option<String>, sim: bool, log: &Option<Arc<ByteLog>>) -> Servos {
    let label = role.label();
    let transport: Box<dyn Transport> = if sim {
        recorded(role.simulated(), label, log)
    } else {
        let path = port.unwrap_or_else(|| fail(&format!("--port for {label} is required (or pass --sim)")));
        let serial = SerialTransport::open(&path).unwrap_or_else(|e| fail(&format!("open {path}: {e}")));
        recorded(serial, label, log)
    };
    Servos::new(transport)
}

fn recorded<T: Transport + 'static>(t: T, label: &str, log: &Option<Arc<ByteLog>>) -> Box<dyn Transport> {
    match log {
        Some(log) => Box::new(Recorder::new(t, label, log.clone())),
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
    let mut arm = Arm::new("arm", servos, cal);
    arm.check_present()?;
    let stop = ctrl_c_flag();
    let mut grid = Grid::new(hz);
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
    let mut arm = Arm::new(arm_id, servos, Calibration::identity(arm_id));
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
    let sweep = calibration::sweep(&mut arm, 30.0, |sweep| {
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
    calibration::check_centred(&sweep)?;

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
) -> Result<()> {
    let mut leader = Arm::new("leader", leader_servos, load_calibration(leader_cal, "leader", sim)?);
    let mut follower = Arm::new("follower", follower_servos, load_calibration(follower_cal, "follower", sim)?);
    leader.check_present()?;
    follower.check_present()?;
    for arm in [&mut leader, &mut follower] {
        let changed = arm.configure_readings()?;
        if changed > 0 {
            println!("{}: cleared the position-wrap bit on {changed} servos", arm.label());
        }
    }
    follower.configure_follower_with(gains)?;

    let options = TeleopOptions {
        hz,
        max_step_counts: (max_step > 0).then_some(max_step),
        max_ticks: (ticks > 0).then_some(ticks),
        read_actuators: actuators,
        lead,
    };
    let stop = ctrl_c_flag();
    println!(
        "teleop at {hz} Hz, max step {max_step} counts, P {}, grip torque {}%, lead {} ms (wrists {} ms); Ctrl-C to stop (follower torque goes off on exit)",
        gains.p,
        gains.grip_torque / 10,
        lead[Joint::ShoulderPan.index()].as_millis(),
        lead[Joint::WristFlex.index()].as_millis()
    );
    let mut held = [false; JOINT_COUNT];
    let mut steps = 0u64;
    let stats = teleop::run(&mut leader, &mut follower, options, stop, |step| {
        steps += 1;
        let events = match step {
            Ok(events) => events,
            Err(e) => return println!("tick {steps:>6}  ERROR {e}"),
        };
        let mut poses = [Pose::default(); 2];
        for event in events {
            match &event.message {
                Message::JointState(ArmRole::Leader, s) => poses[0] = s.pose,
                Message::JointState(ArmRole::Follower, s) => poses[1] = s.pose,
                Message::Tick(tick) => report_holds(tick, &mut held),
                _ => {}
            }
        }
        if steps.is_multiple_of((hz as u64).max(1)) {
            println!("tick {steps:>6}\n   leader   {}\n   follower {}", format_pose(&poses[0]), format_pose(&poses[1]));
        }
    })?;
    println!(
        "stopped after {} ticks: {} overruns, {} errors, worst tick work {:.2} ms",
        stats.ticks,
        stats.overruns,
        stats.errors,
        stats.worst_work_time.as_secs_f64() * 1e3
    );
    Ok(())
}

fn cmd_log_recover(input: &Path, output: &Path) -> Result<()> {
    let bytes = std::fs::read(input)?;
    // OUTPUT may be an existing recording, so never overwrite it.
    let file = std::fs::OpenOptions::new().write(true).create_new(true).open(output)?;
    let (_, summary) =
        so101::log::recover(&bytes, std::io::BufWriter::new(file)).map_err(|e| Error::Other(e.to_string()))?;
    match summary.truncated {
        Some(why) => {
            println!("recovered {} messages -> {} (input stopped early: {why})", summary.messages, output.display())
        }
        None => println!("input was complete; copied {} messages -> {}", summary.messages, output.display()),
    }
    Ok(())
}

fn report_holds(tick: &Tick, held: &mut [bool; JOINT_COUNT]) {
    for joint in Joint::ALL {
        let now = tick.clipped.iter().any(|c| c.joint == joint && c.limit == Limit::Stall);
        match (held[joint.index()], now) {
            (false, true) => {
                println!("{} blocked: holding at contact; back the leader off to resume", joint.name())
            }
            (true, false) => println!("{} released: following again", joint.name()),
            _ => {}
        }
        held[joint.index()] = now;
    }
}
