//! Command-line front end. Every command takes `--sim` for simulated servos
//! and `--log FILE` to record every byte on every bus.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};

use clap::{Parser, Subcommand};

use so101::robot::arm::Arm;
use so101::device::bus::Bus;
use so101::control::calibrate;
use so101::robot::calibration::ArmCalibration;
use so101::robot::joint::{Joint, Values, IDS};
use so101::device::sim::SimulatedBus;
use so101::control::teleop::{self, TeleopOptions};
use so101::control::ticker::Ticker;
use so101::wire::transport::{ByteLog, Recorder, SerialTransport, Transport};
use so101::report::visualize;
use so101::{Error, Result};

#[derive(Parser)]
#[command(name = "so101", about = "Calibrate, monitor and teleoperate an SO-101 arm pair from scratch")]
struct Cli {
    /// Record every byte on every bus to this JSON-lines file.
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
    /// Print live positions and telemetry until Ctrl-C.
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
        #[arg(long, default_value_t = 30.0)]
        hz: f64,
        /// Largest goal change per tick, in ticks (0 disables).
        #[arg(long, default_value_t = 200)]
        max_step: i32,
        /// Stop after this many ticks (0 runs until Ctrl-C).
        #[arg(long, default_value_t = 0)]
        ticks: u64,
    },
    /// Decode a byte log into an HTML report with charts and annotated frames.
    Visualize {
        #[arg(long)]
        log_file: PathBuf,
        #[arg(long)]
        out: PathBuf,
        #[arg(long, default_value_t = 400)]
        rows: usize,
    },
}

// Which bus a command is opening; decides the log label and the simulator's behaviour.
#[derive(Clone, Copy)]
enum Role {
    Single,
    Leader,
    Follower,
}

impl Role {
    fn label(self) -> &'static str {
        match self {
            Role::Single => "bus",
            Role::Leader => "leader",
            Role::Follower => "follower",
        }
    }

    fn simulated(self) -> SimulatedBus {
        match self {
            Role::Single | Role::Leader => SimulatedBus::waving_arm(self.label()),
            Role::Follower => SimulatedBus::arm(self.label()),
        }
    }
}

fn main() {
    let cli = Cli::parse();
    let log = cli.log.as_ref().map(|p| ByteLog::create(p).unwrap_or_else(|e| fail(&format!("cannot create log {}: {e}", p.display()))));
    let open = |role: Role, port: Option<String>| open_bus(role, port, cli.sim, &log);
    let result = match cli.command {
        Command::Ping { port, id } => cmd_ping(open(Role::Single, port), id),
        Command::Scan { port } => cmd_scan(open(Role::Single, port)),
        Command::Monitor { port, calibration, hz } => cmd_monitor(open(Role::Single, port), calibration, hz),
        Command::Calibrate { port, arm_id, out } => cmd_calibrate(open(Role::Single, port), &arm_id, &out),
        Command::Teleop { leader, follower, leader_calibration, follower_calibration, hz, max_step, ticks } => cmd_teleop(
            open(Role::Leader, leader),
            open(Role::Follower, follower),
            leader_calibration,
            follower_calibration,
            hz,
            max_step,
            ticks,
        ),
        Command::Visualize { log_file, out, rows } => cmd_visualize(&log_file, &out, rows),
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

fn open_bus(role: Role, port: Option<String>, sim: bool, log: &Option<Arc<ByteLog>>) -> Bus {
    let label = role.label();
    let transport: Box<dyn Transport> = if sim {
        recorded(role.simulated(), label, log)
    } else {
        let path = port.unwrap_or_else(|| fail(&format!("--port for {label} is required (or pass --sim)")));
        let serial = SerialTransport::open(&path).unwrap_or_else(|e| fail(&format!("open {path}: {e}")));
        recorded(serial, label, log)
    };
    Bus::new(transport)
}

fn recorded<T: Transport + 'static>(t: T, label: &str, log: &Option<Arc<ByteLog>>) -> Box<dyn Transport> {
    match log {
        Some(log) => Box::new(Recorder::new(t, label, log.clone())),
        None => Box::new(t),
    }
}

fn load_calibration(path: Option<PathBuf>, label: &str) -> Result<ArmCalibration> {
    match path {
        Some(p) => ArmCalibration::load(&p),
        None => {
            eprintln!("note: no calibration for {label}, using identity (raw ticks, centred at 2048)");
            Ok(ArmCalibration::identity(label))
        }
    }
}

fn ctrl_c_flag() -> Arc<AtomicBool> {
    let stop = Arc::new(AtomicBool::new(false));
    let s = stop.clone();
    let _ = ctrlc::set_handler(move || s.store(true, Ordering::Relaxed));
    stop
}

fn format_values(values: &Values) -> String {
    Joint::ALL
        .iter()
        .zip(values)
        .map(|(j, v)| format!("{}={v:7.2}{}", j.name(), j.unit().symbol()))
        .collect::<Vec<_>>()
        .join("  ")
}

fn cmd_ping(mut bus: Bus, id: u8) -> Result<()> {
    let servo = bus.identify(id)?;
    let model = if servo.is_sts3215() { "STS3215" } else { "unexpected" };
    println!(
        "servo {id} answered on {}: model {} ({model}), present position {} ticks",
        bus.name(),
        servo.model,
        servo.present_position
    );
    Ok(())
}

fn cmd_scan(mut bus: Bus) -> Result<()> {
    let found = bus.scan();
    if found.is_empty() {
        println!("no servos answered on {}", bus.name());
        return Ok(());
    }
    println!("servos answering on {}: {:?}", bus.name(), found);
    let missing: Vec<u8> = IDS.iter().copied().filter(|id| !found.contains(id)).collect();
    if !missing.is_empty() {
        println!("missing from the expected {IDS:?}: {missing:?}");
    }
    Ok(())
}

fn cmd_monitor(bus: Bus, calibration: Option<PathBuf>, hz: f64) -> Result<()> {
    let cal = load_calibration(calibration, "monitor")?;
    let mut arm = Arm::new("arm", bus, cal);
    arm.check_present()?;
    let stop = ctrl_c_flag();
    let mut ticker = Ticker::new(hz);
    println!("monitoring {} at {hz} Hz; Ctrl-C to stop", arm.bus_name());
    while !stop.load(Ordering::Relaxed) {
        ticker.wait();
        let telemetry = arm.read_telemetry()?;
        let ticks = std::array::from_fn(|i| telemetry[i].position_ticks);
        let values = arm.ticks_to_values(&ticks);
        println!("\x1b[2J\x1b[H{:<14}{:>7}{:>9}{:>7}{:>7}{:>7}{:>6}{:>8}", "joint", "ticks", "value", "speed", "load", "volt", "temp", "status");
        for (t, value) in telemetry.iter().zip(values) {
            println!(
                "{:<14}{:>7}{:>8.1}{}{:>7}{:>7}{:>6.1}V{:>5}C{:>8}",
                t.joint.name(),
                t.position_ticks,
                value,
                t.joint.unit().symbol(),
                t.speed,
                t.load,
                t.voltage_tenths as f64 / 10.0,
                t.temperature_c,
                t.status,
            );
        }
    }
    Ok(())
}

fn cmd_calibrate(bus: Bus, arm_id: &str, out: &Path) -> Result<()> {
    let mut arm = Arm::new(arm_id, bus, ArmCalibration::identity(arm_id));
    arm.check_present()?;
    let enter = stdin_lines();

    println!("Torque is off. Move every joint to the MIDDLE of its travel, then press Enter.");
    arm.torque_off()?;
    enter.recv().map_err(|_| Error::Other("stdin closed".into()))?;
    let offsets = arm.recentre()?;
    println!("homing offsets written: {offsets:?}");

    println!("Now move every joint slowly through its FULL range (and open/close the gripper). Press Enter when done.");
    let sweep = calibrate::sweep(&mut arm, 30.0, |sweep| {
        print!("\r");
        for (joint, (lo, hi)) in Joint::ALL.iter().zip(sweep.min.iter().zip(sweep.max)) {
            print!("{}:{lo}..{hi}  ", &joint.name()[..3]);
        }
        use std::io::Write as _;
        let _ = std::io::stdout().flush();
        enter.try_recv().is_err()
    })?;
    println!();

    let cal = ArmCalibration::from_sweep(arm_id, offsets, sweep.min, sweep.max);
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
    leader_bus: Bus,
    follower_bus: Bus,
    leader_cal: Option<PathBuf>,
    follower_cal: Option<PathBuf>,
    hz: f64,
    max_step: i32,
    ticks: u64,
) -> Result<()> {
    let mut leader = Arm::new("leader", leader_bus, load_calibration(leader_cal, "leader")?);
    let mut follower = Arm::new("follower", follower_bus, load_calibration(follower_cal, "follower")?);
    leader.check_present()?;
    follower.check_present()?;
    follower.configure_follower()?;

    let options = TeleopOptions {
        hz,
        max_step_ticks: (max_step > 0).then_some(max_step),
        max_ticks: (ticks > 0).then_some(ticks),
    };
    let stop = ctrl_c_flag();
    println!("teleop at {hz} Hz, max step {max_step} ticks; Ctrl-C to stop (follower torque goes off on exit)");
    let stats = teleop::run(&mut leader, &mut follower, options, stop, |r| {
        if r.tick.index % (hz as u64).max(1) == 0 || r.error.is_some() {
            match &r.error {
                Some(e) => println!("tick {:>6}  ERROR {e}", r.tick.index),
                None => println!(
                    "tick {:>6}  work {:>5.1} ms  late {:>5.1} ms\n   leader   {}\n   follower {}",
                    r.tick.index,
                    r.work_time.as_secs_f64() * 1e3,
                    r.tick.lateness.as_secs_f64() * 1e3,
                    format_values(&r.leader_values),
                    format_values(&r.follower_values)
                ),
            }
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

fn cmd_visualize(log_file: &Path, out: &Path, rows: usize) -> Result<()> {
    let decoded = visualize::report_from_log(log_file, out, rows)?;
    println!("{} frames decoded, {} undecodable bytes -> {}", decoded.frames.len(), decoded.undecodable_bytes, out.display());
    for (port, s) in &decoded.series {
        println!(
            "  {port}: {} goal writes, {} present-position samples",
            s.goals.values().map(Vec::len).sum::<usize>(),
            s.present.values().map(Vec::len).sum::<usize>()
        );
    }
    Ok(())
}
