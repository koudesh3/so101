//! Replay a byte log as frames, pull out goal and present-position series,
//! and render them as an HTML report.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::path::Path;

use crate::robot::joint::Joint;
use crate::wire::protocol::{parse_frame, Frame, Instruction, ParseError, BROADCAST_ID};
use crate::wire::registers::{self, Register};
use crate::wire::transport::{read_log, Direction, LogRecord};

#[derive(Clone, Debug)]
pub struct DecodedFrame {
    pub t_us: u64,
    pub port: String,
    pub dir: Direction,
    pub bytes: Vec<u8>,
    pub frame: Frame,
    pub meaning: String,
}

#[derive(Clone, Debug)]
pub struct Sample {
    pub t_us: u64,
    pub id: u8,
    pub value: i32,
}

#[derive(Default, Debug)]
pub struct PortSeries {
    pub goals: BTreeMap<u8, Vec<Sample>>,
    pub present: BTreeMap<u8, Vec<Sample>>,
}

impl PortSeries {
    fn push_goal(&mut self, t_us: u64, id: u8, value: i32) {
        self.goals.entry(id).or_default().push(Sample { t_us, id, value });
    }
    fn push_present(&mut self, t_us: u64, id: u8, value: i32) {
        self.present.entry(id).or_default().push(Sample { t_us, id, value });
    }
}

#[derive(Default, Debug)]
pub struct Decoded {
    pub frames: Vec<DecodedFrame>,
    pub series: BTreeMap<String, PortSeries>,
    pub undecodable_bytes: usize,
}

// What the host last asked on a port, which is what makes a reply readable.
#[derive(Clone, Copy)]
enum Expectation {
    None,
    Read { reg: Option<Register> },
    SyncRead { reg: Option<Register>, ids: [u8; 32], count: usize, next: usize },
}

pub fn decode(records: &[LogRecord]) -> Decoded {
    let mut out = Decoded::default();
    let mut buffers: BTreeMap<(String, Direction), Vec<u8>> = BTreeMap::new();
    let mut expectations: BTreeMap<String, Expectation> = BTreeMap::new();

    for rec in records {
        let Some(dir) = Direction::parse(&rec.dir) else { continue };
        let buf = buffers.entry((rec.port.clone(), dir)).or_default();
        buf.extend(rec.bytes());
        loop {
            match parse_frame(buf) {
                Ok((frame, used)) => {
                    let bytes: Vec<u8> = buf.drain(..used).collect();
                    let expectation = expectations.entry(rec.port.clone()).or_insert(Expectation::None);
                    let series = out.series.entry(rec.port.clone()).or_default();
                    let meaning = match dir {
                        Direction::HostToBus => interpret_instruction(&frame, expectation, rec.t_us, series),
                        Direction::BusToHost => interpret_status(&frame, expectation, rec.t_us, series),
                    };
                    out.frames.push(DecodedFrame { t_us: rec.t_us, port: rec.port.clone(), dir, bytes, frame, meaning });
                }
                Err(ParseError::NoHeader { skip }) => {
                    out.undecodable_bytes += skip;
                    buf.drain(..skip);
                }
                Err(ParseError::BadChecksum { .. }) => {
                    out.undecodable_bytes += 2;
                    buf.drain(..2);
                }
                Err(ParseError::Incomplete(_)) => break,
            }
        }
    }
    out
}

fn reg_name(reg: Option<Register>, address: u8) -> String {
    reg.map(|r| r.name.to_string()).unwrap_or_else(|| format!("addr {address}"))
}

fn is_goal(reg: Option<Register>) -> bool {
    reg.is_some_and(|r| r.address == registers::GOAL_POSITION.address)
}

fn interpret_instruction(frame: &Frame, expectation: &mut Expectation, t_us: u64, series: &mut PortSeries) -> String {
    let target = if frame.id == BROADCAST_ID { "all".to_string() } else { format!("servo {}", frame.id) };
    let p = &frame.params;
    match frame.instruction() {
        Some(Instruction::Ping) => {
            *expectation = Expectation::Read { reg: None };
            format!("PING {target}")
        }
        Some(Instruction::Read) if p.len() >= 2 => {
            let reg = registers::by_address(p[0]);
            *expectation = Expectation::Read { reg };
            format!("READ {} ({} bytes) from {target}", reg_name(reg, p[0]), p[1])
        }
        Some(Instruction::Write) | Some(Instruction::RegWrite) if !p.is_empty() => {
            let reg = registers::by_address(p[0]);
            *expectation = Expectation::Read { reg: None };
            let value = reg.map(|r| r.decode(&p[1..]).to_string()).unwrap_or_else(|| format!("{:02X?}", &p[1..]));
            if is_goal(reg) && frame.id != BROADCAST_ID {
                series.push_goal(t_us, frame.id, reg.unwrap().decode(&p[1..]));
            }
            format!("WRITE {} = {value} to {target}", reg_name(reg, p[0]))
        }
        Some(Instruction::SyncRead) if p.len() >= 2 => {
            let reg = registers::by_address(p[0]);
            let mut ids = [0u8; 32];
            let count = (p.len() - 2).min(32);
            ids[..count].copy_from_slice(&p[2..2 + count]);
            *expectation = Expectation::SyncRead { reg, ids, count, next: 0 };
            format!("SYNC READ {} from ids {:?}", reg_name(reg, p[0]), &p[2..])
        }
        Some(Instruction::SyncWrite) if p.len() >= 2 => {
            let reg = registers::by_address(p[0]);
            let width = p[1] as usize;
            let mut parts = Vec::new();
            for block in p[2..].chunks_exact(1 + width) {
                let value = reg.map(|r| r.decode(&block[1..])).unwrap_or(0);
                parts.push(format!("{}={}", block[0], value));
                if is_goal(reg) {
                    series.push_goal(t_us, block[0], value);
                }
            }
            *expectation = Expectation::None;
            format!("SYNC WRITE {}: {}", reg_name(reg, p[0]), parts.join(" "))
        }
        Some(Instruction::Action) => "ACTION".to_string(),
        other => format!("instruction {other:?} params {:02X?}", p),
    }
}

fn interpret_status(frame: &Frame, expectation: &mut Expectation, t_us: u64, series: &mut PortSeries) -> String {
    let health = frame.error();
    let reg = match expectation {
        Expectation::Read { reg } => *reg,
        Expectation::SyncRead { reg, ids, count, next } => {
            let expected_id = if *next < *count { Some(ids[*next]) } else { None };
            *next += 1;
            if expected_id != Some(frame.id) {
                return format!("status from servo {} (out of order; expected {:?}) {health}", frame.id, expected_id);
            }
            *reg
        }
        Expectation::None => None,
    };
    if frame.params.is_empty() {
        return format!("status from servo {}: {health}", frame.id);
    }
    match reg {
        Some(r) if frame.params.len() >= r.width as usize => {
            let value = r.decode(&frame.params);
            if r.address == registers::PRESENT_POSITION.address {
                series.push_present(t_us, frame.id, value);
            }
            format!("servo {} {} = {value} ({health})", frame.id, r.name)
        }
        _ => format!("servo {} data {:02X?} ({health})", frame.id, frame.params),
    }
}

pub fn render_html(decoded: &Decoded, log_path: &Path, max_table_rows: usize) -> String {
    let mut h = String::new();
    let _ = write!(h, r#"<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>so101 log: {name}</title>
<style>
body{{font-family:system-ui,sans-serif;margin:1.5rem;color:#16202E;background:#F6F8FA;max-width:70rem}}
h1,h2{{font-weight:600}} h1{{font-size:1.5rem}} h2{{font-size:1.15rem;margin-top:2rem}}
table{{border-collapse:collapse;font-size:.85rem;width:100%}} th,td{{text-align:left;padding:.25rem .5rem;border-bottom:1px solid #D8DEE5;vertical-align:top}}
td.m{{font-family:ui-monospace,Menlo,monospace;white-space:nowrap}} .tx{{color:#9D174D}} .rx{{color:#1D4ED8}}
svg{{background:#fff;border:1px solid #D8DEE5;border-radius:4px;display:block;margin:.5rem 0 1rem}}
.small{{color:#51607A;font-size:.85rem}}
</style></head><body>
<h1>so101 byte log</h1><p class="small">{name}: {frames} frames decoded, {undec} undecodable bytes</p>
"#, name = log_path.display(), frames = decoded.frames.len(), undec = decoded.undecodable_bytes);

    for (port, series) in &decoded.series {
        let _ = write!(h, "<h2>Port: {port}</h2>");
        let goal_count: usize = series.goals.values().map(|v| v.len()).sum();
        let present_count: usize = series.present.values().map(|v| v.len()).sum();
        let _ = write!(h, "<p class=\"small\">{present_count} present-position samples, {goal_count} goal writes</p>");
        let ids: std::collections::BTreeSet<u8> = series.goals.keys().chain(series.present.keys()).copied().collect();
        for id in ids {
            let name = Joint::from_id(id).map(Joint::name).unwrap_or("?");
            h.push_str(&chart(&format!("servo {id} ({name})"), series.present.get(&id), series.goals.get(&id)));
        }
    }

    let _ = write!(h, "<h2>Frames (first {max_table_rows})</h2><table><tr><th>t (ms)</th><th>port</th><th>dir</th><th>bytes</th><th>meaning</th></tr>");
    for f in decoded.frames.iter().take(max_table_rows) {
        let hex: String = f.bytes.iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ");
        let _ = write!(
            h,
            "<tr><td class=\"m\">{:.3}</td><td>{}</td><td class=\"{d}\">{d}</td><td class=\"m\">{hex}</td><td>{}</td></tr>",
            f.t_us as f64 / 1000.0,
            f.port,
            escape(&f.meaning),
            d = f.dir.as_str()
        );
    }
    h.push_str("</table></body></html>");
    h
}

fn chart(title: &str, present: Option<&Vec<Sample>>, goals: Option<&Vec<Sample>>) -> String {
    let all: Vec<&Sample> = present.into_iter().flatten().chain(goals.into_iter().flatten()).collect();
    if all.is_empty() {
        return String::new();
    }
    let t0 = all.iter().map(|s| s.t_us).min().unwrap() as f64;
    let t1 = all.iter().map(|s| s.t_us).max().unwrap() as f64;
    let v0 = all.iter().map(|s| s.value).min().unwrap() as f64;
    let v1 = all.iter().map(|s| s.value).max().unwrap() as f64;
    let (w, hgt, pad) = (900.0, 220.0, 40.0);
    let span_t = (t1 - t0).max(1.0);
    let span_v = (v1 - v0).max(1.0);
    let x = |t: u64| pad + (t as f64 - t0) / span_t * (w - 2.0 * pad);
    let y = |v: i32| hgt - pad + (v0 - v as f64) / span_v * (hgt - 2.0 * pad);
    let path = |s: &Vec<Sample>| -> String {
        s.iter()
            .enumerate()
            .map(|(i, p)| format!("{}{:.1},{:.1}", if i == 0 { "M" } else { "L" }, x(p.t_us), y(p.value)))
            .collect::<Vec<_>>()
            .join(" ")
    };
    let mut svg = format!(
        "<svg viewBox=\"0 0 {w} {hgt}\" width=\"100%\"><text x=\"{pad}\" y=\"22\" font-size=\"14\" font-weight=\"600\">{title}</text>\
         <text x=\"{}\" y=\"22\" font-size=\"12\" text-anchor=\"end\" fill=\"#51607A\">{} s, ticks {} to {}</text>",
        w - pad,
        format_args!("{:.2}", span_t / 1e6),
        v0 as i32,
        v1 as i32
    );
    if let Some(g) = goals {
        let _ = write!(svg, "<path d=\"{}\" fill=\"none\" stroke=\"#9D174D\" stroke-width=\"1.5\" stroke-dasharray=\"4 3\"/>", path(g));
    }
    if let Some(p) = present {
        let _ = write!(svg, "<path d=\"{}\" fill=\"none\" stroke=\"#1D4ED8\" stroke-width=\"1.8\"/>", path(p));
    }
    let _ = write!(
        svg,
        "<text x=\"{pad}\" y=\"{}\" font-size=\"11\" fill=\"#9D174D\">dashed: goal written</text><text x=\"{}\" y=\"{}\" font-size=\"11\" fill=\"#1D4ED8\">solid: present position read</text></svg>",
        hgt - 10.0,
        pad + 150.0,
        hgt - 10.0
    );
    svg
}

fn escape(s: &str) -> String {
    s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;")
}

pub fn report_from_log(log_path: &Path, out_path: &Path, max_table_rows: usize) -> Result<Decoded, String> {
    let records = read_log(log_path).map_err(|e| format!("{}: {e}", log_path.display()))?;
    let decoded = decode(&records);
    std::fs::write(out_path, render_html(&decoded, log_path, max_table_rows)).map_err(|e| e.to_string())?;
    Ok(decoded)
}
