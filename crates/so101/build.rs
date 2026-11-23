//! Pure Rust (protox), so building needs no protoc. The descriptor set is saved so logs can embed
//! the exact schemas they were written with.

use std::path::PathBuf;

const ROOT: &str = "../../proto";
const PROTOS: [&str; 2] = ["../../proto/so101/v1/messages.proto", "../../proto/foxglove/CompressedImage.proto"];

fn main() -> Result<(), Box<dyn std::error::Error>> {
    for p in PROTOS {
        println!("cargo:rerun-if-changed={p}");
    }
    let fds = protox::compile(PROTOS, [ROOT])?;
    let out = PathBuf::from(std::env::var("OUT_DIR")?);
    std::fs::write(out.join("descriptors.binpb"), prost::Message::encode_to_vec(&fds))?;
    prost_build::Config::new().bytes(["."]).compile_fds(fds)?;
    Ok(())
}
