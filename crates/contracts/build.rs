//! Compiles the .proto files in pure Rust (protox, no protoc) into prost types, and saves the
//! encoded descriptor set so logs can embed the exact schemas they were written with.

use std::path::PathBuf;

// The .proto files live in proto/ at the repo root, shared by every language in the repo.
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
