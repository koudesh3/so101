# The Linux build of the crate, for `just linux ...` on the laptop: same Rust as mise.toml, plus
# libclang (the v4l crate generates its V4L2 bindings with bindgen at build time) and uv (one
# session test reads a log from Python) and ffmpeg (the scene video tests). The kernel headers
# come with the Rust image.
FROM rust:1.90-bookworm
RUN apt-get update && apt-get install -y --no-install-recommends libclang-dev clang ffmpeg \
    && rm -rf /var/lib/apt/lists/* \
    && rustup component add clippy rustfmt \
    && cargo install --locked just
COPY --from=ghcr.io/astral-sh/uv:0.9 /uv /usr/local/bin/uv
