# vexnav

## Building

`vexnav` depends on [`pros-rs`](https://github.com/pros-rs/pros-rs/), which is a `no_std` runtime library that runs on the V5 brain. This targets bare metal ARM, and thus requires some prerequisites.

- Grab a copy of the `arm-none-eabi` toolchain from https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
- Make sure you're on nightly rust, since we use some unstable features for floating point optimizations, as well as what's required by pros-rs.
- Download and install [`cargo-pros`](https://github.com/pros-rs/cargo-pros/).
- If you want to upload, you'll need the [PROS CLI](https://pros.cs.purdue.edu/v5/cli/index.html).
- `cargo pros build`