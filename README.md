# vexnav

## Building

`vexnav` depends on `vex-rt`, which is a `no_std` runtime library that runs on the V5 brain. This targets bare metal ARM, and thus requires some prerequisites.

- Grab a copy of the `arm-none-eabi` toolchain from https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
- > Be sure to check the option in the installer to add the toolchain to `PATH`.
- If you don't have `libclang`, install `libclang`/llvm from https://releases.llvm.org/
- `rustup target add armv7a-none-eabi`
- `cargo build`
