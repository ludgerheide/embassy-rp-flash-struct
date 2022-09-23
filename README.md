# Embassy RP Skeleton
The **Embassy RP Skeleton** repository is a project template intended as a starting point for developing your own
firmware for the [`rp2040`][1] based on the [`embassy`][2] asynchronous embedded development framework for [Rust][3].

It includes all of the [`knurling-rs`][4] tooling ([`defmt`][5], [`defmt-rtt`][5], [`panic-probe`][5], [`flip-link`][6],
[`probe-run`][7]) to enhance the embedded development process.

The default [`cargo`][8] runner is configured as [`probe-run`][7], so you can build, flash and run your firmware _with_
output from the device via a [`probe-rs`][9] compatible debug probe with the command:

```shell
$ cargo run
```

If you want to use a different runner with your debugger (e.g., [`cargo-embed`][10], [`probe-rs-debugger`][11], etc.) or
if you _aren't_ using a debugger and want the runner to flash the firmware via USB (e.g., [`elf2uf2-rs`][12],
[`picotool`][13], etc.) then see: [Alternative Runners][14]

## Table of Contents
1. [Requirements](#requirements)
2. [Setup](#setup)
    1. [System Setup](#system-setup)
    2. [Probe Setup](#probe-setup)
    3. [Hardware Setup](#hardware-setup)
3. [Usage](#usage)
6. [Appendix](#appendix)

## Requirements
* Ubuntu
* Raspberry Pi Pico
* Debug Probe (*or* another Raspberry Pi Pico)
* Rust Toolchain ([`cargo`][8], [`rustup`][15])

## Setup
### System Setup
1. Install [Rust][3] and [`cargo`][8] using [`rustup`][15]
```shell
# Install `rustup` for Rust Toolchain
$ curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

2. Install Cortex-M Target Toolchain Support for [`Rust`][3]
```shell
# Install `thumbv6m-none-eabi` Target for `rp2040`
$ rustup target add thumbv6m-none-eabi
```

3. Install [`probe-run`][7]
```shell
# Install Linux Dependencies
$ sudo apt install -y libusb-1.0-0-dev libudev-dev

# Install `probe-run`
$ cargo install probe-run

# (Optional) Install `udev` Rules and Reload
$ sudo curl https://probe.rs/files/69-probe-rs.rules -o /etc/udev/rules.d/69-probe-rs.rules
$ sudo udevadm control --reload
$ sudo udevadm trigger

# (Optional) Add User to `plugdev` Group
$ sudo usermod -aG plugdev $USER
```

4. Install [`flip-link`][6]
```shell
# Install `flip-link`
$ $ cargo install flip-link
```

### Probe Setup
You can use a Raspberry Pi Pico as a CMSIS-DAP debug probe.

1. Download CMSIS-DAP debugger firmware [`DapperMime`][16] for the Raspberry Pi Pico
2. Boot the Raspberry Pi Pico in "Bootloader Mode" by holding the _BOOTSEL_ button while plugging it in
3. Open the mounted Raspberry Pi Pico storage device
4. Copy the `raspberry_pi_pico-DapperMime.uf2` onto the Raspberry Pi Pico
5. Firmware will be flashed to the Raspberry Pi Pico and it will disconnect

Any [`probe-rs`][9] compatible debug probe can be used with [`probe-run`][7]. For a short list of alternative
compatible debug probes see: [Alternative Debug Probes][17].

### Hardware Setup
#### Connecting the Raspberry Pi Pico Debug Probe
The diagram below shows the wiring loom between Raspberry Pi Pico A (left) and Raspberry Pi Pico B (right), configuring
Raspberry Pi Pico A as a debug probe.

![Raspberry Pi Pico Debug Probe Wiring][18]

The connections shown in the diagram above are listed below.

```
Pico A GND -> Pico B GND
Pico A GP2 -> Pico B SWCLK
Pico A GP3 -> Pico B SWDIO
Pico A GP4/UART1 TX -> Pico B GP1/UART0 RX
Pico A GP5/UART1 RX -> Pico B GP0/UART0 TX
Pico A VSYS -> Pico B VSYS
```

For more information on the wiring loom and its connections, see:
[Getting Started with Raspberry Pi Pico > Appendix A > Picoprobe Wiring][19]

#### Raspberry Pi Pico Dev Board
Alternatively, a custom printed Raspberry Pi Pico Dev Board can be used to enhance development, which includes:

* Debug Probe Host (Raspberry Pi Pico)
* Detachable Target (Raspberry Pi Pico)
* Serial Interface
* Reset Button
* Breakout Pins
* Selection of _VSys_ or _VBus_ Power Sources

![Raspberry Pi Pico Dev Board Debugging][20]

For more information on printing your own custom Raspberry Pi Pico Dev Board, see:
[Raspberry Pi Pico Dev Board][21]

## Usage
#### Running
To run the firmware in debug mode:
```shell
$ cargo run
```

To run the firmware in release mode:
```shell
$ cargo run --release
```

#### Logging
To change the default [`defmt`][5] log level, see `.cargo/config.toml`:
```toml
[env]
DEFMT_LOG = "trace"
```

You can also set the log level inline:
```shell
$ DEFMT_LOG=debug cargo run
$ DEFMT_LOG=error cargo run --release
```

## Appendix
#### Documentation
* [Raspberry Pi Pico][1]
* [Rust][3]
* [Cargo][8]
* [Rustup][15]
* [Embassy][2]
* [Knurling-RS `defmt`][5]
* [Knurling-RS `flip-link`][6]
* [Knurling-RS `probe-run`][7]
* [Probe-RS `cargo-embed`][10]
* [Probe-RS `probe-rs-debugger`][11]
* [Raspberry Pi Pico `elf2uf2`][12]
* [Raspberry Pi Pico `picotool`][13]
* [CMSIS-DAP Firmware `DapperMime`][16]

#### Resources
* [Rust Embedded Book][22]
* [Awesome Embedded Rust][23]
* [Getting Started with Raspberry Pi Pico][24]
* [Ferrous Systems Embedded Training][25]
* [Ferrous Systems Embedded Teaching Material][26]
* [RP-RS App Template][27]
* [RP-RS Alternative Debug Probes][17]
* [RP-RS Alternative Runners][14]
* [Knurling-RS App Template][4]
* [Probe-RS Probe Setup][9]
* [Raspberry Pi Pico Dev Board][21]


<!-- Reference -->
[1]: https://www.raspberrypi.com/documentation/microcontrollers/rp2040.html
[2]: https://embassy.dev/dev/index.html
[3]: https://www.rust-lang.org/
[4]: https://github.com/knurling-rs/app-template
[5]: https://github.com/knurling-rs/defmt
[6]: https://github.com/knurling-rs/flip-link
[7]: https://github.com/knurling-rs/probe-run
[8]: https://doc.rust-lang.org/cargo/
[9]: https://probe.rs/docs/getting-started/probe-setup/
[10]: https://github.com/probe-rs/cargo-embed
[11]: https://github.com/probe-rs/vscode
[12]: https://github.com/JoNil/elf2uf2-rs
[13]: https://github.com/raspberrypi/picotool
[14]: https://github.com/rp-rs/rp2040-project-template#alternative-runners
[15]: https://rustup.rs/
[16]: https://github.com/majbthrd/DapperMime
[17]: https://github.com/rp-rs/rp2040-project-template/blob/main/debug_probes.md
[18]: https://user-images.githubusercontent.com/12226419/134785445-5f651d5a-eda9-4e94-8860-d2ef619dc27a.png
[19]: https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf#%5B%7B%22num%22%3A64%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C115%2C696.992%2Cnull%5D
[20]: https://timsavage.github.io/rpi-pico-devboard/assets/images/devboard-debugging.jpg
[21]: https://timsavage.github.io/rpi-pico-devboard/
[22]: https://docs.rust-embedded.org/book/
[23]: https://github.com/rust-embedded/awesome-embedded-rust
[24]: https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf
[25]: https://embedded-trainings.ferrous-systems.com/
[26]: https://github.com/ferrous-systems/teaching-material
[27]: https://github.com/rp-rs/rp2040-project-template
