# Project Structure

This project contains embedded rust code to run on the device, as well as code for forwarding device packets on to PlotJuggler and run various calibration algorithms.

The forwarding code can be found in the `forward_to_plotjuggler` folder and the embedded rust code in the `cprp-pico` folder.

# CPRP-PICO

## Setup

Has been tested on WSL and native Windows.

Requires rust, as well as picotool. If using a debug probe, install probe-rs as well
To install picotools, follow directions at `https://github.com/raspberrypi/picotool`
To install rust, follow directions at `https://www.rust-lang.org/learn/get-started`.
To install probe-rs, follow directions at `https://probe.rs/docs/getting-started/installation/`

Install the target with `rustup target add thumbv8m-none-eabi`

## Usage

### WSL

Prepare the device for programming by holding down the BOOTSEL button while plugging it into the computer.

Open an administrator powershell window and use usbipd to connect to the device.

Find the device

`usbipd list`

Using the device id found with `usbipd list`, bind the device

`usbipd bind --force --busid <device id>`

Attach to WSL

`usbip attach --busid <device id> --wsl`

In WSL, cd to the `cprp-pico` directory and run `cargo run --target thumbv8m-none-eabi`

### Windows

Prepare the device for programming by holding down the BOOTSEL button while plugging it into the computer.

cd into the cprp-pico directory and run `cargo run --release`

Alternatively, if using a debug probe for upload, edit the file `.cargo/config.toml` and change the line `runner = "picotool load -u -v -x -t elf"` to `runner = "probe-rs run --chip RP235x"`. Make sure

# forward_to_plotjuggler

Install plotjuggler: https://github.com/facontidavide/PlotJuggler

cd into the `forward_to_plotjuggler` directory and run `cargo run --release`

Open PlotJuggler, then navigate to streaming on the left. Select `UDP Server` from the dropdown, then click start. Change the Message Protocol to JSON, then click the checkmark labelled `use field as timestamp if available to enable it` and enter `t` into the message box.
