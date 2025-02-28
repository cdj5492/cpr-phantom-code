# Project Structure

This project contains embedded rust code to run on the device, as well as a python GUI to visualize and process raw data sent via USB

The GUI can be found in the `cprp-gui` folder and the rust code in the `cprp-pico` folder.

# CPRP-PICO

## Setup

Has been tested on WSL, but not windows.

Requires rust, as well as picotool.
To install picotools, follow directions at `https://github.com/raspberrypi/picotool`
To install rust, follow directions at `https://www.rust-lang.org/learn/get-started`.

Install the target with `rustup target add thumbv8m-none-eabi`

## Usage

Prepare the device for programming by holding down the BOOTSEL button while plugging it into the computer.

Open an administrator powershell window and use usbipd to connect to the device.

Find the device

`usbipd list`

Using the device id found with `usbipd list`, bind the device

`usbipd bind --force --busid <device id>`

Attach to WSL

`usbip attach --busid <device id> --wsl`

In WSL, cd to the `cprp-pico` directory and run `cargo run --target thumbv8m-none-eabi`

# CPRP-GUI

## TODO: Documentation cannot be written currently as it is in a state of flux


