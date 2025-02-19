#![no_std]
#![no_main]

use embedded_hal::{delay::DelayNs, digital::OutputPin};
// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
// use panic_halt as _;

// use embedded_io::Write;
// Alias for our HAL crate
use rp235x_hal as hal;

// Some things we need
use core::fmt::Write;
use heapless::String;
use embedded_hal_0_2::{adc::OneShot};

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::{SerialPort, USB_CLASS_CDC};

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

// panic handler
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    // Grab our singleton objects
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let _clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.gpio25.into_push_pull_output();
    led_pin.set_low().unwrap();
    loop {}
}

// Entry point to bare-metal applications.
#[hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    // Timer for reporting the current time since program start
    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USB,
        pac.USB_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    // TODO: Replace with actual VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("CPR Therapeutics")
            .product("CPR Phantom")
            .serial_number("super unique and interesting serial number")])
        .unwrap()
        .device_class(USB_CLASS_CDC) // from: https://www.usb.org/defined-class-codes
        .build();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Enable ADC
    let mut adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);

    // Enable the temperature sense channel
    // let mut temperature_sensor = adc.take_temp_sensor().unwrap();

    // Configure GPIO26 as an ADC input
    let mut adc_pin_0 = hal::adc::AdcPin::new(pins.gpio26).unwrap();
    let mut adc_pin_1 = hal::adc::AdcPin::new(pins.gpio27).unwrap();

    // led pin to indicate the program is running
    let mut led_pin = pins.gpio25.into_push_pull_output();
    led_pin.set_high().unwrap();


    loop {
        // limit to 1000Hz
        timer.delay_us(1000);


        // must be called at least every 10 ms
        usb_dev.poll(&mut [&mut serial]);

        // get current time in microseconds
        let time_micros = timer.get_counter().duration_since_epoch().to_micros();

        // Read the raw ADC counts from the temperature sensor channel.
        let adc1_val: u16 = adc.read(&mut adc_pin_0).unwrap();
        let adc2_val: u16 = adc.read(&mut adc_pin_1).unwrap();
        let adc1_val = adc1_val as i16;
        let adc2_val = adc2_val as i16;
        let diff = adc2_val.wrapping_sub(adc1_val);

        let mut text: String<64> = String::new();
        writeln!(&mut text, "{},{},{},{}\n", time_micros, diff, adc1_val, adc2_val).unwrap();

        // let raw_bytes = [
        //     time_micros as u8, (time_micros >> 8) as u8, (time_micros >> 16) as u8, (time_micros >> 24) as u8,
        //     ',' as u8,
        //     diff as u8, (diff >> 8) as u8,
        //     '\n' as u8,
        // ];

        let buf = text.as_bytes();
        let mut offset = 0;
        while offset < buf.len() {
            match serial.write(&buf[offset..]) {
                Ok(n) => offset += n, // advance by the number of bytes written
                Err(UsbError::WouldBlock) => {
                    usb_dev.poll(&mut [&mut serial]);
                }
                Err(e) => {
                    // Handle other errors if necessary, or just break out/log.
                }
            }
        }
    }
}
