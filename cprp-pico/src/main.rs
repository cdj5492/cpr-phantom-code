#![no_std]
#![no_main]

use defmt::info;
use devices::ads1015::{self, Ads1015, AdsAddressOptions};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::Output;
use embassy_rp::i2c::I2c;
use embassy_rp::peripherals::{I2C0, I2C1, USB};
use embassy_rp::usb::Driver as UsbDriver;
// use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::msos::{self, windows_version};
use embassy_usb::{Builder, Config};
// use embassy_sync::signal::Signal;
use heapless::Vec;
use {defmt_rtt as _, panic_probe as _};

#[allow(dead_code)]
mod devices;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<I2C0>;
    I2C1_IRQ => embassy_rp::i2c::InterruptHandler<I2C1>;
});

bind_interrupts!(
    struct I2CIrqs {}
);

// This is a randomly generated GUID to allow clients on Windows to find our device
const DEVICE_INTERFACE_GUIDS: &[&str] = &["{AFB9A6FB-30BA-44BC-9232-806CFC875321}"];

// Number of ADC boards on each bus
const ADC_COUNT_B1: usize = 1;
const ADC_COUNT_B2: usize = 0;

const MAGIC: u16 = 0xAA55;

const PACKET_SIZE: usize = 2+1+4 + (ADC_COUNT_B1 * 8 + ADC_COUNT_B2 * 8) as usize;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Create the driver, from the HAL.
    let driver = UsbDriver::new(p.USB, Irqs);

    // Create embassy-usb Config
    let mut config = Config::new(0xf569, 0x0001);
    config.manufacturer = Some("CPR Therapeutics");
    config.product = Some("CPR Phantom");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut msos_descriptor = [0; 256];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    // Add the Microsoft OS Descriptor (MSOS/MOD) descriptor.
    // We tell Windows that this entire device is compatible with the "WINUSB" feature,
    // which causes it to use the built-in WinUSB driver automatically, which in turn
    // can be used by libusb/rusb software without needing a custom driver or INF file.
    // In principle you might want to call msos_feature() just on a specific function,
    // if your device also has other functions that still use standard class drivers.
    builder.msos_descriptor(windows_version::WIN8_1, 0);
    builder.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    builder.msos_feature(msos::RegistryPropertyFeatureDescriptor::new(
        "DeviceInterfaceGUIDs",
        msos::PropertyData::RegMultiSz(DEVICE_INTERFACE_GUIDS),
    ));

    let class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder.
    let mut usb = builder.build();

    let mut config = embassy_rp::i2c::Config::default();
    config.frequency = 1_000_000; // 1 MHz

    let mut i2c0 = I2c::new_async(p.I2C0, p.PIN_21, p.PIN_20, Irqs, config);
    let mut i2c1 = I2c::new_async(p.I2C1, p.PIN_23, p.PIN_22, Irqs, config);

    // boards on I2C0
    let mut expansion_boards_b1: [Ads1015; ADC_COUNT_B1] =
        [Ads1015::new(AdsAddressOptions::Addr48)];
    let mut expansion_boards_b2: [Ads1015; ADC_COUNT_B2] = [];

    // indicator led
    let mut indicator = Output::new(p.PIN_25, false.into());

    // Run the USB device.
    let usb_fut = usb.run();

    // packet to send
    let mut packet = Vec::<u8, PACKET_SIZE>::new();

    info!("Connecting ADCs...");

    for ads in expansion_boards_b1.iter_mut() {
        while let Err(_e) = ads.begin(&mut i2c0).await {
            info!("Failed to connect to ADC on bus 0. Retrying...");
            embassy_time::Timer::after_millis(250).await;
            indicator.set_high();
            embassy_time::Timer::after_millis(250).await;
            indicator.set_low();
        }
        ads.set_sample_rate(ads1015::constants::CONFIG_RATE_3300HZ);
        info!("Connected to ADC");
    }
    for ads in expansion_boards_b2.iter_mut() {
        while let Err(_e) = ads.begin(&mut i2c1).await {
            info!("Failed to connect to ADC on bus 1. Retrying...");
            embassy_time::Timer::after_millis(250).await;
            indicator.set_high();
            embassy_time::Timer::after_millis(250).await;
            indicator.set_low();
        }
        ads.set_sample_rate(ads1015::constants::CONFIG_RATE_3300HZ);
        info!("Connected to ADC");
    }

    // let b1_signal: Signal<ThreadModeRawMutex, _> = Signal::new();

    let send_adc_fut = async {
        // class.wait_connection().await;
        let (mut sender, mut reader) = class.split();
        info!("Waiting for USB connection...");
        sender.wait_connection().await;
        reader.wait_connection().await;
        info!("Connected via USB.");
        // let mut buf = [0; 64];

        // let mut values = [0u16; ADC_COUNT_B1 * 4 + ADC_COUNT_B2 * 4];

        loop {
            packet.clear();
            let now = embassy_time::Instant::now();
            let timestamp = now.as_millis() as u32;

            // build packet header (could refactor to move outside loop)
            // MAGIC (2 bytes), data length (1 bytes), timestamp (4 bytes)
            packet.extend(MAGIC.to_le_bytes());
            let _ = packet.push((ADC_COUNT_B1 * 8 + ADC_COUNT_B2 * 8 + 4) as u8);
            packet.extend(timestamp.to_le_bytes());

            // get single-ended adc values from each channel on each board on each bus
            for ads in expansion_boards_b1.iter_mut() {
                for j in 0..4 {
                    let data = match ads.get_single_ended(&mut i2c0, j as u8).await {
                        Ok(data) => data,
                        Err(_) => continue,
                    };
                    packet.extend(data.to_le_bytes());
                }
            }
            for ads in expansion_boards_b2.iter_mut() {
                for j in 0..4 {
                    let data = match ads.get_single_ended(&mut i2c1, j as u8).await {
                        Ok(data) => data,
                        Err(_) => continue,
                    };
                    packet.extend(data.to_le_bytes());
                }
            }

            // send packet to host
            sender.write_packet(&packet).await.unwrap();
        }
    };


    // Run everything concurrently.
    join(usb_fut, send_adc_fut).await;


    // shuold never get here. Blink led if it somehow happens anyway
    loop {
        indicator.set_high();
        embassy_time::Timer::after_millis(500).await;
        indicator.set_low();
        embassy_time::Timer::after_millis(500).await;
    }
}
