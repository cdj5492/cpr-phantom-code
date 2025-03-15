#![no_std]
#![no_main]

use core::fmt::Write;
use defmt::info;
use devices::ads1015::{Ads1015, AdsAddressOptions};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::Output;
use embassy_rp::i2c::I2c;
use embassy_rp::peripherals::{ADC, I2C0, I2C1, USB};
use embassy_rp::usb::Driver as UsbDriver;
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State, USB_CLASS_CDC};
use embassy_usb::driver::{Driver, Endpoint, EndpointIn, EndpointOut};
use embassy_usb::msos::{self, windows_version};
use embassy_usb::{Builder, Config};
use heapless::String;
use {defmt_rtt as _, panic_probe as _};
use bytemuck::cast_slice;

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

    // TODO: Up frequency to 1 mhz
    let config = embassy_rp::i2c::Config::default();

    let mut i2c = I2c::new_async(p.I2C0, p.PIN_21, p.PIN_20, Irqs, config);

    // boards on I2C0
    let mut expansion_boards_b1: [Ads1015; ADC_COUNT_B1] = [Ads1015::new(AdsAddressOptions::Addr48)];
    let mut expansion_boards_b2: [Ads1015; ADC_COUNT_B2] = [];

    // indicator led
    let mut indicator = Output::new(p.PIN_25, false.into());

    // Run the USB device.
    let usb_fut = usb.run();

    let send_adc_fut = async {
        // class.wait_connection().await;
        let (mut sender, mut reader) = class.split();
        sender.wait_connection().await;
        reader.wait_connection().await;
        info!("Connected");
        // let mut buf = [0; 64];

        sender.write_packet("Connecting ADCs...\r\n".as_bytes()).await.unwrap();

        for ads in expansion_boards_b1.iter_mut() {
            while let Err(e) = ads.begin(&mut i2c).await {
                sender.write_packet("Failed to connect to ADC. Retrying...\r\n".as_bytes()).await.unwrap();
                embassy_time::Timer::after_millis(250).await;
                indicator.set_high();
                embassy_time::Timer::after_millis(250).await;
                indicator.set_low();
            }
            sender
                .write_packet("Connected to ADC\r\n".as_bytes())
                .await
                .unwrap();
        }

        loop {
            let mut values = [0u16; ADC_COUNT_B1*4];
            for (i, ads) in expansion_boards_b1.iter_mut().enumerate() {
                for j in 0..4 {
                    let data = match ads.get_single_ended(&mut i2c, j as u8).await {
                        Ok(data) => data,
                        Err(_) => continue,
                    };
                    values[i*4 + j] = data;
                }
            }
            sender.write_packet(&cast_slice(&values)).await.unwrap();
            // let Ok(data) = ads.get_single_ended(&mut i2c, 0).await else { continue; };

            // info!("ADC data: {:x}", data);

            // let mut str_buf = String::<32>::new();

            // let _ = write!(str_buf, "ADC data: {:x}\r\n", data);
            // sender.write_packet(str_buf.as_bytes()).await.unwrap();
        }
    };

    // Run everything concurrently.
    join(usb_fut, send_adc_fut).await;

    loop {
        embassy_time::Timer::after_millis(500).await;
    }
}
