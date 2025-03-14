#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::i2c::I2c;
use embassy_rp::peripherals::{I2C0, I2C1, USB};
use embassy_rp::usb::{Driver as UsbDriver};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State, USB_CLASS_CDC};
use embassy_usb::driver::{Driver, Endpoint, EndpointIn, EndpointOut};
use embassy_usb::msos::{self, windows_version};
use embassy_usb::{Builder, Config};
use {defmt_rtt as _, panic_probe as _};

mod devices;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<I2C0>;
    I2C1_IRQ => embassy_rp::i2c::InterruptHandler<I2C1>;
});

bind_interrupts!(struct I2CIrqs {
});

// This is a randomly generated GUID to allow clients on Windows to find our device
const DEVICE_INTERFACE_GUIDS: &[&str] = &["{AFB9A6FB-30BA-44BC-9232-806CFC875321}"];

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

    let i2c = I2c::new_async(
        p.I2C0,
        p.PIN_21,
        p.PIN_20,
        Irqs,
        config
    );

    let mut ads = devices::ads1015::QwiicAds1015::new(i2c, None);



    // Run the USB device.
    let usb_fut = usb.run();

    let echo_fut = async {
        // class.wait_connection().await;
        let (mut sender, mut reader) = class.split();
        sender.wait_connection().await;
        reader.wait_connection().await;
        info!("Connected");
        let mut buf = [0; 64];

        let status_buf = "Beginning ads\r\n".as_bytes();
        sender.write_packet(status_buf).await.unwrap();
        while let Err(e) = ads.begin().await {
            sender.write_packet(status_buf).await.unwrap();
            embassy_time::Timer::after_millis(500).await;
        }

        loop {
            let n = reader.read_packet(&mut buf).await.unwrap();
            let data = &buf[..n];
            info!("echo data: {:x}, len: {}", data, n);
            sender.write_packet(data).await.unwrap();
            // Clear bufffer
            buf = [0; 64];
        }
    };


    // Run everything concurrently.
    join(usb_fut, echo_fut).await;

    loop {
        embassy_time::Timer::after_millis(500).await;
    }
}
