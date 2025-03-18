#![no_std]
#![no_main]

use core::fmt::Write;
use core::future::Future;

use defmt::info;
use devices::ads1015::{self, Ads1015, AdsAddressOptions};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::Output;
use embassy_rp::i2c::I2c;
use embassy_rp::peripherals::{I2C0, I2C1, USB};
use embassy_rp::usb::Driver as UsbDriver;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;

use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
// use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::msos::{self, windows_version};
use embassy_usb::{Builder, Config};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use heapless::pool::arc::Arc;
// use embassy_sync::signal::Signal;
use heapless::{String, Vec};
use static_cell::StaticCell;
// use {defmt_rtt as _, panic_probe as _};
use {defmt_rtt as _, panic_reset as _};

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
const ADC_COUNT_B0: usize = 2;
const ADC_COUNT_B1: usize = 1;

const BOARD_COUNT: usize = ADC_COUNT_B0 + ADC_COUNT_B1;
const CHANNEL_COUNT: usize = BOARD_COUNT * 4;

const MAGIC: u16 = 0xAA55;

const PACKET_SIZE: usize = 2 + 1 + 4 + CHANNEL_COUNT*2 as usize;

// Ideally how many packets should we be sending per second?
const TARGET_PACKET_RATE : u32 = 1000;

type I2c0Bus = Mutex<CriticalSectionRawMutex, I2c<'static, I2C0, embassy_rp::i2c::Async>>;
type I2c1Bus = Mutex<CriticalSectionRawMutex, I2c<'static, I2C1, embassy_rp::i2c::Async>>;

static ADC_MESSAGE_CHANNEL: Channel<CriticalSectionRawMutex, ADCMessage, BOARD_COUNT> = Channel::new();

// message from ADC task to main task
enum ADCMessage {
    // (board id)
    DoneInitializing(u8),
    // (reading, channel, board id)
    Data(u16, u8, u8),
}

/// runs asynchronously for each board on I2C bus 0
#[embassy_executor::task(pool_size=ADC_COUNT_B0)]
async fn adc_task_b0(mut adc: ads1015::Ads1015, i2c_bus: &'static I2c0Bus, id: u8) -> ! {
    let mut device_bus = I2cDevice::new(i2c_bus);
    
    // initialize sensor
    while let Err(_e) = adc.begin(&mut device_bus).await {
        info!("Failed to connect to ADC on bus 0 (id {}). Retrying...", id);
        embassy_time::Timer::after_millis(500).await;
    }
    adc.set_sample_rate(ads1015::constants::CONFIG_RATE_3300HZ);

    // send a message to the main task
    ADC_MESSAGE_CHANNEL.send(ADCMessage::DoneInitializing(id)).await;

    loop {
        for channel in 0..4 {
            let _ = adc.set_single_ended(&mut device_bus, channel as u8).await;
            adc.conversion_delay().await;
            if let Ok(val) = adc.get_last_conversion_results(&mut device_bus).await {
                ADC_MESSAGE_CHANNEL.send(ADCMessage::Data(val, channel as u8, id)).await;
            }
        }
    }
}

/// runs asynchronously for each board on I2C bus 1
#[embassy_executor::task(pool_size=ADC_COUNT_B1)]
async fn adc_task_b1(mut adc: ads1015::Ads1015, i2c_bus: &'static I2c1Bus, id: u8) -> ! {
    let mut device_bus = I2cDevice::new(i2c_bus);
    
    // initialize sensor
    while let Err(_e) = adc.begin(&mut device_bus).await {
        info!("Failed to connect to ADC on bus 1 (id {}). Retrying...", id);
        embassy_time::Timer::after_millis(500).await;
    }
    adc.set_sample_rate(ads1015::constants::CONFIG_RATE_3300HZ);

    // send a message to the main task
    ADC_MESSAGE_CHANNEL.send(ADCMessage::DoneInitializing(id)).await;

    loop {
        for channel in 0..4 {
            let _ = adc.set_single_ended(&mut device_bus, channel as u8).await;
            adc.conversion_delay().await;
            if let Ok(val) = adc.get_last_conversion_results(&mut device_bus).await {
                ADC_MESSAGE_CHANNEL.send(ADCMessage::Data(val, channel as u8, id)).await;
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut indicator = Output::new(p.PIN_25, false.into());
    // blink at the beginning rapidly 3 times to show it's the beginning
    for _ in 0..3 {
        indicator.set_high();
        embassy_time::Timer::after_millis(70).await;
        indicator.set_low();
        embassy_time::Timer::after_millis(70).await;
    }

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

    let i2c0 = I2c::new_async(p.I2C0, p.PIN_21, p.PIN_20, Irqs, config);
    static I2C0_BUS: StaticCell<I2c0Bus> = StaticCell::new();
    let i2c0_bus = I2C0_BUS.init(Mutex::new(i2c0));
    let i2c1 = I2c::new_async(p.I2C1, p.PIN_19, p.PIN_18, Irqs, config);
    static I2C1_BUS: StaticCell<I2c1Bus> = StaticCell::new();
    let i2c1_bus = I2C1_BUS.init(Mutex::new(i2c1));

    let adc_task_spawn_results = [
        spawner.spawn(adc_task_b1(ads1015::Ads1015::new(ads1015::AdsAddressOptions::Addr48), i2c1_bus, 0)),
        spawner.spawn(adc_task_b0(ads1015::Ads1015::new(ads1015::AdsAddressOptions::Addr49), i2c0_bus, 1)),
        spawner.spawn(adc_task_b0(ads1015::Ads1015::new(ads1015::AdsAddressOptions::Addr4A), i2c0_bus, 2)),
    ];
    if !adc_task_spawn_results.iter().all(|x| x.is_ok()) {
        panic!("Failed to spawn some ADC tasks. Check ADC_COUNT_B0 and ADC_COUNT_B1");
    }

    // Run the USB device.
    let usb_fut = usb.run();

    // packet to send
    let mut packet = Vec::<u8, PACKET_SIZE>::new();

    // Current record of most recent reports for boards
    let mut sensor_data = [0u16; CHANNEL_COUNT];

    let send_adc_fut = async {
        // class.wait_connection().await;
        let (mut sender, mut reader) = class.split();
        info!("Waiting for USB connection...");
        sender.wait_connection().await;
        reader.wait_connection().await;
        info!("Connected via USB.");

        info!("Connecting ADCs...");

        indicator.set_high();

        // wait for a DoneIntializing packet from each board
        let mut boards_initialized = 0;
        while boards_initialized < BOARD_COUNT {
            match ADC_MESSAGE_CHANNEL.receive().await {
                ADCMessage::DoneInitializing(id) => {
                    boards_initialized += 1;
                    info!("Connected new ADC {}", id);
                },
                _ => (),
            }
        }

        indicator.set_low();

        info!("All ADCs connected.");

        // build packet header
        // MAGIC (2 bytes), data length (1 bytes), 
        packet.extend(MAGIC.to_le_bytes());
        let _ = packet.push((ADC_COUNT_B0 * 8 + ADC_COUNT_B1 * 8 + 4) as u8);

        loop {
            // keep header
            packet.truncate(3);

            let now = embassy_time::Instant::now();
            let timestamp = now.as_millis() as u32;

            // timestamp (4 bytes)
            packet.extend(timestamp.to_le_bytes());


            // This only works if this task runs much faster than the ADCs can produce data!
            while let Ok(message) = ADC_MESSAGE_CHANNEL.try_receive() {
                match message {
                    ADCMessage::Data(val, channel, id) => {
                        sensor_data[id as usize * 4 + channel as usize] = val;
                    }
                    _ => (),
                }
            }

            // put data in the packet
            for i in 0..sensor_data.len() {
                packet.extend(sensor_data[i].to_le_bytes());
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
