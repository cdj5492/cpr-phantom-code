#![no_std]
#![no_main]

use defmt::info;
use devices::ads1015::{self};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Output, Pull};
use embassy_rp::i2c::I2c;
use embassy_rp::peripherals::{I2C0, I2C1, USB};
use embassy_rp::pwm::{Pwm, SetDutyCycle};
use embassy_rp::usb::Driver as UsbDriver;
use embassy_rp::adc::{Adc, Channel};

use embassy_sync::mutex::Mutex;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::msos::{self, windows_version};
use embassy_usb::{Builder};
use heapless::Vec;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
// use {defmt_rtt as _, panic_reset as _};

#[allow(dead_code)]
mod devices;
mod messages;
mod tasks;

use tasks::*;

/// This is a randomly generated GUID to allow clients on Windows to find our device
const DEVICE_INTERFACE_GUIDS: &[&str] = &["{AFB9A6FB-30BA-44BC-9232-806CFC875321}"];

/// Number of ADC boards on each bus
const ADC_COUNT_B0: usize = 4;
const ADC_COUNT_B1: usize = 4;

const BOARD_COUNT: usize = ADC_COUNT_B0 + ADC_COUNT_B1;
const CHANNEL_COUNT: usize = BOARD_COUNT * 4;

const MAGIC: u16 = 0xAA55;

/// Size of the packet in bytes
const PACKET_SIZE: usize = 2 + 1 + 4 + CHANNEL_COUNT * 2 as usize;

/// Ideally how many packets should we be sending per second?
const TARGET_PACKET_RATE: u64 = 1000;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<I2C0>;
    I2C1_IRQ => embassy_rp::i2c::InterruptHandler<I2C1>;
    ADC_IRQ_FIFO => embassy_rp::adc::InterruptHandler;
});

bind_interrupts!(
    struct I2CIrqs {}
);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut built_in_adc = Adc::new(p.ADC, Irqs, embassy_rp::adc::Config::default());
    let mut p26 = Channel::new_pin(p.PIN_26, Pull::None);
    let mut p27 = Channel::new_pin(p.PIN_27, Pull::None);
    let mut p28 = Channel::new_pin(p.PIN_28, Pull::None);

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
    let mut config = embassy_usb::Config::new(0xf569, 0x0001);
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

    // PWM pin for power to ADC board MOSFET
    let mut adc_ppwm = Output::new(p.PIN_2, true.into());
    // let desired_freq_hz = 250_000;
    // let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
    // info!("PWM source clock freq: {}", clock_freq_hz);
    // let divider = 16u8;
    // let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

    // let mut c = embassy_rp::pwm::Config::default();
    // c.top = period;
    // c.divider = divider.into();

    // let mut adc_ppwm = Pwm::new_output_a(p.PWM_SLICE1, p.PIN_2, c.clone());
    // adc_ppwm.set_duty_cycle_percent(100).unwrap();

    // general purpose pin for ADC board (not used yet)
    let mut _adc_gp = Output::new(p.PIN_3, false.into());

    let mut config = embassy_rp::i2c::Config::default();
    config.frequency = 1_000_000; // 1 MHz

    let i2c0 = I2c::new_async(p.I2C0, p.PIN_21, p.PIN_20, Irqs, config);
    static I2C0_BUS: StaticCell<I2c0Bus> = StaticCell::new();
    let i2c0_bus = I2C0_BUS.init(Mutex::new(i2c0));
    let i2c1 = I2c::new_async(p.I2C1, p.PIN_19, p.PIN_18, Irqs, config);
    static I2C1_BUS: StaticCell<I2c1Bus> = StaticCell::new();
    let i2c1_bus = I2C1_BUS.init(Mutex::new(i2c1));

    let adc_task_spawn_results = [
        spawner.spawn(adc_task_b0(
            ads1015::Ads1015::new(ads1015::AdsAddressOptions::Addr48),
            i2c0_bus,
            ads1015::AdsGainOptions::Two,
            0,
        )),
        spawner.spawn(adc_task_b0(
            ads1015::Ads1015::new(ads1015::AdsAddressOptions::Addr4B),
            i2c0_bus,
            ads1015::AdsGainOptions::Two,
            1,
        )),
        spawner.spawn(adc_task_b0(
            ads1015::Ads1015::new(ads1015::AdsAddressOptions::Addr4A),
            i2c0_bus,
            ads1015::AdsGainOptions::Two,
            2,
        )),
        // force board 1
        spawner.spawn(adc_task_b0(
            ads1015::Ads1015::new(ads1015::AdsAddressOptions::Addr49),
            i2c0_bus,
            ads1015::AdsGainOptions::One,
            3,
        )),
        // force board 2
        spawner.spawn(adc_task_b1(
            ads1015::Ads1015::new(ads1015::AdsAddressOptions::Addr48),
            i2c1_bus,
            ads1015::AdsGainOptions::One,
            4,
        )),
        spawner.spawn(adc_task_b1(
            ads1015::Ads1015::new(ads1015::AdsAddressOptions::Addr49),
            i2c1_bus,
            ads1015::AdsGainOptions::Two,
            5,
        )),
        spawner.spawn(adc_task_b1(
            ads1015::Ads1015::new(ads1015::AdsAddressOptions::Addr4A),
            i2c1_bus,
            ads1015::AdsGainOptions::Two,
            6,
        )),
        spawner.spawn(adc_task_b1(
            ads1015::Ads1015::new(ads1015::AdsAddressOptions::Addr4B),
            i2c1_bus,
            ads1015::AdsGainOptions::Two,
            7,
        )),
    ];
    if !adc_task_spawn_results.iter().all(|x| x.is_ok()) {
        panic!("Failed to spawn some ADC tasks. Check ADC_COUNT_B0 and ADC_COUNT_B1");
    }

    // Run the USB device.
    let usb_fut = usb.run();

    // packet to send
    let mut packet = Vec::<u8, PACKET_SIZE>::new();

    // Current record of most recent reports for boards
    let mut sensor_data = [i16::MIN; CHANNEL_COUNT];

    let send_adc_fut = async {
        // class.wait_connection().await;
        let (mut sender, mut reader) = class.split();
        info!("Waiting for USB connection...");
        sender.wait_connection().await;
        reader.wait_connection().await;
        info!("Connected via USB.");

        info!("Connecting ADCs...");

        // build packet header
        // MAGIC (2 bytes), data length (1 bytes),
        packet.extend(MAGIC.to_le_bytes());
        let _ = packet.push((ADC_COUNT_B0 * 8 + ADC_COUNT_B1 * 8 + 4 + 3*2) as u8);

        indicator.set_high();

        // wait for a DoneIntializing packet from each board
        let mut boards_initialized = 0;
        while boards_initialized < BOARD_COUNT {
            match messages::ADC_MESSAGE_CHANNEL.receive().await {
                messages::ADCMessage::DoneInitializing(id) => {
                    boards_initialized += 1;
                    info!("Connected new ADC {}", id);
                },
                messages::ADCMessage::Data(val, channel, id) => {
                    sensor_data[id as usize * 4 + channel as usize] = val;
                }
            }

            let now = embassy_time::Instant::now();
            let start = now.as_micros();
            let timestamp = now.as_millis() as u32;

            // keep header
            packet.truncate(3);

            // timestamp (4 bytes)
            packet.extend(timestamp.to_le_bytes());

            // insert on-board adcs
            packet.extend(built_in_adc.read(&mut p26).await.unwrap_or(0).to_le_bytes());
            packet.extend(built_in_adc.read(&mut p27).await.unwrap_or(0).to_le_bytes());
            packet.extend(built_in_adc.read(&mut p28).await.unwrap_or(0).to_le_bytes());

            // put data in the packet
            for i in 0..sensor_data.len() {
                packet.extend(sensor_data[i].to_le_bytes());
            }

            // send packet to host
            // only send 64 bytes at a time
            for i in (0..packet.len()).step_by(64) {
                let end = (i + 64).min(packet.len());
                sender.write_packet(&packet[i..end]).await.unwrap();
            }

            // regulate loop speed
            let now = embassy_time::Instant::now();
            let elapsed = now.as_micros() - start;
            embassy_time::Timer::after_micros((1_000_000 / TARGET_PACKET_RATE).saturating_sub(elapsed)).await;
        }

        indicator.set_low();

        info!("All ADCs connected.");

        loop {
            let now = embassy_time::Instant::now();
            let start = now.as_micros();
            let timestamp = now.as_millis() as u32;

            // keep header
            packet.truncate(3);

            // timestamp (4 bytes)
            packet.extend(timestamp.to_le_bytes());

            // This only works if this task runs much faster than the ADCs can produce data!
            while let Ok(message) = messages::ADC_MESSAGE_CHANNEL.try_receive() {
                match message {
                    messages::ADCMessage::Data(val, channel, id) => {
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
            // only send 64 bytes at a time
            for i in (0..packet.len()).step_by(64) {
                let end = (i + 64).min(packet.len());
                sender.write_packet(&packet[i..end]).await.unwrap();
            }

            // regulate loop speed
            let now = embassy_time::Instant::now();
            let elapsed = now.as_micros() - start;
            embassy_time::Timer::after_micros((1_000_000 / TARGET_PACKET_RATE).saturating_sub(elapsed)).await;
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

