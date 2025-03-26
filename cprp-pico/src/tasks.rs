use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_rp::i2c::I2c;
use embassy_rp::peripherals::{I2C0, I2C1};

use crate::devices::ads1015::{self};
use crate::messages::{ADC_MESSAGE_CHANNEL, ADCMessage};
use crate::{ADC_COUNT_B0, ADC_COUNT_B1};

pub type I2c0Bus = Mutex<CriticalSectionRawMutex, I2c<'static, I2C0, embassy_rp::i2c::Async>>;
pub type I2c1Bus = Mutex<CriticalSectionRawMutex, I2c<'static, I2C1, embassy_rp::i2c::Async>>;

/// runs asynchronously for each board on I2C bus 0
#[embassy_executor::task(pool_size=ADC_COUNT_B0)]
pub async fn adc_task_b0(mut adc: ads1015::Ads1015, i2c_bus: &'static I2c0Bus, gain: ads1015::AdsGainOptions, id: u8) -> ! {
    let mut device_bus = I2cDevice::new(i2c_bus);

    // initialize sensor
    while let Err(_e) = adc.begin(&mut device_bus).await {
        info!("Failed to connect to ADC on bus 0 (id {}). Retrying...", id);
        embassy_time::Timer::after_millis(500).await;
    }
    adc.set_sample_rate(ads1015::constants::CONFIG_RATE_3300HZ);
    adc.set_gain(gain);

    // send a message to the main task
    ADC_MESSAGE_CHANNEL
        .send(ADCMessage::DoneInitializing(id))
        .await;

    loop {
        for channel in 0..4 {
            let _ = adc.set_single_ended(&mut device_bus, channel as u8).await;
            adc.conversion_delay().await;
            // while let Ok(false) = adc.available(&mut device_bus).await {
            //     // small delay
            //     embassy_time::Timer::after_millis(1).await;
            // }
            if let Ok(val) = adc.get_last_conversion_results(&mut device_bus).await {
                ADC_MESSAGE_CHANNEL
                    .send(ADCMessage::Data(val, channel as u8, id))
                    .await;
            }
        }
    }
}

/// runs asynchronously for each board on I2C bus 1
#[embassy_executor::task(pool_size=ADC_COUNT_B1)]
pub async fn adc_task_b1(mut adc: ads1015::Ads1015, i2c_bus: &'static I2c1Bus, gain: ads1015::AdsGainOptions, id: u8) -> ! {
    let mut device_bus = I2cDevice::new(i2c_bus);

    // initialize sensor
    while let Err(_e) = adc.begin(&mut device_bus).await {
        info!("Failed to connect to ADC on bus 1 (id {}). Retrying...", id);
        embassy_time::Timer::after_millis(500).await;
    }
    adc.set_sample_rate(ads1015::constants::CONFIG_RATE_3300HZ);
    adc.set_gain(gain);

    // send a message to the main task
    ADC_MESSAGE_CHANNEL
        .send(ADCMessage::DoneInitializing(id))
        .await;

    loop {
        for channel in 0..4 {
            let _ = adc.set_single_ended(&mut device_bus, channel as u8).await;
            adc.conversion_delay().await;
            // while let Ok(false) = adc.available(&mut device_bus).await {
            //     embassy_time::Timer::after_millis(1).await;
            // }
            if let Ok(val) = adc.get_last_conversion_results(&mut device_bus).await {
                ADC_MESSAGE_CHANNEL
                    .send(ADCMessage::Data(val, channel as u8, id))
                    .await;
            }
        }
    }
} 