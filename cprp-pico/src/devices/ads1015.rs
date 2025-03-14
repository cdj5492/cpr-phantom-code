//! Based on https://github.com/sparkfun/qwiic_ads1015_py/blob/master/qwiic_ads1015.py

use embassy_time::Timer;
use embedded_hal_async::i2c::I2c;

/// An error type for QwiicADS1015 operations.
#[derive(Debug)]
pub enum QwiicAds1015Error<I2cE> {
    /// Underlying I²C error.
    I2c(I2cE),
    /// Device not connected or did not respond.
    NotConnected,
    /// An invalid channel was specified.
    InvalidChannel,
    /// Other error condition.
    Other,
}

#[allow(dead_code)]
mod constants {
    // Device name and I2C addresses:
    pub const DEVICE_NAME: &str = "Qwiic ADS1015";
    pub const AVAILABLE_I2C_ADDRESSES: [u8; 4] = [0x48, 0x49, 0x4A, 0x4B];

    /// A delay in milliseconds used after certain operations.
    pub const ADS1015_DELAY_MS: u64 = 1;

    // Pointer Registers
    pub const POINTER_CONVERT: u8   = 0x00;
    pub const POINTER_CONFIG: u8    = 0x01;
    pub const POINTER_LOW_THRESH: u8 = 0x02;
    pub const POINTER_HI_THRESH: u8 = 0x03;

    // Config Register - common bits (all 16-bit values)
    // Operational status (OS) / single-shot conversion start flag.
    pub const CONFIG_OS_NO: u16      = 0x0000;
    pub const CONFIG_OS_SINGLE: u16  = 0x8000;  // write: start a single conversion, read: result ready when 0x8000

    // Mode selection.
    pub const CONFIG_MODE_CONT: u16   = 0x0000;
    pub const CONFIG_MODE_SINGLE: u16 = 0x0100;

    // Multiplexer configuration (differential/single-ended)
    pub const CONFIG_MUX_SINGLE0: u16 = 0x4000;
    pub const CONFIG_MUX_SINGLE1: u16 = 0x5000;
    pub const CONFIG_MUX_SINGLE2: u16 = 0x6000;
    pub const CONFIG_MUX_SINGLE3: u16 = 0x7000;
    pub const CONFIG_MUX_DIFF_P0N1: u16 = 0x0000;
    pub const CONFIG_MUX_DIFF_P0N3: u16 = 0x1000;
    pub const CONFIG_MUX_DIFF_P1N3: u16 = 0x2000;
    pub const CONFIG_MUX_DIFF_P2N3: u16 = 0x3000;

    // Data rate (sample rate) mask and options.
    pub const CONFIG_RATE_MASK: u16 = 0x00E0;
    pub const CONFIG_RATE_128HZ: u16  = 0x0000;
    pub const CONFIG_RATE_250HZ: u16  = 0x0020;
    pub const CONFIG_RATE_490HZ: u16  = 0x0040;
    pub const CONFIG_RATE_920HZ: u16  = 0x0060;
    pub const CONFIG_RATE_1600HZ: u16 = 0x0080;
    pub const CONFIG_RATE_2400HZ: u16 = 0x00A0;
    pub const CONFIG_RATE_3300HZ: u16 = 0x00C0;

    // Programmable Gain Amplifier (PGA) mask and options.
    pub const CONFIG_PGA_MASK: u16         = 0x0E00;
    pub const CONFIG_PGA_TWO_THIRDS: u16     = 0x0000; // ±6.144 V
    pub const CONFIG_PGA_1: u16            = 0x0200; // ±4.096 V
    pub const CONFIG_PGA_2: u16            = 0x0400; // ±2.048 V (default)
    pub const CONFIG_PGA_4: u16            = 0x0600; // ±1.024 V
    pub const CONFIG_PGA_8: u16            = 0x0800; // ±0.512 V
    pub const CONFIG_PGA_16: u16           = 0x0A00; // ±0.256 V

    // Comparator configuration.
    pub const CONFIG_CMODE_TRAD: u16 = 0x0000;  // Traditional comparator
    pub const CONFIG_CMODE_WINDOW: u16 = 0x0010; // Window comparator
    pub const CONFIG_CPOL_ACTV_LOW: u16 = 0x0000; // ALERT/RDY active low
    pub const CONFIG_CPOL_ACTV_HI: u16 = 0x0008;  // ALERT/RDY active high
    pub const CONFIG_CLAT_NONLAT: u16 = 0x0000;
    pub const CONFIG_CLAT_LATCH: u16  = 0x0004;
    pub const CONFIG_CQUE_1CONV: u16  = 0x0000;
    pub const CONFIG_CQUE_2CONV: u16  = 0x0001;
    pub const CONFIG_CQUE_4CONV: u16  = 0x0002;
    pub const CONFIG_CQUE_NONE: u16   = 0x0003;
}

/// QwiicADS1015 encapsulates the ADS1015 device.
pub struct QwiicAds1015<T> {
    addr: u8,
    /// Underlying I2C bus that implements the async I²C traits.
    i2c: T,
    sample_rate: u16,
    gain: u16,
    use_conversion_ready: bool,
    mode: u16,
    multiplier_to_volts: f32,
    /// Calibration values for channels 0..3.
    /// Each entry holds a two-element array: [low, high].
    calibration_values: [[u16; 2]; 4],
}

impl<T, I2cE> QwiicAds1015<T>
where
    // T: I2c<Error = I2cE> + ?Sized,
    T: I2c<Error = I2cE>,
{
    /// Create a new QwiicAds1015 from an I2C bus.
    /// If an address is provided and is in the list of available addresses,
    /// it is used; otherwise the default address is selected.
    pub fn new(i2c: T, address: Option<u8>) -> Self {
        use constants::AVAILABLE_I2C_ADDRESSES;
        let addr = match address {
            Some(a) if AVAILABLE_I2C_ADDRESSES.contains(&a) => a,
            _ => AVAILABLE_I2C_ADDRESSES[0],
        };
        Self {
            addr,
            i2c,
            // Default sample rate is 1600 Hz and gain ±2.048 V.
            sample_rate: constants::CONFIG_RATE_1600HZ,
            gain: constants::CONFIG_PGA_2,
            use_conversion_ready: false,
            mode: constants::CONFIG_MODE_CONT,
            multiplier_to_volts: 1.0,
            calibration_values: [[0; 2], [0; 2], [0; 2], [0; 2]],
        }
    }

    /// Checks if the device is connected.
    ///
    /// In this implementation we simply try to read the conversion register;
    /// if the I2C transaction fails we assume the device is not connected.
    pub async fn is_connected(&mut self) -> Result<bool, QwiicAds1015Error<I2cE>> {
        // Try reading 1 byte from the conversion register.
        // (Some I²C implementations use an ACK even if the data isn't used.)
        match self.read_block(constants::POINTER_CONVERT).await {
            Ok(_data) => Ok(true),
            Err(e) => Err(e),
        }
    }

    /// Initializes the device.
    /// Returns Ok(()) if the device appears connected.
    pub async fn begin(&mut self) -> Result<(), QwiicAds1015Error<I2cE>> {
        if self.is_connected().await? {
            Ok(())
        } else {
            Err(QwiicAds1015Error::NotConnected)
        }
    }

    /// Reads a single-ended ADC channel (0-3).
    ///
    /// Returns the raw 12-bit ADC value (right-justified).
    pub async fn get_single_ended(&mut self, channel: u8) -> Result<u16, QwiicAds1015Error<I2cE>> {
        if channel > 3 {
            return Err(QwiicAds1015Error::InvalidChannel);
        }
        use constants::*;
        // Build the configuration word.
        let mut config: u16 = CONFIG_OS_SINGLE | CONFIG_CQUE_NONE | self.sample_rate | self.gain;
        // In conversion-ready mode we always force single-shot,
        // otherwise we use the stored mode.
        config |= if self.use_conversion_ready {
            CONFIG_MODE_SINGLE
        } else {
            self.mode
        };
        // Set the MUX for the given single-ended channel.
        config |= match channel {
            0 => CONFIG_MUX_SINGLE0,
            1 => CONFIG_MUX_SINGLE1,
            2 => CONFIG_MUX_SINGLE2,
            3 => CONFIG_MUX_SINGLE3,
            _ => unreachable!(),
        };
        // Write configuration to register POINTER_CONFIG
        self.write_block(
            constants::POINTER_CONFIG,
            &[(config >> 8) as u8, (config & 0xFF) as u8],
        )
        .await?;

        // If using conversion-ready flag, poll until available.
        if self.use_conversion_ready {
            while !self.available().await? {
                // add a small delay (1 ms)
                Timer::after(embassy_time::Duration::from_millis(1)).await;
            }
        } else {
            self.conversion_delay().await;
        }

        // Read the conversion register (2 bytes). The ADS1015 returns 12-bit data left-justified,
        // so we shift right 4 bits.
        let data = self.read_block(POINTER_CONVERT).await?;
        let raw = u16::from(data[0]) << 8 | u16::from(data[1]);
        Ok(raw >> 4)
    }

    /// Returns the signed value for a single-ended reading.
    pub async fn get_single_ended_signed(&mut self, channel: u8) -> Result<i16, QwiicAds1015Error<I2cE>> {
        let val = self.get_single_ended(channel).await?;
        // If the result is greater than 0x07FF then the sign bit (bit 11) is set.
        let signed = if val > 0x07FF {
            (val as i16) - (1 << 12)
        } else {
            val as i16
        };
        Ok(signed)
    }

    /// Returns the ADC value in millivolts.
    pub async fn get_single_ended_millivolts(&mut self, channel: u8) -> Result<f32, QwiicAds1015Error<I2cE>> {
        let reading = self.get_single_ended(channel).await?;
        Ok((reading as f32) * self.get_multiplier())
    }

    /// Reads a differential channel.
    ///
    /// The parameter "mux" must be one of:
    ///   CONFIG_MUX_DIFF_P0N1, CONFIG_MUX_DIFF_P0N3, CONFIG_MUX_DIFF_P1N3, CONFIG_MUX_DIFF_P2N3.
    /// Returns the signed 12-bit differential value.
    pub async fn get_differential(&mut self, mux: u16) -> Result<i16, QwiicAds1015Error<I2cE>> {
        use constants::*;
        // Validate the mux value.
        if ![CONFIG_MUX_DIFF_P0N1, CONFIG_MUX_DIFF_P0N3, CONFIG_MUX_DIFF_P1N3, CONFIG_MUX_DIFF_P2N3]
            .contains(&mux)
        {
            return Err(QwiicAds1015Error::Other);
        }
        let mut config: u16 = CONFIG_OS_SINGLE | CONFIG_CQUE_NONE | self.sample_rate | self.gain;
        config |= if self.use_conversion_ready { CONFIG_MODE_SINGLE } else { self.mode };
        // Add the requested differential mux.
        config |= mux;
        self.write_block(
            POINTER_CONFIG,
            &[(config >> 8) as u8, (config & 0xFF) as u8],
        )
        .await?;
        if self.use_conversion_ready {
            while !self.available().await? {
                Timer::after(embassy_time::Duration::from_millis(1)).await;
            }
        } else {
            self.conversion_delay().await;
        }
        let data = self.read_block(POINTER_CONVERT).await?;
        let raw = ((u16::from(data[0]) << 8) | u16::from(data[1])) >> 4;
        // Adjust for negative value (12-bit two's complement)
        let result = if raw > 0x07FF {
            (raw as i16) - (1 << 12)
        } else {
            raw as i16
        };
        Ok(result)
    }

    /// Returns the differential measurement in millivolts.
    pub async fn get_differential_millivolts(&mut self, mux: u16) -> Result<f32, QwiicAds1015Error<I2cE>> {
        let val = self.get_differential(mux).await?;
        Ok((val as f32) * self.get_multiplier())
    }

    /// Maps a value from one range to another.
    pub fn mapf(val: f32, in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> f32 {
        (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    }

    /// Returns a scaled value (0 to 1) given the calibration for the channel.
    ///
    /// If the sensor is not calibrated (calibration values are zero), returns 0.
    pub async fn get_scaled_analog_data(&mut self, channel: usize) -> Result<f32, QwiicAds1015Error<I2cE>> {
        if channel > 3 {
            return Ok(0.0);
        }
        let raw = self.get_single_ended(channel as u8).await? as f32;
        let lo = self.calibration_values[channel][0] as f32;
        let hi = self.calibration_values[channel][1] as f32;
        let mapped = Self::mapf(raw, lo, hi, 0.0, 1.0);
        if mapped > 1.0 {
            Ok(1.0)
        } else if mapped < 0.0 {
            Ok(0.0)
        } else {
            Ok(mapped)
        }
    }

    /// Calibrates the sensor on channels 0 and 1.
    ///
    /// Updates the calibration values if the new reading is lower/higher than stored thresholds.
    pub async fn calibrate(&mut self) -> Result<(), QwiicAds1015Error<I2cE>> {
        // Only calibrate for channels 0 and 1 as in the original Python code.
        for channel in 0..2 {
            let value = self.get_single_ended(channel as u8).await?;
            // Only update if value is less than current minimum or greater than current maximum.
            if (value < self.calibration_values[channel][0] || self.calibration_values[channel][0] == 0)
                && value < 1085
            {
                self.calibration_values[channel][0] = value;
            } else if value > self.calibration_values[channel][1] || self.calibration_values[channel][1] == 0 {
                self.calibration_values[channel][1] = value;
            }
        }
        Ok(())
    }

    /// Gets the stored calibration value for a channel.
    /// hi_lo should be 0 for low or 1 for high.
    pub fn get_calibration(&self, channel: usize, hi_lo: usize) -> u16 {
        self.calibration_values[channel][hi_lo]
    }

    /// Sets the calibration value for a channel.
    pub fn set_calibration(&mut self, channel: usize, hi_lo: usize, value: u16) {
        self.calibration_values[channel][hi_lo] = value;
    }

    /// Resets all calibration values to zero.
    pub fn reset_calibration(&mut self) {
        self.calibration_values = [[0; 2], [0; 2], [0; 2], [0; 2]];
    }

    /// Sets the conversion mode.
    ///
    /// Pass either CONFIG_MODE_CONT or CONFIG_MODE_SINGLE.
    pub fn set_mode(&mut self, mode: u16) {
        self.mode = mode & constants::CONFIG_MODE_SINGLE;
    }

    /// Returns the current conversion mode.
    pub fn get_mode(&self) -> u16 {
        self.mode
    }

    /// Sets the gain value.
    pub fn set_gain(&mut self, gain: u16) {
        self.gain = gain & constants::CONFIG_PGA_MASK;
        self.update_multiplier_to_volts();
    }

    /// Gets the current gain value.
    pub fn get_gain(&self) -> u16 {
        self.gain
    }

    /// Updates the multiplier to convert ADC counts to volts.
    fn update_multiplier_to_volts(&mut self) {
        use constants::*;
        self.multiplier_to_volts = match self.gain {
            CONFIG_PGA_TWO_THIRDS => 3.0,
            CONFIG_PGA_1 => 2.0,
            CONFIG_PGA_2 => 1.0,
            CONFIG_PGA_4 => 0.5,
            CONFIG_PGA_8 => 0.25,
            CONFIG_PGA_16 => 0.125,
            _ => 1.0,
        };
    }

    /// Returns the multiplier used to convert a raw reading to volts.
    pub fn get_multiplier(&self) -> f32 {
        self.multiplier_to_volts
    }

    /// Sets the sample (data) rate.
    pub fn set_sample_rate(&mut self, rate: u16) {
        self.sample_rate = rate & constants::CONFIG_RATE_MASK;
    }

    /// Gets the sample (data) rate.
    pub fn get_sample_rate(&self) -> u16 {
        self.sample_rate
    }

    /// Checks if a conversion is ready.
    ///
    /// Reads the config register and checks if the OS bit is set.
    pub async fn available(&mut self) -> Result<bool, QwiicAds1015Error<I2cE>> {
        let data = self.read_block(constants::POINTER_CONFIG).await?;
        let value = u16::from(data[0]) << 8 | u16::from(data[1]);
        // The OS ready state is represented by the CONFIG_OS_SINGLE bit.
        Ok((value & constants::CONFIG_OS_SINGLE) != 0)
    }

    /// Sets the comparator for a single-ended channel with a threshold.
    ///
    /// The comparator will assert when the ADC value exceeds the given threshold.
    pub async fn set_comparator_single_ended(
        &mut self,
        channel: u8,
        threshold: u16,
    ) -> Result<(), QwiicAds1015Error<I2cE>> {
        if channel > 3 {
            return Err(QwiicAds1015Error::InvalidChannel);
        }
        use constants::*;
        // Build configuration for comparator: continuous mode, 1 conversion queue,
        // latching, active low comparator, traditional mode.
        let mut config: u16 = CONFIG_MODE_CONT | self.sample_rate | CONFIG_CQUE_1CONV
            | CONFIG_CLAT_LATCH | CONFIG_CPOL_ACTV_LOW | CONFIG_CMODE_TRAD;
        config |= self.gain;

        // Set the MUX for the given channel.
        config |= match channel {
            0 => CONFIG_MUX_SINGLE0,
            1 => CONFIG_MUX_SINGLE1,
            2 => CONFIG_MUX_SINGLE2,
            3 => CONFIG_MUX_SINGLE3,
            _ => unreachable!(),
        };

        // Write the high threshold value.
        // Shift left 4 bits because ADS1015 uses 12-bit data left aligned in 16-bit.
        let thresh = threshold << 4;
        self.write_block(
            constants::POINTER_HI_THRESH,
            &[(thresh >> 8) as u8, (thresh & 0xFF) as u8],
        )
        .await?;
        // Now write the configuration register.
        self.write_block(
            constants::POINTER_CONFIG,
            &[(config >> 8) as u8, (config & 0xFF) as u8],
        )
        .await
    }

    /// Reads the last conversion result (without modifying the configuration).
    ///
    /// Returns the signed 12-bit conversion result.
    pub async fn get_last_conversion_results(&mut self) -> Result<i16, QwiicAds1015Error<I2cE>> {
        let data = self.read_block(constants::POINTER_CONVERT).await?;
        let mut result = ((u16::from(data[0]) << 8) | u16::from(data[1])) >> 4;
        if result > 0x07FF {
            result -= 1 << 12;
        }
        Ok(result as i16)
    }

    /// Delays for an interval based on the current sample rate.
    pub async fn conversion_delay(&mut self) {
        use constants::*;
        let delay_micros = if self.sample_rate >= CONFIG_RATE_3300HZ {
            400
        } else if self.sample_rate >= CONFIG_RATE_2400HZ {
            500
        } else if self.sample_rate >= CONFIG_RATE_1600HZ {
            1000
        } else if self.sample_rate >= CONFIG_RATE_920HZ {
            2000
        } else if self.sample_rate >= CONFIG_RATE_490HZ {
            4000
        } else if self.sample_rate >= CONFIG_RATE_250HZ {
            8000
        } else {
            16000
        };
        Timer::after(embassy_time::Duration::from_micros(delay_micros)).await;
    }

    /// Enables or disables the use of conversion-ready polling.
    pub fn use_conversion_ready(&mut self, enable: bool) {
        self.use_conversion_ready = enable;
    }

    // I²C helper functions
    // These helper functions wrap typical I2C write/read operations.
    // They assume that the I²C implementation supports writing a register
    // byte followed by data, and reading similarly.
    async fn write_block(&mut self, register: u8, data: &[u8]) -> Result<(), QwiicAds1015Error<I2cE>> {
        // A common pattern is to send the register followed by the data.
        // Here we build a buffer vec that starts with the register.
        let mut buf = [0u8; 16]; // large enough for our uses.
        buf[0] = register;
        buf[1..(1+data.len())].copy_from_slice(data);
        self.i2c
            .write(self.addr, &buf[0..(1+data.len())])
            .await
            .map_err(QwiicAds1015Error::I2c)
    }

    async fn read_block(&mut self, register: u8) -> Result<[u8; 2], QwiicAds1015Error<I2cE>> {
        // Write the register address then read the requested number of bytes.
        self.i2c
            .write(self.addr, &[register])
            .await
            .map_err(QwiicAds1015Error::I2c)?;
        let mut buf = [0u8; 2];
        self.i2c
            .read(self.addr, &mut buf)
            .await
            .map_err(QwiicAds1015Error::I2c)?;
        Ok(buf)
    }
}