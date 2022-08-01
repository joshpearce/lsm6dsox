// Copyright 2021 Open Logistics Foundation
//
// Licensed under the Open Logistics License 1.0.
// For details on the licensing terms, see the LICENSE file.

#![no_std]

//! Platform-agnostic embedded-hal driver for the STMicroelectronics LSM6DSOX iNEMO inertial module.
//!
//! Provided functionality is inspired by the C implementation from ST,
//! but tries to provide a higher level interface where possible.
//!
//! To provide measurements the [accelerometer] traits and [measurements] crate are utilized.
//!
//!
//!
//! ## Resources
//!
//! [Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsox.pdf)
//!
//! [LSM6DSOX at st.com](https://www.st.com/en/mems-and-sensors/lsm6dsox.html)
//!
//!
//! For application hints please also refer to the
//! [application note](https://www.st.com/resource/en/application_note/an5272-lsm6dsox-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf)
//! provided by ST.
//!
//! # Examples
//! ```
//! use lsm6dsox::*;
//!
//! let lsm = lsm6dsox::Lsm6dsox::new(
//!     i2c1,
//!     SlaveAddress::Low,
//!     delay,
//! );
//!
//! lsm.setup()?;
//! lsm.set_accel_sample_rate(DataRate::Freq52Hz)?;
//! lsm.set_accel_scale(AccelScale::Accel16g)?;
//! if let Ok(reading) = lsm.accel_norm() {
//!     log::info!(
//!         "Acceleration: {:?}",
//!         reading
//!     );
//! }
//! ```

mod accel;
mod gyro;
mod register;
pub mod types;

use enumflags2::BitFlags;
use register::*;
pub use types::*;

pub use accelerometer;
use accelerometer::{
    vector::{F32x3, I16x3},
    Accelerometer, RawAccelerometer,
};
use byteorder::{ByteOrder, LittleEndian};
use embedded_hal::blocking::{delay::DelayMs, i2c};

/// Trait for general LSM6DSOX configuration/functionality.
///
/// This Trait enables the user to specify the needed functionality,
/// rather then specifying a concrete [Lsm6dsox] in function signatures (which can be tricky with the generics).
///
/// # Examples
///
/// ```
/// fn takes_a_sensor(mut lsm: impl lsm6dsox::Sensor + lsm6dsox::Lsm6dsoxAccelerometer) {
///     // Everything from the Sensor and Lsm6dsoxAccelerometer traits can be used here.
/// }
/// ```
pub trait Sensor {
    /// Check whether the configured Sensor returns its correct id.
    ///
    /// Returns `Ok(id)` if `id` matches the Standard LSM6DSOX id,
    /// `Err(Some(id))` or `Err(None)` if `id` doesn't match or couldn't be read.
    fn check_id(&mut self) -> Result<u8, Option<u8>>;

    /// Initializes the sensor.
    /// A software reset is performed and common settings are applied.
    fn setup(&mut self) -> Result<(), Error>;

    /// Checks the interrupt status of all possible sources.
    ///
    /// The interrupt flags will be cleared after this check, or according to the LIR mode of the specific source.
    fn check_interrupt_sources(&mut self) -> Result<BitFlags<InterruptCause>, Error>;

    /// Maps an available interrupt source to a available interrupt line.
    ///
    /// Toggles whether a interrupt source will generate interrupts on the specified line.
    ///
    /// Note: Interrupt sources [SHUB](InterruptSource::SHUB) and [Timestamp](InterruptSource::Timestamp) are not available on both [interrupt lines](InterruptLine).
    ///
    /// Interrupts need to be enabled globally for a mapping to take effect. See [`Sensor::enable_interrupts()`].
    fn map_interrupt(
        &mut self,
        int_src: InterruptSource,
        int_line: InterruptLine,
        active: bool,
    ) -> Result<(), Error>;

    /// Enable basic interrupts
    ///
    /// Enables/disables interrupts for 6D/4D, free-fall, wake-up, tap, inactivity.
    fn enable_interrupts(&mut self, enabled: bool) -> Result<(), Error>;

    /// Sets both Accelerometer and Gyroscope in power-down mode.
    fn power_down_mode(&mut self) -> Result<(), Error>;
}

/// Trait to implement LSM6DSOX specific accelerometer functions.
pub trait Lsm6dsoxAccelerometer: Accelerometer {
    /// Sets the measurement output rate.
    fn set_accel_sample_rate(&mut self, data_rate: DataRate) -> Result<(), Error>;

    /// Sets the acceleration measurement range.
    ///
    /// Values up to this scale will be reported correctly.
    fn set_accel_scale(&mut self, scale: AccelerometerScale) -> Result<(), Error>;

    /// Sets up double-tap recognition and enables Interrupts on INT2 pin.
    ///
    /// Configures everything necessary to reasonable defaults.
    /// This includes setting the accelerometer scale to 2G, configuring power modes, setting values for thresholds
    /// and optionally mapping a interrupt pin, maps only single-tap or double-tap to the pin.
    fn setup_tap_detection(
        &mut self,
        tap_cfg: TapCfg,
        tap_mode: TapMode,
        int_line: Option<InterruptLine>,
    ) -> Result<(), Error>;

    /// Checks the tap source register.
    ///
    /// - The Register will be cleared according to the LIR setting.
    /// - The interrupt flag will be cleared after this check, or according to the LIR mode.
    /// - If LIR is set to `False` the interrupt will be set for the quiet-time window and clears automatically after that.
    fn check_tap(&mut self) -> Result<BitFlags<TapSource>, Error>;

    /// Sets the tap Threshold for each individual axis.
    ///
    /// [...] [These registers] are used to select the unsigned threshold value used to detect
    /// the tap event on the respective axis. The value of 1 LSB of these 5 bits depends on the selected accelerometer
    /// full scale: 1 LSB = (FS_XL)/(2⁵). The unsigned threshold is applied to both positive and negative slope data.[^note]
    ///
    /// [^note]: Definition from the LSM6DSOX Application Note
    fn set_tap_threshold(&mut self, x: u8, y: u8, z: u8) -> Result<(), Error>;

    /// Sets the duration of maximum time gap for double tap recognition. Default value: `0b0000`
    ///
    /// In the double-tap case, the Duration time window defines the maximum time between two consecutive detected
    /// taps. The Duration time period starts just after the completion of the Quiet time of the first tap. The `DUR[3:0]` bits
    /// of the `INT_DUR2` register are used to set the Duration time window value: the default value of these bits is `0000b`
    /// and corresponds to `16/ODR_XL` time, where `ODR_XL` is the accelerometer output data rate. If the `DUR[3:0]` bits
    /// are set to a different value, 1 LSB corresponds to `32/ODR_XL` time.[^note]
    ///
    /// [^note]: Definition from the LSM6DSOX Application Note
    fn set_tap_duration(&mut self, dur: u8) -> Result<(), Error>;

    /// Sets the expected quiet time after a tap detection. Default value: `0b00`
    ///
    /// In the double-tap case, the Quiet time window defines the time after the first tap recognition in which there must
    /// not be any overcoming threshold event. When latched mode is disabled (`LIR` bit of `TAP_CFG` is set to 0), the
    /// Quiet time also defines the length of the interrupt pulse (in both single and double-tap case).[^note]
    ///
    /// The `QUIET[1:0]` bits of the `INT_DUR2` register are used to set the Quiet time window value:
    /// the default value of these bits is `00b` and corresponds to `2/ODR_XL` time, where `ODR_XL` is the accelerometer output data rate.
    /// If the `QUIET[1:0]` bits are set to a different value, 1 LSB corresponds to `4/ODR_XL` time.[^note]
    ///
    /// [^note]: Definition from the LSM6DSOX Application Note
    fn set_tap_quiet(&mut self, quiet: u8) -> Result<(), Error>;

    /// Sets the maximum duration of overthreshold event. Default value: `0b00`
    ///
    /// The Shock time window defines the maximum duration of the overcoming threshold event: the acceleration must
    /// return below the threshold before the Shock window has expired, otherwise the tap event is not detected. The
    /// `SHOCK[1:0]` bits of the `INT_DUR2` register are used to set the Shock time window value: the default value of
    /// these bits is 00b and corresponds to `4/ODR_XL` time, where `ODR_XL` is the accelerometer output data rate. If the
    /// `SHOCK[1:0]` bits are set to a different value, 1 LSB corresponds to `8/ODR_XL` time.[^note]
    ///
    /// [^note]: Definition from the LSM6DSOX Application Note
    fn set_tap_shock(&mut self, shock: u8) -> Result<(), Error>;
}
/// Trait to implement LSM6DSOX specific gyroscope functions.
pub trait Lsm6dsoxGyroscope {
    /// Sets the measurement output rate.
    ///
    /// Note: [DataRate::Freq1Hz6] is not supported by the gyroscope and will yield an [Error::InvalidData].
    fn set_gyro_sample_rate(&mut self, odr: DataRate) -> Result<(), Error>;

    /// Sets the gyroscope measurement range.
    ///
    /// Values up to this scale will be reported correctly.
    fn set_gyro_scale(&mut self, scale: GyroscopeScale) -> Result<(), Error>;

    /// Get a angular rate reading.
    ///
    /// If no data is ready returns the appropriate [Error].
    fn angular_rate(&mut self) -> Result<AngularRate, Error>;

    /// Get a *raw* angular rate reading.
    ///
    /// If no data is ready returns the appropriate [Error].
    fn angular_rate_raw(&mut self) -> Result<RawAngularRate, Error>;
}
/// Representation of a LSM6DSOX. Stores the address and device peripherals.
pub struct Lsm6dsox<I2C, Delay>
where
    I2C: i2c::Write + i2c::WriteRead,
    Delay: DelayMs<u32>,
{
    i2c: I2C,
    address: SlaveAddress,
    delay: Delay,
    config: Configuration,
}

impl<I2C, Delay> Sensor for Lsm6dsox<I2C, Delay>
where
    I2C: i2c::Write + i2c::WriteRead,
    Delay: DelayMs<u32>,
{
    fn check_id(&mut self) -> Result<u8, Option<u8>> {
        let mut buf = [0; 1];

        match self
            .i2c
            .write_read(self.address as u8, &[Register::WHO_AM_I as u8], &mut buf)
        {
            Ok(()) => {
                if buf[0] == 0x6C {
                    Ok(buf[0])
                } else {
                    Err(Some(buf[0]))
                }
            }
            Err(_) => Err(None),
        }
    }

    // TODO Add fancy config structure
    fn setup(&mut self) -> Result<(), Error> {
        self.update_reg_command(Command::SwReset)?;

        // Give it 5 tries
        // A delay is necessary here, otherwise reset may never finish because the lsm is to busy.
        let mut buf = [1; 1];
        for _ in 0..5 {
            self.delay.delay_ms(10);
            self.i2c
                .write_read(self.address as u8, &[Register::CTRL3_C as u8], &mut buf)
                .map_err(|_| Error::I2cReadError)?;
            if buf[0] & 1 == 0 {
                break;
            }
        }

        if buf[0] & 1 != 0 {
            Err(Error::ResetFailed)
        } else {
            /* Disable I3C interface */
            self.update_reg_bits(Register::CTRL9_XL, 0x02, 0x02)?;
            self.update_reg_bits(Register::I3C_BUS_AVB, 0x00, 0b0001_1000)?;

            /* Enable Block Data Update */
            self.update_reg_bits(Register::CTRL3_C, 0b0100_0000, 0b0100_0000)?;

            /* Power-Down gyroscope */
            self.update_reg_command(Command::SetDataRateG(DataRate::PowerDown))?;
            self.config.g_odr = DataRate::PowerDown;

            /* Set Output Data Rate */
            self.set_accel_sample_rate(DataRate::Freq52Hz)?;
            // Output data rate and full scale could be set with one register update, but extra code for that seems unnecessary
            /* Set full scale */
            self.set_accel_scale(AccelerometerScale::Accel4g)?;

            /* Wait stable output */
            self.delay.delay_ms(100);

            Ok(())
        }
    }

    fn check_interrupt_sources(&mut self) -> Result<BitFlags<InterruptCause>, Error> {
        let mut buf = 0;
        self.read_reg_byte(Register::ALL_INT_SRC, &mut buf)?;
        let flags = BitFlags::from_bits(buf).map_err(|_| Error::InvalidData)?;

        Ok(flags)
    }

    fn power_down_mode(&mut self) -> core::result::Result<(), Error> {
        self.update_reg_command(Command::SetDataRateXl(DataRate::PowerDown))?;
        self.config.xl_odr = DataRate::PowerDown;

        self.update_reg_command(Command::SetDataRateG(DataRate::PowerDown))?;
        self.config.g_odr = DataRate::PowerDown;

        Ok(())
    }
    fn map_interrupt(
        &mut self,
        int_src: InterruptSource,
        int_line: InterruptLine,
        active: bool,
    ) -> Result<(), types::Error> {
        // TODO track interrupt mapping state in config
        //  This would allow us to automatically enable or disable interrupts globally.

        match (int_line, int_src) {
            (InterruptLine::INT1, InterruptSource::Timestamp) => Err(Error::NotSupported),
            (InterruptLine::INT2, InterruptSource::SHUB) => Err(Error::NotSupported),
            (_, _) => self.update_reg_command(Command::MapInterrupt(int_line, int_src, active)),
        }
    }

    fn enable_interrupts(&mut self, enabled: bool) -> Result<(), Error> {
        self.update_reg_command(Command::InterruptEnable(enabled))
    }
}

impl<I2C, Delay> Lsm6dsox<I2C, Delay>
where
    I2C: i2c::Write + i2c::WriteRead,
    Delay: DelayMs<u32>,
{
    /// Create a new instance of this sensor.
    pub fn new(i2c: I2C, address: SlaveAddress, delay: Delay) -> Self {
        Self {
            i2c,
            address,
            delay,
            config: Configuration {
                xl_odr: DataRate::PowerDown,
                xl_scale: AccelerometerScale::Accel2g,
                g_odr: DataRate::PowerDown,
                g_scale: GyroscopeScale::Dps250,
            },
        }
    }

    /// Destroy the sensor and return the hardware peripherals
    pub fn destroy(self) -> (I2C, Delay) {
        (self.i2c, self.delay)
    }

    /// Updates the bits in register `reg` specified by `bitmask` with payload `data`.
    fn update_reg_bits(&mut self, reg: Register, data: u8, bitmask: u8) -> Result<(), Error> {
        // We have to do a read of the register first to keep the bits we don't want to touch.
        let mut buf = [0; 1];
        self.i2c
            .write_read(self.address as u8, &[reg as u8], &mut buf)
            .map_err(|_| Error::I2cReadError)?;

        // First set `bitmask` bits to zero,
        buf[0] &= !bitmask;
        // then write our data to these bits.
        buf[0] |= data & bitmask;

        // A write takes the register address as first byte, the data as second.
        let update = [reg as u8, buf[0]];
        self.i2c
            .write(self.address as u8, &update)
            .map_err(|_| Error::I2cWriteError)
    }

    /// Convenience function to execute a [register::Command].
    fn update_reg_command(&mut self, command: Command) -> Result<(), Error> {
        self.update_reg_bits(command.register(), command.bits(), command.mask())
    }

    fn read_reg_byte(&mut self, reg: Register, buffer: &mut u8) -> Result<(), Error> {
        let mut tbuf = [1; 1];
        let result = self
            .i2c
            .write_read(self.address as u8, &[reg as u8], &mut tbuf)
            .map_err(|_| Error::I2cReadError);
        *buffer = tbuf[0];
        result
    }
}
