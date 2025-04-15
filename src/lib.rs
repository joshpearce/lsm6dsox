// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3

// Modified by JJP - removed Delay from Lsm6dsox. It's only used in set_up now.

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
//! # Resources
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
//! # Features
//!
//! - [`Accelerometer`](https://docs.rs/accelerometer/latest/accelerometer/trait.Accelerometer.html) trait implementation
//! - [`embedded-hal`](https://crates.io/crates/embedded-hal) I²C support
//! - Gyroscope
//! - Tap recognition
//! - Interrupts
//! - Further features may be added in the future
//!
//! # Examples
//! ```no_run
//! # extern crate embedded_hal;
//! # extern crate embedded_hal_mock;
//! # use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
//! # use embedded_hal_mock::eh1::delay::NoopDelay;
//! # type I2cError = <embedded_hal_mock::common::Generic<embedded_hal_mock::eh1::i2c::Transaction> as embedded_hal::i2c::ErrorType>::Error;
//! use accelerometer::Accelerometer;
//! use lsm6dsox::*;
//!
//! # fn main() {
//! # example().unwrap();
//! # }
//! # fn example() -> Result<(), Error<I2cError>> {
//! # let i2c = I2cMock::new(&Vec::new());
//! # let mut delay = NoopDelay::new();
//! let mut lsm = lsm6dsox::Lsm6dsox::new(i2c, SlaveAddress::Low);
//!
//! lsm.setup(&mut delay)?;
//! lsm.set_accel_sample_rate(DataRate::Freq52Hz)?;
//! lsm.set_accel_scale(AccelerometerScale::Accel16g)?;
//! if let Ok(reading) = lsm.accel_norm() {
//!     println!("Acceleration: {:?}", reading);
//! }
//! # Ok(())
//! # }
//! ```
//! # License
//!
//! Open Logistics Foundation License\
//! Version 1.3, January 2023
//!
//! See the LICENSE file in the top-level directory.
//!
//! # Contact
//!
//! Fraunhofer IML Embedded Rust Group - <embedded-rust@iml.fraunhofer.de>

mod accel;
mod gyro;
pub mod register;
pub mod types;

pub use register::*;
pub use types::*;

pub use accelerometer;
use accelerometer::{
    vector::{F32x3, I16x3},
    Accelerometer, RawAccelerometer,
};
use byteorder::{ByteOrder, LittleEndian};
use embedded_hal::{delay::DelayNs, i2c::I2c};
use enumflags2::BitFlags;

/// Representation of a LSM6DSOX. Stores the device peripherals and current configuration.
pub struct Lsm6dsox<I2C>
where
    I2C: I2c,
{
    config: Configuration,
    registers: RegisterAccess<I2C>,
}

impl<I2C> Lsm6dsox<I2C>
where
    I2C: I2c,
{
    /// Create a new instance of this sensor.
    pub fn new(i2c: I2C, address: SlaveAddress) -> Self {
        Self {
            config: Configuration {
                xl_odr: DataRate::PowerDown,
                xl_scale: AccelerometerScale::Accel2g,
                g_odr: DataRate::PowerDown,
                g_scale: GyroscopeScale::Dps250,
            },
            registers: RegisterAccess::new(i2c, address),
        }
    }

    /// Destroy the sensor and return the underlying I²C interface.
    pub fn destroy(self) -> I2C {
        self.registers.destroy()
    }

    /// Check whether the configured sensor returns its correct id.
    ///
    /// Returns `Ok(id)` if `id` matches the standard LSM6DSOX id,
    /// `Err(Some(id))` if the id doesn't match, or `Err(None)` if it couldn't be read.
    pub fn check_id(&mut self) -> Result<u8, Option<u8>> {
        match self.registers.read_reg(PrimaryRegister::WHO_AM_I) {
            Ok(val) => {
                if val == 0x6C {
                    Ok(val)
                } else {
                    Err(Some(val))
                }
            }
            Err(_) => Err(None),
        }
    }

    /// Initializes the sensor.
    ///
    /// Performs a software reset and applies common settings.
    /// The accelerometer and gyroscope are initialized with [`DataRate::PowerDown`].
    ///
    /// This method takes a mutable reference to a delay implementation.
    pub fn setup<Delay>(&mut self, delay: &mut Delay) -> Result<(), Error<I2C::Error>>
    where
        Delay: DelayNs,
    {
        self.update_reg_command(Command::SwReset)?;

        // Give it 5 tries.
        // A delay is necessary here, otherwise reset may never finish because the LSM6DSOX is too busy.
        let mut ctrl3_c_val = 0xFF;
        for _ in 0..5 {
            delay.delay_ms(10);
            ctrl3_c_val = self.registers.read_reg(PrimaryRegister::CTRL3_C)?;
            if ctrl3_c_val & 1 == 0 {
                break;
            }
        }

        if ctrl3_c_val & 1 != 0 {
            Err(Error::ResetFailed)
        } else {
            /* Disable I3C interface */
            self.registers
                .update_reg(PrimaryRegister::CTRL9_XL, 0x02, 0x02)?;
            self.registers
                .update_reg(PrimaryRegister::I3C_BUS_AVB, 0x00, 0b0001_1000)?;

            /* Enable Block Data Update */
            self.registers
                .update_reg(PrimaryRegister::CTRL3_C, 0b0100_0000, 0b0100_0000)?;

            self.set_accel_sample_rate(self.config.xl_odr)?;
            self.set_accel_scale(self.config.xl_scale)?;
            self.set_gyro_sample_rate(self.config.g_odr)?;
            self.set_gyro_scale(self.config.g_scale)?;

            /* Wait for stable output */
            delay.delay_ms(100);

            Ok(())
        }
    }

    /// Checks the interrupt status of all possible sources.
    ///
    /// The interrupt flags will be cleared after this check, or according to the LIR mode of the specific source.
    pub fn check_interrupt_sources(
        &mut self,
    ) -> Result<BitFlags<InterruptCause>, Error<I2C::Error>> {
        let all_int_src = self.registers.read_reg(PrimaryRegister::ALL_INT_SRC)?;
        let flags = BitFlags::from_bits(all_int_src).map_err(|_| Error::InvalidData)?;

        Ok(flags)
    }

    /// Sets both the accelerometer and gyroscope into power-down mode.
    pub fn power_down_mode(&mut self) -> core::result::Result<(), Error<I2C::Error>> {
        self.update_reg_command(Command::SetDataRateXl(DataRate::PowerDown))?;
        self.config.xl_odr = DataRate::PowerDown;

        self.update_reg_command(Command::SetDataRateG(DataRate::PowerDown))?;
        self.config.g_odr = DataRate::PowerDown;

        Ok(())
    }

    /// Maps an available interrupt source to a specific interrupt line.
    ///
    /// Toggles whether an interrupt source will generate interrupts on the specified line.
    ///
    /// Note: Interrupt sources [SHUB](InterruptSource::SHUB) and [Timestamp](InterruptSource::Timestamp) are not available on both [interrupt lines](InterruptLine).
    ///
    /// Interrupts need to be enabled globally for a mapping to take effect. See [`Lsm6dsox::enable_interrupts()`].
    pub fn map_interrupt(
        &mut self,
        int_src: InterruptSource,
        int_line: InterruptLine,
        active: bool,
    ) -> Result<(), types::Error<I2C::Error>> {
        // TODO: Track interrupt mapping state in the configuration
        //       This would allow us to automatically enable or disable interrupts globally.

        match (int_line, int_src) {
            (InterruptLine::INT1, InterruptSource::Timestamp) => Err(Error::NotSupported),
            (InterruptLine::INT2, InterruptSource::SHUB) => Err(Error::NotSupported),
            (_, _) => self.update_reg_command(Command::MapInterrupt(int_line, int_src, active)),
        }
    }

    /// Enables or disables basic interrupts.
    ///
    /// This enables/disables interrupts for 6D/4D, free-fall, wake-up, tap, and inactivity.
    pub fn enable_interrupts(&mut self, enabled: bool) -> Result<(), Error<I2C::Error>> {
        self.update_reg_command(Command::InterruptEnable(enabled))
    }

    /// Updates a register according to the provided [`Command`].
    fn update_reg_command(&mut self, command: Command) -> Result<(), Error<I2C::Error>> {
        self.registers
            .update_reg(command.register(), command.bits(), command.mask())
    }

    /// Provides direct access to the internal sensor registers.
    ///
    /// # Safety
    ///
    /// Direct register access may misconfigure the sensor or invalidate some of the driver's assumptions.
    /// It is provided mainly for testing, debugging, or advanced configuration scenarios.
    pub unsafe fn register_access(&mut self) -> &mut RegisterAccess<I2C> {
        &mut self.registers
    }
}
