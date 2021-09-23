// Copyright 2021 Open Logistics Foundation
//
// Licensed under the Open Logistics License 1.0.
// For details on the licensing terms, see the LICENSE file.

use super::*;

impl<I2C, Delay> Accelerometer for Lsm6dsox<I2C, Delay>
where
    I2C: i2c::Write + i2c::WriteRead,
    Delay: DelayMs<u32>,
{
    type Error = Error;

    fn accel_norm(
        &mut self,
    ) -> Result<accelerometer::vector::F32x3, accelerometer::Error<Self::Error>> {
        // First read the status register to determine if data is available,
        // if so, read the six bytes of data and convert it.
        let mut data_rdy: u8 = 0;
        self.read_reg_byte(Register::STATUS_REG, &mut data_rdy)?;
        if (data_rdy & 0b0000_0001) == 0 {
            // check if XLDA bit is set in STATUS_REG
            Err(accelerometer::Error::new_with_cause(
                accelerometer::ErrorKind::Mode,
                Error::NoDataReady,
            ))
        } else {
            let mut data_raw: [u8; 6] = [0; 6]; // All 3 axes x, y, z i16 values, decoded little endian, 2nd Complement
            self.i2c
                .write_read(
                    self.address as u8,
                    &[Register::OUTX_L_A as u8],
                    &mut data_raw,
                )
                .map_err(|_| Error::I2cReadError)?;
            let mut data: [f32; 3] = [0.0; 3];
            // Now convert the raw i16 values to f32 engineering units
            for i in 0..3 {
                match self.config.xl_scale {
                    AccelScale::FS_XL_2g => {
                        data[i] =
                            LittleEndian::read_i16(&data_raw[i * 2..i * 2 + 2]) as f32 * 0.000061
                        // If the i16 represents a range of -2G to +2G range multiply by the given factor.
                        // Corresponds to 2 / 0x7FFF (rounded).
                        // Same goes for the other ranges.
                    }
                    AccelScale::FS_XL_16g => {
                        data[i] =
                            LittleEndian::read_i16(&data_raw[i * 2..i * 2 + 2]) as f32 * 0.0006714
                    }
                    AccelScale::FS_XL_4g => {
                        data[i] =
                            LittleEndian::read_i16(&data_raw[i * 2..i * 2 + 2]) as f32 * 0.000122
                    }
                    AccelScale::FS_XL_8g => {
                        data[i] =
                            LittleEndian::read_i16(&data_raw[i * 2..i * 2 + 2]) as f32 * 0.000244
                    }
                }
            }

            Ok(F32x3::new(data[0], data[1], data[2]))
        }
    }

    fn sample_rate(&mut self) -> Result<f32, accelerometer::Error<Self::Error>> {
        // Read the sample rate first, update the config if necessary and then report the current rate.
        // Since the user shouldn't have to update the config manually,
        // this is done here in case the config differs from the device state (which it shouldn't, but who knows).
        // It may be better to only query the config here if this function is called often.
        let mut buf = [0; 1];
        self.i2c
            .write_read(self.address as u8, &[Register::CTRL1_XL as u8], &mut buf)
            .map_err(|_| Error::I2cReadError)?;

        // This match compares the raw sample rate readout with the DataRate enum.
        // Rust lacks the ability to match an enum with a int directly,
        // so it has to be done with this construct (more or less a more readible nested if).
        match buf[0] & 0xF0 {
            x if x == DataRate::ODR_PD as u8 => {
                self.config.xl_odr = DataRate::ODR_PD;
                Ok(0.0)
            }
            x if x == DataRate::ODR_1Hz6 as u8 => {
                self.config.xl_odr = DataRate::ODR_1Hz6;
                Ok(1.6)
            }
            x if x == DataRate::ODR_12Hz5 as u8 => {
                self.config.xl_odr = DataRate::ODR_12Hz5;
                Ok(12.5)
            }
            x if x == DataRate::ODR_26Hz as u8 => {
                self.config.xl_odr = DataRate::ODR_26Hz;
                Ok(26.0)
            }
            x if x == DataRate::ODR_52Hz as u8 => {
                self.config.xl_odr = DataRate::ODR_52Hz;
                Ok(52.0)
            }
            x if x == DataRate::ODR_104Hz as u8 => {
                self.config.xl_odr = DataRate::ODR_104Hz;
                Ok(104.0)
            }
            x if x == DataRate::ODR_208Hz as u8 => {
                self.config.xl_odr = DataRate::ODR_208Hz;
                Ok(208.0)
            }
            x if x == DataRate::ODR_416Hz as u8 => {
                self.config.xl_odr = DataRate::ODR_416Hz;
                Ok(416.0)
            }
            x if x == DataRate::ODR_833Hz as u8 => {
                self.config.xl_odr = DataRate::ODR_833Hz;
                Ok(833.0)
            }
            x if x == DataRate::ODR_1kHz66 as u8 => {
                self.config.xl_odr = DataRate::ODR_1kHz66;
                Ok(1660.0)
            }
            x if x == DataRate::ODR_3kHz33 as u8 => {
                self.config.xl_odr = DataRate::ODR_3kHz33;
                Ok(3330.0)
            }
            x if x == DataRate::ODR_6kHz66 as u8 => {
                self.config.xl_odr = DataRate::ODR_6kHz66;
                Ok(6660.0)
            }
            // in all other cases the sensor reported an unspecified/invalid data rate.
            _ => Err(accelerometer::Error::new(accelerometer::ErrorKind::Device)),
        }
    }
}

impl<I2C, Delay> Lsm6dsoxAccelerometer for Lsm6dsox<I2C, Delay>
where
    I2C: i2c::Write + i2c::WriteRead,
    Delay: DelayMs<u32>,
{
    fn set_accel_sample_rate(&mut self, data_rate: DataRate) -> Result<(), Error> {
        self.update_reg_command(Command::SetDataRateXl(data_rate))?;
        self.config.xl_odr = data_rate;
        Ok(())
    }

    fn set_accel_scale(&mut self, scale: AccelScale) -> Result<(), Error> {
        self.update_reg_command(Command::SetAccelScale(scale))?;
        self.config.xl_scale = scale;
        Ok(())
    }

    fn setup_tap_detection(&mut self, tap_cfg: TapCfg, tap_mode: TapMode) -> Result<(), Error> {
        /* Set Output Data Rate */
        self.update_reg_command(Command::SetDataRateXl(DataRate::ODR_104Hz))?;
        // Output data rate and full scale could be set with one register update, maybe change this
        /* Set full scale */
        self.update_reg_command(Command::SetAccelScale(AccelScale::FS_XL_2g))?;
        self.config.xl_scale = AccelScale::FS_XL_2g;

        /*
        Set XL_ULP_EN = 1 and XL_HM_MODE = 0 = high-performance mode
        Refer to application note table 8 and 9 for further information on operating modes.
        */
        self.update_reg_bits(Register::CTRL5_C, 0b1000_0000, 0b1000_0000)?;
        self.update_reg_bits(Register::CTRL6_C, 0b0000_0000, 0b0000_0000)?;

        /* Enable tap detection */
        self.update_reg_command(Command::TapEnable(tap_cfg))?;

        /* Set tap threshold 0x08 = 500mg for configured FS_XL*/
        self.update_reg_command(Command::TapThreshold(Axis::X, 0x08))?;
        self.update_reg_command(Command::TapThreshold(Axis::Y, 0x08))?;
        self.update_reg_command(Command::TapThreshold(Axis::Z, 0x08))?;

        self.update_reg_command(Command::TapDuration(0x07))?;
        self.update_reg_command(Command::TapQuiet(0x03))?;
        self.update_reg_command(Command::TapShock(0x03))?;

        self.update_reg_command(Command::TapMode(tap_mode))?;

        /* Enable Interrupts */
        self.update_reg_command(Command::InterruptEnable(true))?; // This must always be enabled
        self.update_reg_command(Command::MapInterrupt(
            InterruptLine::INT2,
            InterruptSource::DoubleTap,
            true,
        ))
    }

    fn check_tap(&mut self) -> Result<bool, Error> {
        let mut buf = 0;
        self.read_reg_byte(Register::TAP_SRC, &mut buf)?;

        if buf & 0x10 == 0x10 {
            Ok(true)
        } else {
            Ok(false)
        }
    }

    fn set_tap_threshold(&mut self, x: u8, y: u8, z: u8) -> Result<(), Error> {
        self.update_reg_command(Command::TapThreshold(Axis::X, x))?;
        self.update_reg_command(Command::TapThreshold(Axis::Y, y))?;
        self.update_reg_command(Command::TapThreshold(Axis::Z, z))
    }

    fn set_tap_duration(&mut self, dur: u8) -> Result<(), Error> {
        self.update_reg_command(Command::TapDuration(dur))
    }

    fn set_tap_quiet(&mut self, quiet: u8) -> Result<(), Error> {
        self.update_reg_command(Command::TapQuiet(quiet))
    }

    fn set_tap_shock(&mut self, shock: u8) -> Result<(), Error> {
        self.update_reg_command(Command::TapShock(shock))
    }
}
