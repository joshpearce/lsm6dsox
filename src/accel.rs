// Copyright 2021 Open Logistics Foundation
//
// Licensed under the Open Logistics License 1.0.
// For details on the licensing terms, see the LICENSE file.

use core::convert::TryFrom;

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
            let factor = self.config.xl_scale.to_factor();
            // Now convert the raw i16 values to f32 engineering units
            for i in 0..3 {
                data[i] = LittleEndian::read_i16(&data_raw[i * 2..i * 2 + 2]) as f32 * factor
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

        let sample_rate = DataRate::try_from(buf[0] & 0xF0)
            .map_err(|_| accelerometer::Error::new(accelerometer::ErrorKind::Device))?;
        self.config.xl_odr = sample_rate;
        Ok(sample_rate.into())
    }
}

impl<I2C, Delay> RawAccelerometer<I16x3> for Lsm6dsox<I2C, Delay>
where
    I2C: i2c::Write + i2c::WriteRead,
    Delay: DelayMs<u32>,
{
    type Error = Error;

    fn accel_raw(
        &mut self,
    ) -> Result<accelerometer::vector::I16x3, accelerometer::Error<Self::Error>> {
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
            let mut data: [i16; 3] = [0; 3];
            // Now convert the raw i16 values to f32 engineering units
            for i in 0..3 {
                data[i] = LittleEndian::read_i16(&data_raw[i * 2..i * 2 + 2])
            }

            Ok(I16x3::new(data[0], data[1], data[2]))
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

    fn set_accel_scale(&mut self, scale: AccelerometerScale) -> Result<(), Error> {
        self.update_reg_command(Command::SetAccelScale(scale))?;
        self.config.xl_scale = scale;
        Ok(())
    }

    fn setup_tap_detection(
        &mut self,
        tap_cfg: TapCfg,
        tap_mode: TapMode,
        int_line: Option<InterruptLine>,
    ) -> Result<(), Error> {
        /* Set Output Data Rate */
        self.update_reg_command(Command::SetDataRateXl(DataRate::Freq104Hz))?;
        // Output data rate and full scale could be set with one register update, maybe change this
        /* Set full scale */
        self.update_reg_command(Command::SetAccelScale(AccelerometerScale::Accel2g))?;
        self.config.xl_scale = AccelerometerScale::Accel2g;

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
        if let Some(int_line) = int_line {
            self.update_reg_command(Command::InterruptEnable(true))?; // This must always be enabled
            match (tap_mode, int_line) {
                (TapMode::Single, line) => self.update_reg_command(Command::MapInterrupt(
                    line,
                    InterruptSource::SingleTap,
                    true,
                ))?,
                (TapMode::SingleAndDouble, line) => self.update_reg_command(
                    Command::MapInterrupt(line, InterruptSource::DoubleTap, true),
                )?,
            }
        }
        Ok(())
    }

    fn check_tap(&mut self) -> Result<BitFlags<TapSource>, Error> {
        let mut buf = 0;
        self.read_reg_byte(Register::TAP_SRC, &mut buf)?;

        BitFlags::from_bits(buf).map_err(|_| Error::InvalidData)
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
