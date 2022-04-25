// Copyright 2021 Open Logistics Foundation
//
// Licensed under the Open Logistics License 1.0.
// For details on the licensing terms, see the LICENSE file.

use super::*;
use measurements::AngularVelocity;

impl<I2C, Delay> Lsm6dsoxGyroscope for Lsm6dsox<I2C, Delay>
where
    I2C: i2c::Write + i2c::WriteRead,
    Delay: DelayMs<u32>,
{
    fn set_gyro_sample_rate(&mut self, data_rate: DataRate) -> Result<(), Error> {
        match data_rate {
            DataRate::Freq1Hz6 => return Err(Error::NotSupported),
            _ => {
                self.update_reg_command(Command::SetDataRateG(data_rate))?;
                self.config.g_odr = data_rate;
                Ok(())
            }
        }
    }

    fn set_gyro_scale(&mut self, scale: GyroscopeScale) -> Result<(), Error> {
        self.update_reg_command(Command::SetGyroScale(scale))?;
        self.config.g_scale = scale;
        Ok(())
    }

    fn angular_rate(&mut self) -> Result<AngularRate, Error> {
        let mut data_rdy: u8 = 0;
        self.read_reg_byte(Register::STATUS_REG, &mut data_rdy)?;
        if (data_rdy & 0b000_010) == 0 {
            Err(Error::NoDataReady)
        } else {
            let mut data_raw: [u8; 6] = [0; 6]; // All 3 axes x, y, z i16 values, decoded little endian, 2nd Complement
            self.i2c
                .write_read(
                    self.address as u8,
                    &[Register::OUTX_L_G as u8],
                    &mut data_raw,
                )
                .map_err(|_| Error::I2cReadError)?;
            let mut data: [f32; 3] = [0.0; 3];
            let factor = self.config.g_scale.to_factor();
            for i in 0..3 {
                data[i] = LittleEndian::read_i16(&data_raw[i * 2..i * 2 + 2]) as f32 * factor;
            }


            Ok(AngularRate {
                x: AngularVelocity::from_hertz((data[0]/360.0).into()),
                y: AngularVelocity::from_hertz((data[1]/360.0).into()),
                z: AngularVelocity::from_hertz((data[2]/360.0).into()),
            })
        }
    }

    fn angular_rate_raw(&mut self) -> Result<RawAngularRate, Error> {
        todo!()
    }
}
