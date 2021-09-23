// Copyright 2021 Open Logistics Foundation
//
// Licensed under the Open Logistics License 1.0.
// For details on the licensing terms, see the LICENSE file.

use super::*;

impl<I2C, Delay> Gyroscope for Lsm6dsox<I2C, Delay>
where
    I2C: i2c::Write + i2c::WriteRead,
    Delay: DelayMs<u32>,
{
    fn set_gyro_sample_rate(&mut self, data_rate: DataRate) -> Result<(), Error> {
        self.update_reg_command(Command::SetDataRateG(data_rate))?;
        self.config.g_odr = data_rate;
        Ok(())
    }

    fn set_gyro_scale(&mut self, scale: GyroScale) -> Result<(), Error> {
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
            for i in 0..3 {
                match self.config.g_scale {
                    // Following values have be copied from the official ST driver.
                    // They correspond nicely to the scarce examples from the application note,
                    // but otherwise doesnt make much sense (calculate scale/32767 and you'll get slightly different values)
                    GyroScale::FS_G_125dps => {
                        data[i] =
                            LittleEndian::read_i16(&data_raw[i * 2..i * 2 + 2]) as f32 * 0.004375
                    }
                    GyroScale::FS_G_250dps => {
                        data[i] =
                            LittleEndian::read_i16(&data_raw[i * 2..i * 2 + 2]) as f32 * 0.008750
                    }
                    GyroScale::FS_G_500dps => {
                        data[i] =
                            LittleEndian::read_i16(&data_raw[i * 2..i * 2 + 2]) as f32 * 0.01750
                    }
                    GyroScale::FS_G_1000dps => {
                        data[i] =
                            LittleEndian::read_i16(&data_raw[i * 2..i * 2 + 2]) as f32 * 0.0350
                    }
                    GyroScale::FS_G_2000dps => {
                        data[i] = LittleEndian::read_i16(&data_raw[i * 2..i * 2 + 2]) as f32 * 0.070
                    }
                }
            }

            Ok(AngularRate {
                x: data[0],
                y: data[1],
                z: data[2],
            })
        }
    }
}
