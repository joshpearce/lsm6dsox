// Copyright 2021 Open Logistics Foundation
//
// Licensed under the Open Logistics License 1.0.
// For details on the licensing terms, see the LICENSE file.

#![no_std]

pub mod register;
use register::*;

use byteorder::{ByteOrder, LittleEndian};
use embedded_hal::blocking::{i2c, delay::DelayMs};

/// Acceleration measurement result
#[derive(Debug)]
pub struct Acceleration {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// I2C Address of the sensor in use, determined by the sensors address pin
#[derive(Clone, Copy)]
pub enum SlaveAddress {
    // Whether SDO/SA0 is pulled High or Low
    Low  = 0b110_1010,
    High = 0b110_1011,
}

/// Power Pin of the Sensors
///
/// Allows the drivers user (!) to enable/disable the sensor while it is not in use for power saving.
/// Note: Event when passing AlwaysOn, you still have to specify a type that implements the OutputPin trait due to the generics. This can be any pin from your embedded_hal
pub enum PowerActive<PwrPin: embedded_hal::digital::v2::OutputPin> {
    /// Active High power pin
    High(PwrPin),
    /// Active Low power pin
    Low(PwrPin),
    /// The power to the sensor is always on
    None
}

/// Representation of a LSM6DSOX. Stores the address and device peripherals.
pub struct Lsm6dsox<PwrPin, I2C, Delay>
where
    PwrPin: embedded_hal::digital::v2::OutputPin,
    I2C: i2c::Write + i2c::WriteRead,
    Delay: DelayMs<u32>
{
    power: PowerActive<PwrPin>,
    i2c: I2C,
    address: SlaveAddress,
    delay: Delay
}


impl<PwrPin, I2C, Delay> Lsm6dsox<PwrPin, I2C, Delay>
where
    PwrPin: embedded_hal::digital::v2::OutputPin,
    I2C: i2c::Write + i2c::WriteRead,
    Delay: DelayMs<u32> 
{
    /// Create a new instance of this sensor.
    pub fn new(power: PowerActive<PwrPin>, i2c: I2C, address: SlaveAddress, delay: Delay) -> Self {
        Self { power, i2c, address, delay }
    }

    /// Destroy the sensor and return the hardware peripherals
    pub fn destroy(self) -> (PowerActive<PwrPin>, I2C, Delay) {
        (self.power, self.i2c, self.delay)
    }

    /// Change the power state of the sensor.
    pub fn power(&mut self, on: bool) -> Result<(), PwrPin::Error> {
        // should this method do a delay when turning on the sensor?
        match (&mut self.power, on) {
            (PowerActive::High(pp), true) => pp.set_high(),
            (PowerActive::Low(pp), true) => pp.set_low(),
            (PowerActive::High(pp), false) => pp.set_low(),
            (PowerActive::Low(pp), false) => pp.set_high(),
            (_, _) => Ok(()),
        }
    }

    fn update_reg_bits<'a>(&mut self, reg: Register, data: u8, bitmask: u8) -> Result<(), &'a str> {  // lifetime has to be annotated so that self wont be borrowed as long as Err(&str) exists
        let mut buf =[0; 1];
        self.i2c.write_read(self.address as u8, &[reg as u8], &mut buf).map_err(|_| "i2c-read error")?;

        buf[0] = buf[0] & !bitmask;
        buf[0] = buf[0] & (data & bitmask);

        let update = [reg as u8, buf[0]];
        self.i2c.write(self.address as u8, &update).map_err(|_| "i2c-write error") 
        
    }  

    // convenience function to execute commands
    fn update_reg_command<'a>(&mut self, command: Command) -> Result<(), &'a str> {
        self.update_reg_bits(command.register(), command.bits(), command.mask())
    }

    fn read_reg_word<'a>(&mut self, reg: Register, buffer: &mut u8) -> Result<(), &'a str> {
        let mut tbuf = [1; 1];
        let result = self.i2c.write_read(self.address as u8, &[reg as u8], &mut tbuf).map_err(|_| "i2c-read error");
        *buffer = tbuf[0];
        result
    }

    /// Check whether the configured Sensor returns its correct id.
    pub fn who_am_i(&mut self) -> Result<(), ()> {
        let mut buf =[0; 1];

        match self.i2c.write_read(self.address as u8, &[Register::WHO_AM_I as u8], &mut buf) {
            Ok(()) => {
                if buf[0] == 0x6C { Ok(()) }
                else { Err(())}
            }
            Err(_) => {Err(())}
        }
    }

    // TODO Add fancy config structure
    /// Initializes the sensor.
    /// A software reset is performed and common settings are applied.
    pub fn setup(&mut self) -> Result<(), &str> {

        self.update_reg_command(Command::SwReset)?;
        
        // Give it 10 tries. Timeout may be better here...
        let mut buf = [1; 1];
        for _ in 0..10 {
            self.i2c.write_read(self.address as u8, &[Register::CTRL3_C as u8], &mut buf).map_err(|_| "i2c-read error")?; 
            if buf[0] == 0 { break;}
        }

        if buf[0] != 0 {
            Err("Could not reset to default Config")
        } else {
            /* Disable I3C interface */
            self.update_reg_bits(Register::CTRL9_XL, 0x02, 0x02)?;
            self.update_reg_bits(Register::I3C_BUS_AVB, 0x00, 0b0001_1000)?;

            /* Enable Block Data Update */
            self.update_reg_bits(Register::CTRL3_C, 0b0100_0000, 0b0100_0000)?;

            /* Set Output Data Rate */
            self.update_reg_command(Command::SetDataRate(DataRate::ODR_52Hz))?;

            /* Set full scale */
            self.update_reg_command(Command::SetFullScale(FullScale::FS_XL_4g))?;

            /* Wait stable output */
            self.delay.delay_ms(100);

            Ok(())
        }
    }


    /// Sets the measurement output rate.
    pub fn set_output_data_rate<'a>(&mut self, datarate: DataRate) -> Result<(), &'a str> {
        self.update_reg_command(Command::SetDataRate(datarate))
    }

    /// Simple function to read single acceleration measurement
    // The sensor and its configuration is pretty complex
    // the final implementation would look quite fancy
    pub fn read_acceleration(&mut self) -> Result<Acceleration, &str> {
        let mut data_rdy: u8 = 0;
        self.read_reg_word(Register::STATUS_REG, &mut data_rdy)?;
        if data_rdy == 0 {
            Err("No Data Ready")
        } else {
            let mut data_raw: [u8; 6] = [0; 6]; // All 3 axes x, y, z i16 values, decoded little endian, 2nd Complement 
            self.i2c.write_read(self.address as u8 ,&[Register::OUTX_L_A as u8], &mut data_raw).map_err(|_| "i2c-read error")?;
            let mut data: [f32; 3] = [0.0; 3];
            for i in 0..3 {
                data[i] = LittleEndian::read_i16(&data_raw[i*2..i*2+1]) as f32 * 0.122;
            }

            Ok(Acceleration {
                x: data[0],
                y: data[1],
                z: data[2],
            })
        }
    }
}
