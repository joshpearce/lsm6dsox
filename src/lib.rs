// Copyright 2021 Open Logistics Foundation
//
// Licensed under the Open Logistics License 1.0.
// For details on the licensing terms, see the LICENSE file.

#![no_std]

mod register;

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

    fn update_reg_bits<'a>(&mut self, reg: register::Register, data: u8, bitmask: u8) -> Result<(), &'a str> {  // lifetime has to be annotated so that self wont be borrowed as long as Err(&str) exists
        let mut buf =[0; 1];
        self.i2c.write_read(self.address as u8, &[reg as u8], &mut buf).map_err(|_| "i2c-read error")?;

        buf[0] = buf[0] & !bitmask;
        buf[0] = buf[0] & (data & bitmask);

        let update = [reg as u8, buf[0]];
        self.i2c.write(self.address as u8, &update).map_err(|_| "i2c-write error") 
        
    }  

    // convenience function to execute commands
    fn update_reg_command<'a>(&mut self, command: register::Command) -> Result<(), &'a str> {
        self.update_reg_bits(command.register(), command.bits(), command.mask())
    }

    /// Check whether the configured Sensor returns its correct id.
    pub fn who_am_i(&mut self) -> Result<(), ()> {
        let mut buf =[0; 1];

        match self.i2c.write_read(self.address as u8, &[register::Register::WHO_AM_I as u8], &mut buf) {
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

        self.update_reg_command(register::Command::SwReset)?;
        
        // Give it 10 tries. Timeout may be better here...
        let mut buf = [1; 1];
        for _ in 0..10 {
            self.i2c.write_read(self.address as u8, &[register::Register::CTRL3_C as u8], &mut buf).map_err(|_| "i2c-read error")?; 
            if buf[0] == 0 { break;}
        }

        if buf[0] != 0 {
            Err("Could not reset to default Config")
        } else {
            /* Disable I3C interface */
            self.update_reg_bits(register::Register::CTRL9_XL, 0x02, 0x02)?;
            self.update_reg_bits(register::Register::I3C_BUS_AVB, 0x00, 0b0001_1000)?;

            /* Enable Block Data Update */
            self.update_reg_bits(register::Register::CTRL3_C, 0b0100_0000, 0b0100_0000)?;

            /* Set Output Data Rate */
            self.update_reg_command(register::Command::SetDataRate(register::DataRate::ODR_52Hz))?;

            /* Set full scale */
            self.update_reg_command(register::Command::SetFullScale(register::FullScale::FS_XL_4g))?;

            /* Wait stable output */
            self.delay.delay_ms(100);

            Ok(())
        }
    }


    /// Sets the measurement output rate.
    pub fn set_output_data_rate<'a>(&mut self, datarate: register::DataRate) -> Result<(), &'a str> {
        self.update_reg_command(register::Command::SetDataRate(datarate))
    }

    // Simple functions for testing
    // The sensor and its configuration is pretty complex
    // the final implementation would look quite fancy
    pub fn read_acceleration(&mut self) -> Result<Acceleration, &str> {

        Err("not implemented yet")
        /* Check if new value available */
        /*
        let mut data_rdy: u8 = 0;
        unsafe {
            lsm6dsox_xl_flag_data_ready_get(&mut self.hw.dev_ctx, &mut data_rdy);
        }

        if data_rdy == 0 {
            Err("No Data Ready")
        } else {
            /* Read dummy data and discard it */
            let mut data_raw: [i16; 3] = [0, 0, 0];
            unsafe {
                lsm6dsox_acceleration_raw_get(&mut self.hw.dev_ctx, data_raw.as_mut_ptr());
            }

            Ok(Acceleration {
                x: unsafe { lsm6dsox_from_fs4_to_mg(data_raw[0]) } as f32,
                y: unsafe { lsm6dsox_from_fs4_to_mg(data_raw[1]) } as f32,
                z: unsafe { lsm6dsox_from_fs4_to_mg(data_raw[2]) } as f32,
            })
        } */
    }
}
