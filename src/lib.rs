// Copyright 2021 Open Logistics Foundation
//
// Licensed under the Open Logistics License 1.0.
// For details on the licensing terms, see the LICENSE file.

#![no_std]

use stmems_driver_sys::*;

#[derive(Debug)]
pub struct Acceleration {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

pub struct HardwareContext {
    pub power: fn(on: bool),
    pub delay: fn(ms: u32),
    pub dev_ctx: stmdev_ctx_t,
}

pub struct Device {
    hw: HardwareContext,
}

impl Device {
    pub fn new(hw: HardwareContext) -> Self {
        hw.dev_ctx.write_reg.expect("write_reg function mising");
        hw.dev_ctx.read_reg.expect("read_reg function mising");

        Self { hw: hw }
    }

    pub fn power(&self, on: bool) {
        (self.hw.power)(on);

        if on {
            (self.hw.delay)(10); // 10ms Boot Time as in STs examples
        }
    }

    // Basically all function will burrow self as mut since ST refuses to use the const keyword in their drivers
    pub fn who_am_i(&mut self) -> Result<(), ()> {
        let mut whoami: u8 = 0;

        let ret = unsafe { lsm6dsox_device_id_get(&mut self.hw.dev_ctx, &mut whoami) };

        if 0 == ret {
            if whoami == stmems_driver_sys::LSM6DSOX_ID as u8 {
                Ok(())
            } else {
                Err(())
            }
        } else {
            Err(())
        }
    }

    // TODO Add fancy config structure
    pub fn setup(&mut self) -> Result<(), &str> {
        /* Restore default configuration */
        unsafe {
            lsm6dsox_reset_set(&mut self.hw.dev_ctx, PROPERTY_ENABLE as u8);
        }

        let mut rst: u8 = 1;

        for _i in 0..10 {
            // Give it 10 tries. Maybe introduce timeout instead
            unsafe {
                lsm6dsox_reset_get(&mut self.hw.dev_ctx, &mut rst);
            }
            if rst == 0 {
                break;
            }
        }

        if rst != 0 {
            Err("Could not reset to default Config")
        } else {
            /* Disable I3C interface */
            unsafe {
                lsm6dsox_i3c_disable_set(
                    &mut self.hw.dev_ctx,
                    lsm6dsox_i3c_disable_t_LSM6DSOX_I3C_DISABLE,
                );
            }
            /* Enable Block Data Update */
            unsafe {
                lsm6dsox_block_data_update_set(&mut self.hw.dev_ctx, PROPERTY_ENABLE as u8);
            }

            /* Set Output Data Rate */
            unsafe {
                lsm6dsox_xl_data_rate_set(
                    &mut self.hw.dev_ctx,
                    lsm6dsox_odr_xl_t_LSM6DSOX_XL_ODR_52Hz,
                );
            }
            /* Set full scale */
            unsafe {
                lsm6dsox_xl_full_scale_set(&mut self.hw.dev_ctx, lsm6dsox_fs_xl_t_LSM6DSOX_4g);
            }
            /* Wait stable output */
            (self.hw.delay)(100);

            Ok(())
        }
    }

    // Simple functions for testing
    // The sensor and its configuration is pretty complex
    // the final implementation would look quite fancy
    pub fn read_acceleration(&mut self) -> Result<Acceleration, &str> {
        /* Check if new value available */
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
        }
    }
}
