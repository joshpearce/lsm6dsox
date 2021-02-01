// Copyright 2021 Open Logistics Foundation
//
// Licensed under the Open Logistics License 1.0.
// For details on the licensing terms, see the LICENSE file.

#![no_std]


use stmems_driver_sys::*;

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

    pub fn who_am_i(&mut self) -> bool {
        let mut whoami: u8 = 0;

        let ret = unsafe {
            lsm6dsox_device_id_get(&mut self.hw.dev_ctx, &mut whoami)
        };

        if 0 == ret {
            whoami == stmems_driver_sys::LSM6DSOX_ID as u8
        } else {
            false
        }
    }
}
