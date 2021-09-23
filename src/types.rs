// Copyright 2021 Open Logistics Foundation
//
// Licensed under the Open Logistics License 1.0.
// For details on the licensing terms, see the LICENSE file.

//! Types used by the sensor.
//!
//! Structs and Enums representing the sensors configuration, readings and states.

use crate::register;
pub use crate::register::{AccelScale, DataRate, GyroScale, TapCfg, TapMode};

/// Lsm6dsox errors
#[derive(Clone, Copy, Debug)]
pub enum Error {
    I2cWriteError,
    I2cReadError,
    ResetFailed,
    NoDataReady,
}
/// Angular rate measurement result.
#[derive(Debug)]
pub struct AngularRate {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Representation for all interrupts sources.
///
/// Reports which sources triggered an interrupt,
/// which hasen't been cleared yet.
#[derive(Debug)]
pub struct AllIntSrc {
    pub timestamp_endcount: bool,
    pub sleep_change: bool,
    pub d6d: bool,
    pub double_tap: bool,
    pub single_tap: bool,
    pub wake_up: bool,
    pub free_fall: bool,
}

/// Impl to map the raw register reading to the Struct.
#[doc(hidden)]
impl AllIntSrc {
    pub fn new(buf: u8) -> AllIntSrc {
        AllIntSrc {
            timestamp_endcount: (buf & 0b1000_0000 >> 7) != 0,
            sleep_change: (buf & 0b0010_0000 >> 5) != 0,
            d6d: (buf & 0b0001_0000 >> 4) != 0,
            double_tap: (buf & 0b0000_1000 >> 3) != 0,
            single_tap: (buf & 0b0000_0100 >> 2) != 0,
            wake_up: (buf & 0b0000_0010 >> 1) != 0,
            free_fall: (buf & 0b0000_0001 >> 0) != 0,
        }
    }
}

/// I2C Address of the sensor in use, determined by the sensors address pin.
#[derive(Clone, Copy)]
pub enum SlaveAddress {
    // Whether SDO/SA0 is pulled High or Low
    Low = 0b110_1010,
    High = 0b110_1011,
}

/// Sensor configuration state.
#[doc(hidden)]
pub struct Configuration {
    // Would it be better to use Option types here
    // to be able to have uninitialized config fields?
    pub xl_odr: register::DataRate,
    pub xl_scale: register::AccelScale,
    pub g_odr: register::DataRate,
    pub g_scale: register::GyroScale,
}
