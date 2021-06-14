// Copyright 2021 Open Logistics Foundation
//
// Licensed under the Open Logistics License 1.0.
// For details on the licensing terms, see the LICENSE file.

/// Registers which are accessible from the primary SPI/I2C/MIPI I3C interfaces.
///
/// **Note:** These Register names correspond to the normal function registers.
/// When embedded function access is enabled in `FUNC_CFG_ACCESS` these adresses correspond to different registers.
#[allow(non_camel_case_types)]
#[allow(dead_code)]
#[derive(Clone, Copy)]
pub enum Register {
    I2C_ADD_L = 0xD5,
    I2C_ADD_H = 0xD7,
    ID = 0x6C,

    FUNC_CFG_ACCESS = 0x01,
    PIN_CTRL = 0x02,
    S4S_TPH_L = 0x04,
    S4S_TPH_H = 0x05,
    S4S_RR = 0x06,
    FIFO_CTRL1 = 0x07,
    FIFO_CTRL2 = 0x08,
    FIFO_CTRL3 = 0x09,
    FIFO_CTRL4 = 0x0A,
    COUNTER_BDR_REG1 = 0x0B,
    COUNTER_BDR_REG2 = 0x0C,
    INT1_CTRL = 0x0D,
    INT2_CTRL = 0x0E,
    WHO_AM_I = 0x0F,
    CTRL1_XL = 0x10,
    CTRL2_G = 0x11,
    CTRL3_C = 0x12,
    CTRL4_C = 0x13,
    CTRL5_C = 0x14,
    CTRL6_C = 0x15,
    CTRL7_G = 0x16,
    CTRL8_XL = 0x17,
    CTRL9_XL = 0x18,
    CTRL10_C = 0x19,
    ALL_INT_SRC = 0x1A,
    WAKE_UP_SRC = 0x1B,
    TAP_SRC = 0x1C,
    D6D_SRC = 0x1D,
    STATUS_REG = 0x1E,
    OUT_TEMP_L = 0x20,
    OUT_TEMP_H = 0x21,
    OUTX_L_G = 0x22,
    OUTX_H_G = 0x23,
    OUTY_L_G = 0x24,
    OUTY_H_G = 0x25,
    OUTZ_L_G = 0x26,
    OUTZ_H_G = 0x27,
    OUTX_L_A = 0x28,
    OUTX_H_A = 0x29,
    OUTY_L_A = 0x2A,
    OUTY_H_A = 0x2B,
    OUTZ_L_A = 0x2C,
    OUTZ_H_A = 0x2D,
    EMB_FUNC_STATUS_MAINPAGE = 0x35,
    FSM_STATUS_A_MAINPAGE = 0x36,
    FSM_STATUS_B_MAINPAGE = 0x37,
    MLC_STATUS_MAINPAGE = 0x38,
    STATUS_MASTER_MAINPAGE = 0x39,
    FIFO_STATUS1 = 0x3A,
    FIFO_STATUS2 = 0x3B,
    TIMESTAMP0 = 0x40,
    TIMESTAMP1 = 0x41,
    TIMESTAMP2 = 0x42,
    TIMESTAMP3 = 0x43,
    UI_STATUS_REG_OIS = 0x49,
    UI_OUTX_L_G_OIS = 0x4A,
    UI_OUTX_H_G_OIS = 0x4B,
    UI_OUTY_L_G_OIS = 0x4C,
    UI_OUTY_H_G_OIS = 0x4D,
    UI_OUTZ_L_G_OIS = 0x4E,
    UI_OUTZ_H_G_OIS = 0x4F,
    UI_OUTX_L_A_OIS = 0x50,
    UI_OUTX_H_A_OIS = 0x51,
    UI_OUTY_L_A_OIS = 0x52,
    UI_OUTY_H_A_OIS = 0x53,
    UI_OUTZ_L_A_OIS = 0x54,
    UI_OUTZ_H_A_OIS = 0x55,
    TAP_CFG0 = 0x56,
    TAP_CFG1 = 0x57,
    TAP_CFG2 = 0x58,
    TAP_THS_6D = 0x59,
    INT_DUR2 = 0x5A,
    WAKE_UP_THS = 0x5B,
    WAKE_UP_DUR = 0x5C,
    FREE_FALL = 0x5D,
    MD1_CFG = 0x5E,
    MD2_CFG = 0x5F,
    S4S_ST_CMD_CODE = 0x60,
    S4S_DT_REG = 0x61,
    I3C_BUS_AVB = 0x62,
    INTERNAL_FREQ_FINE = 0x63,
    UI_INT_OIS = 0x6F,
    UI_CTRL1_OIS = 0x70,
    UI_CTRL2_OIS = 0x71,
    UI_CTRL3_OIS = 0x72,
    X_OFS_USR = 0x73,
    Y_OFS_USR = 0x74,
    Z_OFS_USR = 0x75,
    FIFO_DATA_OUT_TAG = 0x78,
    FIFO_DATA_OUT_X_L = 0x79,
    FIFO_DATA_OUT_X_H = 0x7A,
    FIFO_DATA_OUT_Y_L = 0x7B,
    FIFO_DATA_OUT_Y_H = 0x7C,
    FIFO_DATA_OUT_Z_L = 0x7D,
    FIFO_DATA_OUT_Z_H = 0x7E,
}

/// Possible output data rates
/// Corresponding to register ```CTRL1_XL```
#[allow(non_camel_case_types)]
#[allow(dead_code)]
#[derive(Clone, Copy)]
pub enum DataRate {
    /// Power down state
    ODR_PD = 0b0000_0000,
    ODR_1Hz6 = 0b1011_0000,
    ODR_12Hz5 = 0b0001_0000,
    ODR_26Hz = 0b0010_0000,
    ODR_52Hz = 0b0011_0000,
    ODR_104Hz = 0b0100_0000,
    ODR_208Hz = 0b0101_0000,
    ODR_416Hz = 0b0110_0000,
    ODR_833Hz = 0b0111_0000,
    ODR_1kHz66 = 0b1000_0000,
    ODR_3kHz33 = 0b1001_0000,
    ODR_6kHz66 = 0b1010_0000,
}

/// Possible output ranges.
/// Corresponding to register ```CTRL1_XL```
#[allow(non_camel_case_types)]
#[allow(dead_code)]
#[derive(Clone, Copy)]
pub enum FullScale {
    FS_XL_2g = 0b0000_0000,
    /// When ```XL_FS_MODE``` in ```CTRL8_XL``` is set to 1, ```FS_XL_16g``` set scale to 2g.
    FS_XL_16g = 0b0000_0100,
    FS_XL_4g = 0b0000_1000,
    FS_XL_8g = 0b0000_1100,
}

/// Generic axis enum to map various commands.
pub enum Axis {
    X,
    Y,
    Z,
}

pub enum TapMode {
    Single,
    SingleAndDouble,
}

/// Representation of interrupt pins 1 and 2.
pub enum InterruptLine {
    INT1,
    INT2,
}

/// Interrupt sources which can be routed to interrupt pins.
/// Note: This is incomplete right now.
pub enum InterruptSource {
    SingleTap,
    DoubleTap,
}

// Maybe this command structure is not a good way, since the register structure leads to complex commands
// TODO find a better way to simplify Commands, e.g. define bitmasks for the register bits
pub enum Command {
    SwReset,
    SetDataRate(DataRate),
    SetFullScale(FullScale),
    TapEnable(bool, bool, bool),
    TapThreshold(Axis, u8),
    TapDuration(u8),
    TapQuiet(u8),
    TapShock(u8),
    TapMode(TapMode),
    InterruptEnable(bool),
    MapInterrupt(InterruptLine, InterruptSource, bool),
}

impl Command {
    pub fn register(&self) -> Register {
        match *self {
            Command::SwReset => Register::CTRL3_C,
            Command::SetDataRate(_) => Register::CTRL1_XL,
            Command::SetFullScale(_) => Register::CTRL1_XL,
            Command::TapEnable(_, _, _) => Register::TAP_CFG0,
            Command::TapThreshold(Axis::X, _) => Register::TAP_CFG1,
            Command::TapThreshold(Axis::Y, _) => Register::TAP_CFG2,
            Command::TapThreshold(Axis::Z, _) => Register::TAP_THS_6D,
            Command::TapDuration(_) => Register::INT_DUR2,
            Command::TapQuiet(_) => Register::INT_DUR2,
            Command::TapShock(_) => Register::INT_DUR2,
            Command::TapMode(_) => Register::WAKE_UP_THS,
            Command::InterruptEnable(_) => Register::TAP_CFG2,
            Command::MapInterrupt(InterruptLine::INT1, _, _) => Register::MD1_CFG,
            Command::MapInterrupt(InterruptLine::INT2, _, _) => Register::MD2_CFG,
        }
    }
    pub fn bits(&self) -> u8 {
        match *self {
            Command::SwReset => 0x01,
            Command::SetDataRate(dr) => dr as u8,
            Command::SetFullScale(fs) => fs as u8,
            Command::TapEnable(x, y, z) => (x as u8) << 3 | (y as u8) << 2 | (z as u8) << 1,
            Command::TapThreshold(_, value) => value,
            Command::TapDuration(value) => value << 4,
            Command::TapQuiet(value) => value << 2,
            Command::TapShock(value) => value,
            Command::TapMode(TapMode::Single) => 0 << 7,
            Command::TapMode(TapMode::SingleAndDouble) => 1 << 7,
            Command::InterruptEnable(en) => (en as u8) << 7,
            Command::MapInterrupt(_, InterruptSource::SingleTap, en) => (en as u8) << 6,
            Command::MapInterrupt(_, InterruptSource::DoubleTap, en) => (en as u8) << 3,
        }
    }
    pub fn mask(&self) -> u8 {
        match *self {
            Command::SwReset => 0x01,
            Command::SetDataRate(_) => 0xF0,
            Command::SetFullScale(_) => 0b0000_1100,
            Command::TapEnable(_, _, _) => 0b0000_1110,
            Command::TapThreshold(_, _) => 0b0001_1111,
            Command::TapDuration(_) => 0xF0,
            Command::TapQuiet(_) => 0x0C,
            Command::TapShock(_) => 0x03,
            Command::TapMode(_) => 0x80,
            Command::InterruptEnable(_) => 0x80,
            Command::MapInterrupt(_, InterruptSource::SingleTap, _) => 0b0100_0000,
            Command::MapInterrupt(_, InterruptSource::DoubleTap, _) => 0b0000_1000,
        }
    }
}
