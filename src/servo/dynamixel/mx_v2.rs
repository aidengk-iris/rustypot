//! MX robotis register (protocol v2)
//!
//! Despite some minor differences among MX variants, it should work for
//! * MX-28
//! * MX-64
//! * MX-106
//!
//! See <https://emanual.robotis.com/docs/en/dxl/mx/mx-28/> for example.

use std::f64::consts::PI;

use crate::{generate_servo, servo::conversion::Conversion};

generate_servo!(
    MX, v2,
    reg: (model_number, r, 0, u16, None),
    reg: (firmware_version, r, 6, u8, None),
    reg: (id, rw, 7, u8, None),
    reg: (baudrate, rw, 8, u8, None),
    reg: (return_delay_time, rw, 9, u8, None),
    reg: (drive_mode, rw, 10, u8, None),

    reg: (operating_mode, rw,11, u8, None),
    reg: (secondary_id, rw,12, u8, None),
    reg: (protocol_type, rw,13, u8, None),

    // 	Home Position Offset
    reg: (homing_offset, rw, 20, i32, None),
    // threshold unit is about 0.229 rpm
    reg: (moving_threshold, rw, 24, u32, None),
    // temperature unit is about 1°
    reg: (temperature_limit, rw, 31, u8, None),
    // voltage unit is about 0.1V
    reg: (max_voltage_limit, rw, 32, u16, None),
    reg: (min_voltage_limit, rw, 34, u16, None),
    // pwm unit is about 0.113 %
    reg: (pwm_limit, rw, 36, u16, None),
    // current limit unit is about 3.36mA
    reg: (current_limit, rw, 38, u16, None),
    // acceleration unit is 214.577 Rev/min2
    reg: (acceleration_limit, rw, 40, u32, None),
    //  velocity unit is about 0.229rpm
    reg: (velocity_limit, rw, 44, u32, None),
    // position limit unit is 0.088 [°]
    reg: (max_position_limit, rw, 48, u32, None),
    // position limit unit is 0.088 [°]
    reg: (min_position_limit, rw,52, u32, None),
    reg: (shutdown, rw, 63, u8, None),

    // RAM area
    reg: (torque_enable, rw, 64, u8, None),
    reg: (led, rw, 65, u8, None),

    // TODO: many ram area registers pending
    reg: (goal_pwm, rw, 100, i16, None),
    reg: (goal_current, rw, 102, i16, None),
    // unit is 0.229 rpm, value range: -Velocity Limit(44) ~ Velocity Limit(44)
    reg: (goal_velocity, rw, 104, i32, None),
    // In Velocity-based profile, unit is 214.577 [rev/min2], range from 0 ~ 32767
    // In Time-based profile, unit is 1ms, range from 0 ~ 32767
    reg: (profile_acceleration, rw, 108, u32, None),

    //  In Velocity-based profile, unit is 0.229 [rev/min], range from 0 ~ 32767
    // In Time-based profile,  unit is 1ms, range from 0 ~ 32767
    reg: (profile_velocity, rw, 112, u32, None),
    // From the front view of DYNAMIXEL, CCW is an increasing direction, whereas CW is a decreasing direction. 
    // The way of reaching the Goal Position(116) can differ by the Profile provided by DYNAMIXEL
    // In Position Control Mode, values are between	Min Position Limit(52) ~ Max Position Limit(48), representing Initial Value : 0 ~ 4,095
    // In Extended Position Control Mode, values are between -1,048,575 ~ 1,048,575, representing -256[rev] ~ 256[rev]
    reg: (goal_position, rw, 116, i32, AnglePosition),
    reg: (realtime_tick, r, 120, u16, None),
    reg: (moving, r, 122, u8, None),
    reg: (moving_status, r, 123, u8, None),
    // unit is about about 0.113 [%]
    reg: (present_pwm, r, 124, u16, None),
    // unit is 0.1%, range from -1000 (CW) to 1000 (CCW)
    reg: (present_current, r, 126, i16, None),
    // unit is 0.229 rpm
    reg: (present_velocity, r, 128, u32, None),
    // TODO: double check if position should be i32
    reg: (present_position, r, 132, i32, None),
    reg: (velocity_trajectory, r, 136, u32, None),
    reg: (position_trajectory, r, 140, u32, None),
    reg: (present_input_voltage, r, 144, u16, None),
    reg: (present_temperature, r, 146, u8, None),

);

/// Sync read present_position, present_speed and present_load in one message
///
/// reg_read_only!(present_position_speed_load, 36, (i16, u16, u16))
pub fn sync_read_present_position_speed_load(
    dph: &crate::DynamixelProtocolHandler,
    serial_port: &mut dyn serialport::SerialPort,
    ids: &[u8],
) -> crate::Result<Vec<(i16, u16, u16)>> {
    let val = dph.sync_read(serial_port, ids, 36, 2 + 2 + 2)?;
    let val = val
        .iter()
        .map(|v| {
            (
                i16::from_le_bytes(v[0..2].try_into().unwrap()),
                u16::from_le_bytes(v[2..4].try_into().unwrap()),
                u16::from_le_bytes(v[4..6].try_into().unwrap()),
            )
        })
        .collect();

    Ok(val)
}

pub struct AnglePosition;

impl Conversion for AnglePosition {
    type RegisterType = i16;
    type UsiType = f64;

    fn from_raw(raw: i16) -> f64 {
        (2.0 * PI * (raw as f64) / 4096.0) - PI
    }

    fn to_raw(value: f64) -> i16 {
        (4096.0 * (PI + value) / (2.0 * PI)) as i16
    }
}

/// Unit conversion for MX motors
pub mod conv {
    /// Dynamixel absolute speed to radians per second
    ///
    /// Works for moving_speed in joint mode for instance
    pub fn dxl_abs_speed_to_rad_per_sec(speed: u16) -> f64 {
        let rpm = speed as f64 * 0.114;
        rpm * 0.10472
    }
    /// Radians per second to dynamixel absolute speed
    ///
    /// Works for moving_speed in joint mode for instance
    pub fn rad_per_sec_to_dxl_abs_speed(speed: f64) -> u16 {
        let rpm = speed / 0.10472;
        (rpm / 0.114) as u16
    }
    /// Dynamixel speed to radians per second
    ///
    /// Works for present_speed for instance
    pub fn dxl_oriented_speed_to_rad_per_sec(speed: u16) -> f64 {
        let cw = (speed >> 11) == 1;

        let rad_per_sec = dxl_abs_speed_to_rad_per_sec(speed % 1024);

        match cw {
            true => rad_per_sec,
            false => -rad_per_sec,
        }
    }
    /// Radians per second to dynamixel speed
    ///
    /// Works for present_speed for instance
    pub fn rad_per_sec_to_dxl_oriented_speed(speed: f64) -> u16 {
        let raw = rad_per_sec_to_dxl_abs_speed(speed.abs());

        match speed < 0.0 {
            true => raw,
            false => raw + 2048,
        }
    }

    /// Dynamixel absolute load to torque percentage
    ///
    /// Works for torque_limit for instance
    pub fn dxl_load_to_abs_torque(load: u16) -> f64 {
        load as f64 / 1023.0 * 100.0
    }
    /// Torque percentage to dynamixel absolute load
    ///
    /// Works for torque_limit for instance
    pub fn torque_to_dxl_abs_load(torque: f64) -> u16 {
        assert!((0.0..=100.0).contains(&torque));

        (torque * 1023.0 / 100.0) as u16
    }
    /// Dynamixel load to torque percentage
    ///
    /// Works for present_torque for instance
    pub fn dxl_load_to_oriented_torque(load: u16) -> f64 {
        let cw = (load >> 10) == 1;

        let torque = dxl_load_to_abs_torque(load % 1024);

        match cw {
            true => torque,
            false => -torque,
        }
    }
    /// Torque percentage to dynamixel load
    pub fn oriented_torque_to_dxl_load(torque: f64) -> u16 {
        let load = torque_to_dxl_abs_load(torque.abs());

        match torque < 0.0 {
            true => load,
            false => load + 1024,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use crate::servo::{conversion::Conversion, dynamixel::mx::AnglePosition};

    use super::conv::*;

    #[test]
    fn position_conversions() {
        assert_eq!(AnglePosition::to_raw(0.0), 2048);
        assert_eq!(AnglePosition::to_raw(-PI / 2.0), 1024);
        assert_eq!(AnglePosition::to_raw(PI / 2.0), 3072);
        assert_eq!(AnglePosition::from_raw(2048), 0.0);
    }

    #[test]
    fn abs_speed_conversions() {
        assert_eq!(rad_per_sec_to_dxl_abs_speed(0.0), 0);
        assert_eq!(rad_per_sec_to_dxl_abs_speed(0.5), 41);
    }

    #[test]
    fn speed_conversions() {
        assert_eq!(dxl_oriented_speed_to_rad_per_sec(300), -3.581424);
        assert_eq!(dxl_oriented_speed_to_rad_per_sec(2048 + 300), 3.581424);

        assert_eq!(rad_per_sec_to_dxl_oriented_speed(-3.581424), 300);
        assert_eq!(rad_per_sec_to_dxl_oriented_speed(3.581424), 2048 + 300);
    }

    #[test]
    fn torque_conversions() {
        assert_eq!(torque_to_dxl_abs_load(0.0), 0);
        assert_eq!(torque_to_dxl_abs_load(50.0), 511);
        assert_eq!(torque_to_dxl_abs_load(100.0), 1023);

        assert_eq!(dxl_load_to_abs_torque(0), 0.0);
        assert!((dxl_load_to_abs_torque(511) - 50.0).abs() < 1e-1);
        assert_eq!(dxl_load_to_abs_torque(1023), 100.0);
    }

    #[test]
    fn load_conversions() {
        assert!((dxl_load_to_oriented_torque(511) + 50.0).abs() < 1e-1);
        assert!((dxl_load_to_oriented_torque(1024 + 512) - 50.0).abs() < 1e-1);

        assert_eq!(oriented_torque_to_dxl_load(-50.0), 511);
        assert_eq!(oriented_torque_to_dxl_load(50.0), 1024 + 511);
    }
}
