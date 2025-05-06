//! MX robotis register (protocol v2)
//!
//! Despite some minor differences among MX variants, it should work for
//! * MX-28
//! * MX-64
//! * MX-106
//!
//! See <https://emanual.robotis.com/docs/en/dxl/mx/mx-28/> for example.

use crate::device::*;

reg_read_only!(model_number, 0, u16);
reg_read_only!(firmware_version, 6, u8);

// unit is 1ms
reg_read_only!(realtime_tick, 120, u16);
reg_read_only!(moving, 122, u8);
reg_read_only!(moving_status, 123, u8);
// unit is about about 0.113 [%]
reg_read_only!(present_pwm, 124, u16);
// unit is 0.1%, range from -1000 (CW) to 1000 (CCW)
reg_read_only!(present_load, 126, i16);
// unit is 0.229 rpm
reg_read_only!(present_velocity, 128, u32);
// TODO: double check if position shoul dbe i32
reg_read_only!(present_position, 132, i32);
reg_read_only!(velocity_trajectory, 136, u32);
reg_read_only!(position_trajectory, 140, u32);
reg_read_only!(present_input_voltage, 144, u16);
reg_read_only!(present_temperature, 146, u8);


reg_read_write!(id, 7, u8);
reg_read_write!(baudrate, 8, u8);
reg_read_write!(return_delay_time, 9, u8);
reg_read_write!(drive_mode, 10, u8);
reg_read_write!(operating_mode, 11, u8);
reg_read_write!(secondary_id, 12, u8);
reg_read_write!(protocol_type, 13, u8);
// 	Home Position Offset
reg_read_write!(homing_offset, 20, i32);
// threshold unit is about 0.229 rpm
reg_read_write!(moving_threshold, 24, u32);
// temperature unit is about 1°
reg_read_write!(temperature_limit, 31, u8);

// voltage unit is about 0.1V
reg_read_write!(max_voltage_limit, 32, u16);
reg_read_write!(min_voltage_limit, 34, u16);

// pwm unit is about 0.113 %
reg_read_write!(pwm_limit, 36, u16);
// current limit unit is about 3.36mA
reg_read_write!(current_limit, 38, u16);
// acceleration unit is 214.577 Rev/min2
reg_read_write!(acceleration_limit, 40, u32);
//  velocity unit is about 0.229rpm
reg_read_write!(velocity_limit, 44, u32);
// position limit unit is 0.088 [°]
reg_read_write!(max_position_limit, 48, u32);
// position limit unit is 0.088 [°]
reg_read_write!(min_position_limit, 52, u32);

reg_read_write!(shutdown, 63, u8);

// RAM area
reg_read_write!(torque_enable, 64, u8);
reg_read_write!(led, 65, u8);

// TODO: status return level
// TODO: Registered Instruction
// TODO: 	Hardware Error Status

reg_read_write!(velocity_i_gain, 76, u16);
reg_read_write!(velocity_p_gain, 78, u16);
reg_read_write!(position_d_gain, 80, u16);
reg_read_write!(position_i_gain, 82, u16);
reg_read_write!(position_p_gain, 84, u16);
// unit is 0.229 rpm, value range: -Velocity Limit(44) ~ Velocity Limit(44)
reg_read_write!(goal_velocity, 104, i32);
// In Velocity-based profile, unit is 214.577 [rev/min2], range from 0 ~ 32767
// In Time-based profile, unit is 1ms, range from 0 ~ 32767
reg_read_write!(profile_acceleration, 108, u32);
//  In Velocity-based profile, unit is 0.229 [rev/min], range from 0 ~ 32767
// In Time-based profile,  unit is 1ms, range from 0 ~ 32767
reg_read_write!(profile_velocity, 112, u32);
// From the front view of DYNAMIXEL, CCW is an increasing direction, whereas CW is a decreasing direction. 
// The way of reaching the Goal Position(116) can differ by the Profile provided by DYNAMIXEL
// In Position Control Mode, values are between	Min Position Limit(52) ~ Max Position Limit(48), representing Initial Value : 0 ~ 4,095
// In Extended Position Control Mode, values are between -1,048,575 ~ 1,048,575, representing -256[rev] ~ 256[rev]
reg_read_write!(goal_position, 116, i32);

/// Sync read present_position, present_speed and present_load in one message
///
/// reg_read_only!(present_position_speed_load, 36, (i16, u16, u16))
pub fn sync_read_present_position_speed_load(
    io: &DynamixelSerialIO,
    serial_port: &mut dyn serialport::SerialPort,
    ids: &[u8],
) -> Result<Vec<(i16, u16, u16)>> {
    let val = io.sync_read(serial_port, ids, 36, 2 + 2 + 2)?;
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

/// Unit conversion for MX motors
pub mod conv {
    use std::f64::consts::PI;

    /// Dynamixel angular position to radians
    ///
    /// Works in joint and multi-turn mode
    pub fn dxl_pos_to_radians(pos: i16) -> f64 {
        (2.0 * PI * (pos as f64) / 4096.0) - PI
    }
    /// Radians to dynamixel angular position
    ///
    /// Works in joint and multi-turn mode
    pub fn radians_to_dxl_pos(rads: f64) -> i16 {
        (4096.0 * (PI + rads) / (2.0 * PI)) as i16
    }

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

    use super::conv::*;

    #[test]
    fn position_conversions() {
        assert_eq!(radians_to_dxl_pos(0.0), 2048);
        assert_eq!(radians_to_dxl_pos(-PI / 2.0), 1024);
        assert_eq!(radians_to_dxl_pos(PI / 2.0), 3072);
        assert_eq!(dxl_pos_to_radians(2048), 0.0);
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
