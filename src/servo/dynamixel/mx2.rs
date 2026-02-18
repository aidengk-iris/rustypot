//! MX robotis register (protocol v2)
//!
//! Despite some minor differences among MX variants, it should work for
//! * MX-28
//! * MX-64
//! * MX-106
//!
//! See <https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/> for example.

use crate::generate_servo;

generate_servo!(
    MX, V2,
    // read only registers
    reg: (model_number, r, 0, u16, None),
    reg: (firmware_version, r, 6, u8),
    // unit is 1ms
    reg: (realtime_tick, r, 120, u16),
    reg: (moving, r, 122, u8),
    reg: (moving_status, r, 123, u8),
    // unit is about about 0.113 [%]
    reg: (present_pwm, r, 124, u16),
    // unit is 0.1%, range from -1000 (CW) to 1000 (CCW)
    reg: (present_load, r, 126, i16),
    // unit is 0.229 rpm
    reg: (present_velocity, r, 128, u32),
    // TODO: double check if position shoul dbe i32
    reg: (present_position, r, 132, i32),
    reg: (velocity_trajectory, r, 136, u32),
    reg: (position_trajectory, r, 140, u32),
    reg: (present_input_voltage, r, 144, u16),
    reg: (present_temperature, r, 146, u8),
    // read and write registers
    reg_read_write!(id, 7, u8),
    reg_read_write!(baudrate, 8, u8),
    reg_read_write!(return_delay_time, 9, u8),
    reg_read_write!(drive_mode, 10, u8),
    reg_read_write!(operating_mode, 11, u8),
    reg_read_write!(secondary_id, 12, u8),
    reg_read_write!(protocol_type, 13, u8),
    // 	Home Position Offset
    reg_read_write!(homing_offset, 20, i32),
    // threshold unit is about 0.229 rpm
    reg_read_write!(moving_threshold, 24, u32),
    //temperature unit is about 1°
    reg_read_write!(temperature_limit, 31, u8),

    // voltage unit is about 0.1V
    reg_read_write!(max_voltage_limit, 32, u16),
    reg_read_write!(min_voltage_limit, 34, u16),

    // pwm unit is about 0.113 %
    reg_read_write!(pwm_limit, 36, u16),
    // current limit unit is about 3.36mA
    reg_read_write!(current_limit, 38, u16),
    // acceleration unit is 214.577 Rev/min2
    reg_read_write!(acceleration_limit, 40, u32),
    //  velocity unit is about 0.229rpm
    reg_read_write!(velocity_limit, 44, u32),
    // position limit unit is 0.088 [°]
    reg_read_write!(max_position_limit, 48, u32),
    // position limit unit is 0.088 [°]
    reg_read_write!(min_position_limit, 52, u32),

    reg_read_write!(shutdown, 63, u8),

    // RAM area
    reg_read_write!(torque_enable, 64, u8),
    reg_read_write!(led, 65, u8),

    // TODO: status return level
    // TODO: Registered Instruction
    // TODO: 	Hardware Error Status

    reg_read_write!(velocity_i_gain, 76, u16),
    reg_read_write!(velocity_p_gain, 78, u16),
    reg_read_write!(position_d_gain, 80, u16),
    reg_read_write!(position_i_gain, 82, u16),
    reg_read_write!(position_p_gain, 84, u16),
    //  unit is 0.229 rpm, value range: -Velocity Limit(44) ~ Velocity Limit(44)
    reg_read_write!(goal_velocity, 104, i32),
    // In Velocity-based profile, unit is 214.577 [rev/min2], range from 0 ~ 32767
    // In Time-based profile, unit is 1ms, range from 0 ~ 32767
    reg_read_write!(profile_acceleration, 108, u32),
    //  In Velocity-based profile, unit is 0.229 [rev/min], range from 0 ~ 32767
    // In Time-based profile,  unit is 1ms, range from 0 ~ 32767
    reg_read_write!(profile_velocity, 112, u32),
    // From the front view of DYNAMIXEL, CCW is an increasing direction, whereas CW is a decreasing direction. 
    // The way of reaching the Goal Position(116) can differ by the Profile provided by DYNAMIXEL
    // In Position Control Mode, values are between	Min Position Limit(52) ~ Max Position Limit(48), representing Initial Value : 0 ~ 4,095
    // In Extended Position Control Mode, values are between -1,048,575 ~ 1,048,575, representing -256[rev] ~ 256[rev]
    reg_read_write!(goal_position, 116, i32),
);
