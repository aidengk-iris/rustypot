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
    reg: (firmware_version, r, 6, u8, None),
    // unit is 1ms
    reg: (realtime_tick, r, 120, u16, None),
    reg: (moving, r, 122, u8, None),
    reg: (moving_status, r, 123, u8, None),
    // unit is about about 0.113 [%]
    reg: (present_pwm, r, 124, u16, None),
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
    reg: (id, rw, 7, u8),
    reg: (baudrate, rw, 8, u8),
    reg: (return_delay_time, rw, 9, u8),
    reg: (drive_mode, rw, 10, u8),
    reg: (operating_mode, rw, 11, u8),
    reg: (secondary_id, rw, 12, u8),
    reg: (protocol_type, rw, 13, u8),
    // 	Home Position Offset
    reg: (homing_offset, rw, 20, i32),
    // threshold unit is about 0.229 rpm
    reg: (moving_threshold, rw, 24, u32),
    //temperature unit is about 1°
    reg: (temperature_limit, rw, 31, u8),

    // voltage unit is about 0.1V
    reg: (max_voltage_limit, rw, 32, u16),
    reg: (min_voltage_limit, rw, 34, u16),

    // pwm unit is about 0.113 %
    reg: (pwm_limit, rw, 36, u16),
    // current limit unit is about 3.36mA
    reg: (current_limit, rw, 38, u16),
    // acceleration unit is 214.577 Rev/min2
    reg: (acceleration_limit, rw, 40, u32),
    //  velocity unit is about 0.229rpm
    reg: (velocity_limit, rw, 44, u32),
    // position limit unit is 0.088 [°]
    reg: (max_position_limit, rw, 48, u32),
    // position limit unit is 0.088 [°]
    reg: (min_position_limit, rw, 52, u32),

    reg: (shutdown, rw, 63, u8),

    // RAM area
    reg: (torque_enable, rw, 64, u8),
    reg: (led, rw, 65, u8),

    // TODO: status return level
    // TODO: Registered Instruction
    // TODO: 	Hardware Error Status

    reg: (velocity_i_gain, rw, 76, u16),
    reg: (velocity_p_gain, rw, 78, u16),
    reg: (position_d_gain, rw, 80, u16),
    reg: (position_i_gain, rw, 82, u16),
    reg: (position_p_gain, rw, 84, u16),
    //  unit is 0.229 rpm, value range: -Velocity Limit(44) ~ Velocity Limit(44)
    reg: (goal_velocity, rw, 104, i32),
    // In Velocity-based profile, unit is 214.577 [rev/min2], range from 0 ~ 32767
    // In Time-based profile, unit is 1ms, range from 0 ~ 32767
    reg: (profile_acceleration, rw, 108, u32),
    //  In Velocity-based profile, unit is 0.229 [rev/min], range from 0 ~ 32767
    // In Time-based profile,  unit is 1ms, range from 0 ~ 32767
    reg: (profile_velocity, rw, 112, u32),
    // From the front view of DYNAMIXEL, CCW is an increasing direction, whereas CW is a decreasing direction. 
    // The way of reaching the Goal Position(116) can differ by the Profile provided by DYNAMIXEL
    // In Position Control Mode, values are between	Min Position Limit(52) ~ Max Position Limit(48), representing Initial Value : 0 ~ 4,095
    // In Extended Position Control Mode, values are between -1,048,575 ~ 1,048,575, representing -256[rev] ~ 256[rev]
    reg: (goal_position, rw, 116, i32),
);
