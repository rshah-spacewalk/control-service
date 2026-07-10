#include "motor/Config.hpp"

void gravity::MotorConfig::read_configs()
{
    for (const auto &motor : motors)
    {
        try
        {
            const auto current_pos = motor->position_actual_value->read_sdo();
            const auto current_pos_rad = config::gear_pulse_to_rad<>(current_pos, motor->joint);

            const auto max_position_limit = motor->max_position_limit->read_sdo();
            const auto max_pos_rad = config::gear_pulse_to_rad(max_position_limit, motor->joint);

            const auto min_position_limit = motor->min_position_limit->read_sdo();
            const auto min_pos_rad = config::gear_pulse_to_rad(min_position_limit, motor->joint);

            const auto max_motor_speed = motor->max_motor_speed->read_sdo();
            const auto max_speed_rad = config::gear_pulse_to_rad(max_motor_speed, motor->joint);

            const auto max_acceleration = motor->max_acceleration->read_sdo();
            const auto max_acl_rad = config::gear_pulse_to_rad(max_acceleration, motor->joint);

            const auto max_deceleration = motor->max_deceleration->read_sdo();
            const auto max_dcl_rad = config::gear_pulse_to_rad(max_deceleration, motor->joint);

            const auto encoder_mode = motor->absolute_encoder_setting->read_sdo();
            const auto feed_constant = motor->feed->read_sdo();

            const auto direction = motor->polarity->read_sdo();

            _log->info(
                "------------------------------------------------------------\n"
                "Motor Joint         : {}\n"
                "Motor Position      : {}\n"
                "Motor Direction     : {}\n"
                "Encoder Mode        : {}\n"
                "Feed Constant       : {}\n"
                "Max Acceleration    : {} pulse ({:.4f} rad)\n"
                "Max Deceleration    : {} pulse ({:.4f} rad)\n"
                "Max Motor Speed     : {} pulse ({:.4f} rad)\n"
                "Max Position Limit  : {} pulse ({:.4f} rad)\n"
                "Min Position Limit  : {} pulse ({:.4f} rad)\n"
                "Current Position    : {} pulse ({:.4f} rad)\n"
                "------------------------------------------------------------",
                motor->joint,
                motor->position,
                direction,
                encoder_mode,
                feed_constant,
                max_acceleration, max_acl_rad,
                max_deceleration, max_dcl_rad,
                max_motor_speed, max_speed_rad,
                max_position_limit, max_pos_rad,
                min_position_limit, min_pos_rad,

                current_pos, current_pos_rad);
        }
        catch (const std::exception &e)
        {
            _log->error("Failed to read config for motor joint {}: {}", motor->joint, e.what());
        }
    }
}