#include "motor/Config.hpp"

void gravity::MotorConfig::apply_configs()
{
    try
    {
        for (const auto &motor : motors)
        {

            // 1. set direction
            motor->polarity->write_sdo(config::MOTOR_DIR[motor->joint]);

            // 2. set Direction & max position limits
            auto allowed_max_pos = config::rad_to_gear_pulse<int32_t>(params.max_pos[motor->joint], motor->joint);
            auto allowed_min_pos = config::rad_to_gear_pulse<int32_t>(params.min_pos[motor->joint], motor->joint);

            motor->max_position_limit->write_sdo(allowed_max_pos);
            motor->min_position_limit->write_sdo(allowed_min_pos);

            // 3. set velocity
            auto allowed_max_vel = config::rad_to_gear_pulse<uint32_t>(params.max_vel[motor->joint], motor->joint);
            motor->max_motor_speed->write_sdo(allowed_max_vel);

            // 4. set acceleration
            auto allowed_max_acl = config::rad_to_gear_pulse<uint32_t>(params.max_acl[motor->joint], motor->joint);
            motor->max_acceleration->write_sdo(allowed_max_acl);
            motor->max_deceleration->write_sdo(allowed_max_acl);
        }
    }
    catch (const std::exception &e)
    {
        auto msg = (fmt::format("Motor PDO Registration failed: {}", e.what()));
        throw std::runtime_error(msg);
    }
}

void gravity::MotorConfig::reset_errors()
{
    _log->info("Handling any errors!");
    for (auto &motor : motors)
    {
        uint16_t error_code = motor->error_code->read_sdo();
        if (error_code)
        {
            _log->error("Motor {} has error : 0x{:x}", motor->position, error_code);
            if (error_code == 0x821b)
            {
                _log->error("Motor: {} -> SyncManager2 watchdog timer timeout", motor->position);
                motor->control_word->write_sdo(static_cast<uint16_t>(SLAVE_CONTROL_WORD::FAULT_TO_FAULTLESS));
                gravity::Clock::fromSeconds(1).sleepFor();
            }

            if (error_code == 0x8612)
            {
                _log->error("Motor: {} -> Bus input signal dithering", motor->position);
                motor->control_word->write_sdo(static_cast<uint16_t>(SLAVE_CONTROL_WORD::FAULT_TO_FAULTLESS));
                gravity::Clock::fromSeconds(1).sleepFor();
            }
            else
            {
                motor->control_word->write_sdo(static_cast<uint16_t>(SLAVE_CONTROL_WORD::FAULT_TO_FAULTLESS));
                gravity::Clock::fromSeconds(1).sleepFor();
            }
        }
    }
}

void gravity::MotorConfig::reset_encoder()
{
    for (const auto &motor : motors)
    {
        // Absolute Encoder settings, 9: Clear multiturn position, reset multiturn alarm and activate multiturn absolute function
        // sdo_write<uint32_t>(motor->position, 0x2015, 0x0, 0x9);
        motor->absolute_encoder_setting->write_sdo(static_cast<uint32_t>(
            AbsoluteEncoderMode::CLEAR_MULTI_TURN_POSITION_RESET_MULTI_TURN_ALARM_ACTIVATE_MULTI_TURN_ABSOLUTE));
    }
    _log->info("Encoder position reset done!");
}
