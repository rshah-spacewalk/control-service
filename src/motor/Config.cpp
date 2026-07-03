#include "motor/Config.hpp"

void gravity::MotorConfig::register_pdos()
{
    try
    {
        std::vector<ec_pdo_entry_reg_t> all_regs;
        for (MotorBase &motor : motors)
        {
            if (std::find(enabled.begin(), enabled.end(), motor.position) != enabled.end())
            {
                auto regs = motor.get_domain_regs();
                all_regs.insert(all_regs.end(), regs.begin(), regs.end());
            }
        }

        all_regs.push_back({}); // End marker
        int resp = ecrt_domain_reg_pdo_entry_list(master->ec_domain_ptr, all_regs.data());
        if (resp)
        {
            auto msg = (fmt::format("Motor PDO Registration failed: {}", resp));
            throw std::runtime_error(msg);
        }
        else
        {
            _log->info("Successfully registered PDOs");
        }
    }
    catch (const std::exception &e)
    {
        auto msg = (fmt::format("Motor PDO Registration failed: {}", e.what()));
        throw std::runtime_error(msg);
    }
}

void gravity::MotorConfig::apply_configs()
{
    try
    {
        for (const MotorBase &motor : motors)
        {
            if (std::find(enabled.begin(), enabled.end(), motor.position) != enabled.end())
            {
                const uint32_t feed = motor.dictionary_map.feed->read_sdo();
                auto allowd_max_pos = config::kepler::rad_to_gear_pulse(config::kepler::MAX_JOINT_LIMITS[motor.position], motor.position);
                auto allowd_min_pos = config::kepler::rad_to_gear_pulse(config::kepler::MIN_JOINT_LIMITS[motor.position], motor.position);

                motor.dictionary_map.max_position_limit->write_sdo(allowd_max_pos);
                motor.dictionary_map.min_position_limit->write_sdo(allowd_min_pos);
                motor.dictionary_map.polarity->write_sdo(config::kepler::MOTOR_DIR[motor.position]);

                const uint32_t encoder_mode = sdo_read<uint32_t>(motor.position, 0x2015, 0x0);
                const uint32_t max_acl = motor.dictionary_map.max_acceleration->read_sdo();
                const uint32_t max_dcl = motor.dictionary_map.max_deceleration->read_sdo();
                const uint32_t max_speed = motor.dictionary_map.max_motor_speed->read_sdo();
                const int32_t max_pos = motor.dictionary_map.max_position_limit->read_sdo();
                const int32_t min_pos = motor.dictionary_map.min_position_limit->read_sdo();
                const uint8_t direction = motor.dictionary_map.polarity->read_sdo();
                const int32_t current_pos = motor.dictionary_map.position_actual_value->read_sdo();

                _log->info(
                    "------------------------------------------------------------\n"
                    "Motor Position      : {}\n"
                    "Encoder Mode        : {}\n"
                    "Feed Constant       : {}\n"
                    "Max Acceleration    : {}\n"
                    "Max Deceleration    : {}\n"
                    "Max Motor Speed     : {}\n"
                    "Max Position Limit  : {}\n"
                    "Min Position Limit  : {}\n"
                    "Direction           : {}\n"
                    "Position            : {}\n"
                    "------------------------------------------------------------\n",
                    motor.position, encoder_mode, feed, max_acl, max_dcl, max_speed, max_pos, min_pos, direction, current_pos);
            }
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
        if (std::find(enabled.begin(), enabled.end(), motor.position) != enabled.end())
        {
            uint16_t error_code = motor.dictionary_map.error_code->read_sdo();
            if (error_code)
            {
                _log->error("Motor {} has error : 0x{:x}", motor.position, error_code);
                if (error_code == 0x821b)
                {
                    _log->error("Motor: {} -> SyncManager2 watchdog timer timeout", motor.position);
                    motor.dictionary_map.control_word->write_sdo(static_cast<uint16_t>(SLAVE_CONTROL_WORD::FAULT_TO_FAULTLESS));
                    gravity::Clock::fromSeconds(1).sleepFor();
                }

                if (error_code == 0x8612)
                {
                    _log->error("Motor: {} -> Bus input signal dithering", motor.position);
                    motor.dictionary_map.control_word->write_sdo(static_cast<uint16_t>(SLAVE_CONTROL_WORD::FAULT_TO_FAULTLESS));
                    gravity::Clock::fromSeconds(1).sleepFor();
                }
                else
                {
                    motor.dictionary_map.control_word->write_sdo(static_cast<uint16_t>(SLAVE_CONTROL_WORD::FAULT_TO_FAULTLESS));
                    gravity::Clock::fromSeconds(1).sleepFor();
                }
            }
        }
    }
}

void gravity::MotorConfig::reset_encoder()
{
    for (const MotorBase &motor : motors)
    {
        if (std::find(enabled.begin(), enabled.end(), motor.position) != enabled.end())
        {
            sdo_write<uint32_t>(motor.position, 0x2015, 0x0, 0x9);
        }
    }
    _log->info("Encoder position reset done!");
}
