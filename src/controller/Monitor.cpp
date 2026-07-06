#include "controller/Controller.hpp"
#include "ethercat/EthEnums.hpp"

// 603f
bool gravity::Controller::handle_motor_error(const uint16_t &error, const uint16_t &position)
{
    if (error)
    {
        if (motor_error[position] != error)
        {
            _log->error("Error code [{}] -> [{:#x}]", position, error);
            if (master->is_activated())
            {
                quick_stop();
                // disable();
            }

            if (error == 0x8612)
            {
                _log->error("Motor: {} -> Bus input signal dithering", position);
                if (master->is_requested())
                {
                    motors[position]->control_word->write_sdo(static_cast<uint16_t>(SLAVE_CONTROL_WORD::FAULT_TO_FAULTLESS));
                    gravity::Clock::fromSeconds(1).sleepFor();
                    return true;
                }
            }
        }
        motor_error[position] = error;
    }
    return false;
}

bool gravity::Controller::handle_motor_status(const uint16_t &status, const uint16_t &position)
{
    if (motor_status[position] != status)
    {
        status_word_entity sw = decode_status_word(status);
        _log->info("Status [{}] -> [{:#x}]", position, status);

        if (sw.servo_running)
        {
            _log->info("MotorStatus [{}] Servo Running", position);
        }

        if (sw.fault)
        {
            _log->error("MotorStatus [{}] Fault", position);
        }

        // if (!sw.quick_stop)
        // {
        //     _log->warn("MotorStatus [{}] Quick Stop", position);
        // }

        if (sw.arrived_at_position)
        {
            _log->info("MotorStatus [{}] Arrived at Position", position);
        }
    }

    motor_status[position] = status;
    return true;
}
