#include "controller/Controller.hpp"
#include "ethercat/EthEnums.hpp"
#include "ethercat/EthStatus.hpp"

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
            }

            if (error == 0x8612)
            {
                _log->error("Motor: {} -> Bus input signal dithering", position);
                return motors[position]->reset_error();
            }

            if (error == 0x821b)
            {
                _log->error("Motor [{}] SyncManager2 watchdog timer timeout", position);
                return motors[position]->reset_error();
            }

            if (error == 0x8212)
            {
                _log->error("Motor [{}] No valid output data", position);
                return motors[position]->reset_error();
            }
        }

        motor_error[position] = error;
        return false;
    }
    return true;
}

bool gravity::Controller::handle_motor_status(const uint16_t &status, const uint16_t &position)
{
    if (motor_status[position] != status)
    {
        status_word_entity sw = decode_status_word(status);
        _log->info("Status  [{}] -> [{:#x}]", position, status);
        // _log->info("Status {}", status_word_str(status));
    }
    motor_status[position] = status;
    return true;
}
