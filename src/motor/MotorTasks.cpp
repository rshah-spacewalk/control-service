
#include "ethercat/EthEnums.hpp"
#include "motor/Motor.hpp"
#include <gravity/Clock.hpp>

void gravity::MotorBase::enable()
{
    try
    {
        for (const auto &arg : {
                 SLAVE_CONTROL_WORD::FAULTLESS,
                 SLAVE_CONTROL_WORD::READY,
                 SLAVE_CONTROL_WORD::WAITING_ENABLE,
                 SLAVE_CONTROL_WORD::RUNNING})
        {
            uint16_t value = static_cast<uint16_t>(arg);
            control_word->write_pdo(value);
            _log->info("Motor [{}] [{}]", position, util::tf::enum_str(arg));
            gravity::Clock::fromMilliseconds(setup_delay_ms).sleepFor();

            uint16_t sw = status_word->read_pdo();
            _log->info("Status [{}] -> [{:#x}]", position, sw);
        }

        _log->info("Motor [{}] State: {}", position, al_state_str(get_slave_info().al_state));
    }
    catch (const std::exception &e)
    {
        _log->error("enable_cycle failed: {}", e.what());
        throw;
    }
}

void gravity::MotorBase::disable()
{
    for (const auto &arg : {
             SLAVE_CONTROL_WORD::WAITING_ENABLE,
             SLAVE_CONTROL_WORD::READY,
             SLAVE_CONTROL_WORD::FAULTLESS})
    {
        uint16_t value = static_cast<uint16_t>(arg);
        control_word->write_pdo(value);
        gravity::Clock::fromMilliseconds(setup_delay_ms).sleepFor();
        _log->info("Motor [{}] [{}]", position, util::tf::enum_str(arg));
    }
    _log->info("Motor [{}] State: {}", position, al_state_str(get_slave_info().al_state));
}

void gravity::MotorBase::quick_stop()
{

    uint16_t value = static_cast<uint16_t>(SLAVE_CONTROL_WORD::QUICK_STOP);
    control_word->write_pdo(value);
    _log->warn("Quick stop triggered for motor {}", position);
}

void gravity::MotorBase::release_quick_stop()
{
    for (const auto &arg : {
             SLAVE_CONTROL_WORD::READY,
             SLAVE_CONTROL_WORD::WAITING_ENABLE,
             SLAVE_CONTROL_WORD::RUNNING})
    {
        uint16_t value = static_cast<uint16_t>(arg);
        control_word->write_pdo(value);
        gravity::Clock::fromMilliseconds(setup_delay_ms).sleepFor();
    }
    _log->info("Quick stop -> Running for motor {}", position);
}

void gravity::MotorBase::cycle(uint8_t *domain_pdm)
{

    // // rx
    // control_word->write_to_domain(domain_pdm);
    // target_position->write_to_domain(domain_pdm);
    // position_offset->write_to_domain(domain_pdm);
    // velocity_offset->write_to_domain(domain_pdm);
    // target_torque->write_to_domain(domain_pdm);
    // torque_slope->write_to_domain(domain_pdm);
    // mode_of_operation->write_to_domain(domain_pdm);

    for (auto &rx : rx_pdos)
    {
        rx->write_to_domain(domain_pdm);
    }

    for (auto &tx : tx_pdos)
    {
        tx->read_from_domain(domain_pdm);
    }

    // // tx
    // error_code->read_from_domain(domain_pdm);
    // status_word->read_from_domain(domain_pdm);
    // position_actual_value->read_from_domain(domain_pdm);
    // velocity_actual_value->read_from_domain(domain_pdm);
    // torque_actual_value->read_from_domain(domain_pdm);
    // following_error_actual_value->read_from_domain(domain_pdm);
    // mode_of_operation_display->read_from_domain(domain_pdm);
}
