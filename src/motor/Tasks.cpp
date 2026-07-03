
#include "ethercat/EthEnums.hpp"
#include "motor/Motor.hpp"
#include <gravity/Clock.hpp>

void gravity::MotorBase::enable_cycle()
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
            dictionary_map.control_word->write_pdo(value);
            gravity::Clock::fromMilliseconds(setup_delay_ms).sleepFor();
            dictionary_map.status_word->read_pdo();
        }
        _log->info("Motor {} set to RUNNING", position);
    }
    catch (const std::exception &e)
    {
        _log->error("enable_cycle failed: {}", e.what());
        throw;
    }
}

void gravity::MotorBase::disable_cycle()
{
    try
    {
        for (const auto &arg : {
                 SLAVE_CONTROL_WORD::WAITING_ENABLE,
                 SLAVE_CONTROL_WORD::READY,
                 SLAVE_CONTROL_WORD::FAULTLESS})
        {
            uint16_t value = static_cast<uint16_t>(arg);
            dictionary_map.control_word->write_pdo(value);
            gravity::Clock::fromMilliseconds(setup_delay_ms).sleepFor();
            dictionary_map.status_word->read_pdo();
        }
        _log->info("Motor {} set to FAULTLESS", position);
    }
    catch (const std::exception &e)
    {
        _log->error("enable_cycle failed: {}", e.what());
        throw;
    }
}

void gravity::MotorBase::quick_stop()
{
    try
    {
        uint16_t value = static_cast<uint16_t>(SLAVE_CONTROL_WORD::QUICK_STOP);
        dictionary_map.control_word->write_pdo(value);
        _log->warn("Quick stop triggered for motor {}", position);
    }
    catch (const std::exception &e)
    {
        _log->error("Quick stop failed for motor {} -> {}", position, e.what());
    }
}

void gravity::MotorBase::quick_stop_to_running()
{
    try
    {
        for (const auto &arg : {
                 SLAVE_CONTROL_WORD::READY,
                 SLAVE_CONTROL_WORD::WAITING_ENABLE,
                 SLAVE_CONTROL_WORD::RUNNING})
        {
            uint16_t value = static_cast<uint16_t>(arg);
            dictionary_map.control_word->write_pdo(value);
            gravity::Clock::fromMilliseconds(setup_delay_ms).sleepFor();
        }
        _log->info("Quick stop -> Running for motor {}", position);
    }
    catch (const std::exception &e)
    {
        _log->error("Quick stop -> Running failed for motor {} -> {}", position, e.what());
    }
}

// void gravity::MotorBase::cycle(uint8_t *domain_pdm)
// {
//     // rx
//     EC_WRITE_U16(domain_pdm + dictionary_map.control_word->getOffset(), dictionary_map.control_word->getValue());
//     EC_WRITE_S32(domain_pdm + dictionary_map.target_position->getOffset(), dictionary_map.target_position->getValue());
//     EC_WRITE_S32(domain_pdm + dictionary_map.position_offset->getOffset(), dictionary_map.position_offset->getValue());
//     EC_WRITE_S32(domain_pdm + dictionary_map.velocity_offset->getOffset(), dictionary_map.velocity_offset->getValue());
//     EC_WRITE_S16(domain_pdm + dictionary_map.target_torque->getOffset(), dictionary_map.target_torque->getValue());
//     EC_WRITE_U32(domain_pdm + dictionary_map.torque_slope->getOffset(), dictionary_map.torque_slope->getValue());
//     EC_WRITE_S8(domain_pdm + dictionary_map.mode_of_operation->getOffset(), dictionary_map.mode_of_operation->getValue());
//     // tx
//     dictionary_map.error_code->setValue(EC_READ_U16(domain_pdm + dictionary_map.error_code->getOffset()));
//     dictionary_map.status_word->setValue(EC_READ_U16(domain_pdm + dictionary_map.status_word->getOffset()));
//     dictionary_map.position_actual_value->setValue(EC_READ_S32(domain_pdm + dictionary_map.position_actual_value->getOffset()));
//     dictionary_map.velocity_actual_value->setValue(EC_READ_S32(domain_pdm + dictionary_map.velocity_actual_value->getOffset()));
//     dictionary_map.torque_actual_value->setValue(EC_READ_S16(domain_pdm + dictionary_map.torque_actual_value->getOffset()));
//     dictionary_map.following_error_actual_value->setValue(EC_READ_S32(domain_pdm + dictionary_map.following_error_actual_value->getOffset()));
//     dictionary_map.mode_of_operation_display->setValue(EC_READ_S8(domain_pdm + dictionary_map.mode_of_operation_display->getOffset()));
// }

void gravity::MotorBase::cycle(uint8_t *domain_pdm)
{
    // rx
    dictionary_map.control_word->write_to_domain(domain_pdm);
    dictionary_map.target_position->write_to_domain(domain_pdm);
    dictionary_map.position_offset->write_to_domain(domain_pdm);
    dictionary_map.velocity_offset->write_to_domain(domain_pdm);
    dictionary_map.target_torque->write_to_domain(domain_pdm);
    dictionary_map.torque_slope->write_to_domain(domain_pdm);
    dictionary_map.mode_of_operation->write_to_domain(domain_pdm);

    // tx
    dictionary_map.error_code->read_from_domain(domain_pdm);
    dictionary_map.status_word->read_from_domain(domain_pdm);
    dictionary_map.position_actual_value->read_from_domain(domain_pdm);
    dictionary_map.velocity_actual_value->read_from_domain(domain_pdm);
    dictionary_map.torque_actual_value->read_from_domain(domain_pdm);
    dictionary_map.following_error_actual_value->read_from_domain(domain_pdm);
    dictionary_map.mode_of_operation_display->read_from_domain(domain_pdm);
}
