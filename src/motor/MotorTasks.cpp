
#include "ethercat/EthEnums.hpp"
#include "ethercat/EthStatus.hpp"
#include "motor/Motor.hpp"
#include <gravity/Clock.hpp>
#include <utility>
#include <array>

void gravity::MotorBase::control_cmd(const gravity::SLAVE_CONTROL_WORD cw)
{
    if (last_cmd.has_value() && last_cmd.value() == cw)
    {
        return;
    }
    uint16_t value = static_cast<uint16_t>(cw);
    control_word->write_pdo(value);
    last_cmd = cw;
    _log->info("Control [{}] -> [{}] [{}]", position, util::tf::enum_str(cw), hex_str(value));
}

void gravity::MotorBase::enable()
{
    _log->info("Motor [{}] enabling", position);
    status_word_entity status = decode_status_word(status_word->read_pdo());
    gravity::Clock start = gravity::Clock::now();

    while (!status.servo_running)
    {
        if (status.main_circuit_power && status.servo_ready && status.start)
        {
            control_cmd(SLAVE_CONTROL_WORD::RUNNING); // 3. push to running
        }
        else if (status.main_circuit_power && status.servo_ready)
        {
            control_cmd(SLAVE_CONTROL_WORD::WAITING_ENABLE); // 2. push to start
        }
        else if (status.main_circuit_power)
        {
            control_cmd(SLAVE_CONTROL_WORD::READY); // 1. push to ready
        }
        else
        {
            control_cmd(SLAVE_CONTROL_WORD::FAULTLESS); // 0. wait for automatic transition
        }

        auto elapsed_sec = (gravity::Clock::now() - start).toSeconds();
        if (elapsed_sec >= 3)
        {
            break;
        }
        else
        {
            gravity::Clock::fromMilliseconds(10).sleepFor();
            status = decode_status_word(status_word->read_pdo());
        }
    }
}

void gravity::MotorBase::disable()
{
    _log->info("Motor [{}] disabling", position);
    status_word_entity status = decode_status_word(status_word->read_pdo());
    gravity::Clock start = gravity::Clock::now();
    while (status.servo_ready)
    {

        if (status.servo_ready && status.start && status.servo_running)
        {
            control_cmd(SLAVE_CONTROL_WORD::WAITING_ENABLE); // RUNNING -> WAITING_ENABLE
        }
        else if (status.servo_ready && status.start)
        {
            control_cmd(SLAVE_CONTROL_WORD::READY); //  WAITING_ENABLE -> READY
        }
        else if (status.servo_ready)
        {
            control_cmd(SLAVE_CONTROL_WORD::FAULTLESS); // RUNNING -> WAITING_ENABLE
        }

        auto elapsed_sec = (gravity::Clock::now() - start).toSeconds();
        if (elapsed_sec >= 3)
        {
            break;
        }
        else
        {
            gravity::Clock::fromMilliseconds(10).sleepFor();
            status = decode_status_word(status_word->read_pdo());
        }
    }
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

void gravity::MotorBase::cycle(uint8_t *domain_pdm, const uint64_t _counter)
{
    for (auto &rx : rx_pdos)
    {
        rx->write_to_domain(domain_pdm, _counter);
    }

    for (auto &tx : tx_pdos)
    {
        tx->read_from_domain(domain_pdm, _counter);
    }
}

bool gravity::MotorBase::reset_error()
{
    _log->info("Motor [{}] resetting error", position);
    auto state = get_master_state();
    if (state.al_states == EC_AL_STATE_OP)
    {
        control_word->write_pdo(static_cast<uint16_t>(SLAVE_CONTROL_WORD::FAULT_TO_FAULTLESS));
        return true;
    }
    else
    {
        return control_word->write_sdo(static_cast<uint16_t>(SLAVE_CONTROL_WORD::FAULT_TO_FAULTLESS)) == 0;
    }
    return false;
}

bool gravity::MotorBase::reset_encoder()
{
    _log->info("Motor [{}] resetting encoder", position);

    auto encoder_value = static_cast<uint32_t>(
        AbsoluteEncoderMode::CLEAR_MULTI_TURN_POSITION_RESET_MULTI_TURN_ALARM_ACTIVATE_MULTI_TURN_ABSOLUTE);

    auto state = get_master_state();

    if (state.al_states != EC_AL_STATE_OP)
    {
        return control_word->write_sdo(encoder_value) == 0;
    }
    return false;
}

// faultless(circuit power), ready(ready), w_enable(start), running(running)
// const std::vector<uint16_t> status_cycle = {0x0670, 0x0631, 0x633, 0x637};
// const std::vector<SLAVE_CONTROL_WORD> startup_cmd = {
//     SLAVE_CONTROL_WORD::FAULTLESS,
//     SLAVE_CONTROL_WORD::READY,
//     SLAVE_CONTROL_WORD::WAITING_ENABLE,
//     SLAVE_CONTROL_WORD::RUNNING};