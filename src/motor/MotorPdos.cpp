#include "motor/Motor.hpp"

void gravity::MotorBase::config_pdo_list()
{
    // rx
    rx_pdos.push_back(control_word.get());
    rx_pdos.push_back(target_position.get());
    rx_pdos.push_back(position_offset.get());
    rx_pdos.push_back(velocity_offset.get());
    rx_pdos.push_back(target_torque.get());
    rx_pdos.push_back(torque_slope.get());
    rx_pdos.push_back(mode_of_operation.get());

    // tx
    tx_pdos.push_back(error_code.get());
    tx_pdos.push_back(status_word.get());
    tx_pdos.push_back(position_actual_value.get());
    tx_pdos.push_back(velocity_actual_value.get());
    tx_pdos.push_back(torque_actual_value.get());
    tx_pdos.push_back(following_error_actual_value.get());
    tx_pdos.push_back(mode_of_operation_display.get());
}

std::vector<uint32_t> gravity::MotorBase::rx_pdo_entries() const
{
    std::vector<uint32_t> keys;
    keys.reserve(rx_pdos.size());
    for (const auto *e : rx_pdos)
        keys.push_back(e->getEntryKey());
    return keys;
}

std::vector<uint32_t> gravity::MotorBase::tx_pdo_entries() const
{
    std::vector<uint32_t> keys;
    keys.reserve(tx_pdos.size());
    for (const auto *e : tx_pdos)
        keys.push_back(e->getEntryKey());
    return keys;
}

std::vector<ec_pdo_entry_reg_t> gravity::MotorBase::get_domain_regs()
{
    try
    {
        size_t total_pdo_bytes = 0;
        std::vector<ec_pdo_entry_reg_t> regs;
        regs.reserve(rx_pdos.size() + tx_pdos.size());
        auto build_entries = [&](const std::vector<DictionaryEntity *> &pdos)
        {
            for (const auto &obj : pdos)
            {
                regs.push_back({alias,
                                position,
                                vendor_id,
                                product_code,
                                obj->getIndex(),
                                obj->getSubindex(),
                                &obj->getOffset()});
                total_pdo_bytes += obj->get_size_bytes();
            }
        };
        _log->info("Total pdo bytes: {}", total_pdo_bytes);

        build_entries(rx_pdos);
        build_entries(tx_pdos);
        return regs;
    }
    catch (const std::exception &e)
    {
        throw std::runtime_error("Failed to get Domain Registration -> ");
    }
}
