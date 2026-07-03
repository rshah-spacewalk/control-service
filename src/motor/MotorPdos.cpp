#include "motor/Motor.hpp"

std::vector<uint32_t> gravity::MotorBase::build_pdo_entries(const std::vector<std::string> &names) const
{
    std::vector<uint32_t> entries;
    entries.reserve(names.size());

    for (const auto &name : names)
    {
        if (auto it = dictionary_map.get_entries().find(name); it != dictionary_map.get_entries().end())
        {
            if (auto obj = it->second.get())
            {
                entries.push_back(obj->getEntryKey());
            }
            else
            {
                _log->warn("PDO entry '{}' found but is null!", name);
            }
        }
        else
        {
            _log->warn("PDO entry '{}' not found in dictionary!", name);
        }
    }
    return entries;
}

std::vector<ec_pdo_entry_reg_t> gravity::MotorBase::get_domain_regs()
{
    try
    {
        std::vector<ec_pdo_entry_reg_t> regs;
        regs.reserve(rx_pdo_list.size() + tx_pdo_list.size());
        auto build_entries = [&](const std::vector<std::string> &names)
        {
            size_t total_pdo_bytes = 0;
            for (const auto &name : names)
            {
                if (auto obj = dictionary_map.get_entries().at(name).get())
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
                else
                {
                    _log->warn("Dictionary entry '{}' not found!", name);
                }
            }
            // spdlog::info("Total pdo bytes: {}", total_pdo_bytes);
        };

        build_entries(rx_pdo_list);
        build_entries(tx_pdo_list);
        _log->debug("Domain Registration Fetched");
        return regs;
    }
    catch (const std::exception &e)
    {
        throw std::runtime_error("Failed to get Domain Registration -> ");
    }
}

std::vector<uint32_t> gravity::MotorBase::rx_pdo_entries() const
{
    return build_pdo_entries(rx_pdo_list);
}

std::vector<uint32_t> gravity::MotorBase::tx_pdo_entries() const
{
    return build_pdo_entries(tx_pdo_list);
}