#include "motor/Config.hpp"

void gravity::MotorConfig::map_custom_pdo(
    const uint16_t slave,
    const std::vector<uint32_t> &rxpdo_entries,
    const std::vector<uint32_t> &txpdo_entries)
{
    try
    {
        uint16_t rxpdo_index = 0x1600;
        uint16_t txpdo_index = 0x1A00;
        uint8_t rxpdo_count = rxpdo_entries.size();
        uint8_t txpdo_count = txpdo_entries.size();

        // 2. Clear the RxPDO (1C12h) and TxPDO (1C13h)
        sdo_write<uint8_t>(slave, 0x1C12, 0x0, 0);
        sdo_write<uint8_t>(slave, 0x1C13, 0x0, 0);

        // 3.Set subindex 0 of RxPDOs (1600h-1603h) and TxPDOs (1A00h-1A01h) to 0 to invalidate them.
        // each of these are a set of pdos.
        sdo_write<uint32_t>(slave, 0x1600, 0x0, 0);
        sdo_write<uint32_t>(slave, 0x1601, 0x0, 0);
        sdo_write<uint32_t>(slave, 0x1602, 0x0, 0);
        sdo_write<uint32_t>(slave, 0x1603, 0x0, 0);

        sdo_write<uint32_t>(slave, 0x1A00, 0x0, 0);
        sdo_write<uint32_t>(slave, 0x1A01, 0x0, 0);

        // 4. write address to the pdo map : create a new set in new pdos
        spdlog::info("Mapping RX pdos for slave: {}", slave);
        for (uint8_t i = 0; i < rxpdo_count; i++)
        {
            sdo_write<uint32_t>(slave, 0x1600, i + 1, rxpdo_entries[i]);
        }

        spdlog::info("Mapping TX pdos for slave: {}", slave);
        for (uint8_t i = 0; i < txpdo_count; i++)
        {
            sdo_write<uint32_t>(slave, 0x1A00, i + 1, txpdo_entries[i]);
        }
        spdlog::info("PDOs written successfully for slave: {}!", slave);

        // 5. set the number of mapped objects in subindex 0x00 of the respective PDOs.
        sdo_write<uint32_t>(slave, 0x1600, 0x0, rxpdo_count);
        sdo_write<uint32_t>(slave, 0x1A00, 0x0, txpdo_count);

        // 6. Assign RxPDO (1C12-01h ~ 1C12-04h) and TxPDO (1C13-01h, 1C13-02h) indices. # number of pdos in this set
        sdo_write<uint16_t>(slave, 0x1C12, 0x1, rxpdo_index);
        sdo_write<uint16_t>(slave, 0x1C13, 0x1, txpdo_index);

        // 7.  write the number of RxPDOs and TxPDOs in 1C12-00h & 1C13-00h. # provide number of pdo sets
        sdo_write<uint8_t>(slave, 0x1C12, 0x0, 1);
        sdo_write<uint8_t>(slave, 0x1C13, 0x0, 1);

        // 8. save
        sdo_write<uint32_t>(slave, 0x1010, 0x01, 0x65766173);
        gravity::Clock(1, 0).sleepFor();
        spdlog::info("PDO Mapping saved for slave {}", slave);
    }
    catch (const std::exception &e)
    {
        auto msg = fmt::format("Custom PDO Mapping failed: {}", e.what());
        throw std::runtime_error(msg);
    }
}

void gravity::MotorConfig::map_pdos()
{
    try
    {
        for (const MotorBase &motor : motors)
        {
            if (std::find(enabled.begin(), enabled.end(), motor.position) != enabled.end())
            {
                _log->info("Mapping PDOs for Motor {}", motor.position);
                map_custom_pdo(motor.position, motor.rx_pdo_entries(), motor.tx_pdo_entries());
            }
        }
        std::vector<std::string> states = {"SAFEOP", "PREOP", "BOOT", "INIT", "PREOP"};
        for (const auto &state : states)
        {
            master->set_master_state(state);
            gravity::Clock(2, 0).sleepFor();
        }
    }
    catch (const std::exception &e)
    {
        auto msg = (fmt::format("Custom PDO Mapping failed: {}", e.what()));
        throw std::runtime_error(msg);
    }
}