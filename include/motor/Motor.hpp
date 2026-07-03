#pragma once

#include "ethercat/DictionaryMap.hpp"
#include "ethercat/Master.hpp"
#include "ethercat/Slave.hpp"

namespace gravity
{
    class MotorBase : public SlaveBase
    {
    private:
        std::shared_ptr<spdlog::logger> _log = make_class_logger("Motor");
        int setup_delay_ms = 500;
        std::vector<uint32_t> build_pdo_entries(const std::vector<std::string> &names) const;

    public:
        DictionaryMap dictionary_map;

        std::vector<std::string> rx_pdo_list =
            {
                "control_word",
                "target_position",
                "position_offset",
                "velocity_offset",
                "target_torque",
                "torque_slope",
                "mode_of_operation"};

        std::vector<std::string> tx_pdo_list =
            {
                "error_code",
                "status_word",
                "position_actual_value",
                "velocity_actual_value",
                "torque_actual_value",
                "following_error_actual_value",
                "mode_of_operation_display"};

        MotorBase(
            const uint16_t alias,
            const uint16_t position,
            const uint32_t vendor_id,
            const uint32_t product_code) : SlaveBase(alias, position, vendor_id, product_code), dictionary_map(position)
        {
            _log->info(str());
        }

        MotorBase(ec_master_t *ec_master_ptr, int position) : SlaveBase(ec_master_ptr, position), dictionary_map(position) {}

        std::vector<ec_pdo_entry_reg_t> get_domain_regs();
        std::vector<uint32_t> rx_pdo_entries() const;
        std::vector<uint32_t> tx_pdo_entries() const;

        void enable_cycle();
        void disable_cycle();
        void quick_stop();
        void quick_stop_to_running();
        void cycle(uint8_t *domain_pdm);
    };

} // namespace gravity::mover