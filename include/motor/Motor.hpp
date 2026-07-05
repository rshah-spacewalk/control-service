#pragma once

#include "ethercat/Master.hpp"
#include "ethercat/Slave.hpp"
#include "ethercat/DictionaryEntity.hpp"

namespace gravity
{
    class MotorBase : public SlaveBase
    {
    private:
        int setup_delay_ms = 500;
        std::vector<DictionaryEntity *> rx_pdos;
        std::vector<DictionaryEntity *> tx_pdos;
        std::shared_ptr<spdlog::logger> _log = make_class_logger("Motor");

    public:
        MotorBase(
            const uint16_t alias,
            const uint16_t position,
            const uint32_t vendor_id,
            const uint32_t product_code)
            : SlaveBase(alias, position, vendor_id, product_code)
        {
            _log->info(str());
        }

        MotorBase(ec_master_t *ec_master_ptr, int position)
            : SlaveBase(ec_master_ptr, position) {}

        void enable_cycle();
        void disable_cycle();
        void quick_stop();
        void quick_stop_to_running();
        void cycle(uint8_t *domain_pdm);

        void config_pdo_list();
        void build_data_objects();
        std::vector<uint32_t> rx_pdo_entries() const;
        std::vector<uint32_t> tx_pdo_entries() const;
        std::vector<ec_pdo_entry_reg_t> get_domain_regs();

        // dictionary Map

        // rx
        std::unique_ptr<DataObject<uint16_t>> control_word = nullptr;    // 1
        std::unique_ptr<DataObject<int32_t>> target_position = nullptr;  // 2
        std::unique_ptr<DataObject<int32_t>> position_offset = nullptr;  // 3
        std::unique_ptr<DataObject<int32_t>> velocity_offset = nullptr;  // 4
        std::unique_ptr<DataObject<int16_t>> target_torque = nullptr;    // 5
        std::unique_ptr<DataObject<uint32_t>> torque_slope = nullptr;    // 6
        std::unique_ptr<DataObject<int8_t>> mode_of_operation = nullptr; // 7

        // tx
        std::unique_ptr<DataObject<uint16_t>> error_code = nullptr;                  // 8
        std::unique_ptr<DataObject<uint16_t>> status_word = nullptr;                 // 9
        std::unique_ptr<DataObject<int32_t>> position_actual_value = nullptr;        // 10
        std::unique_ptr<DataObject<int32_t>> velocity_actual_value = nullptr;        // 11
        std::unique_ptr<DataObject<int16_t>> torque_actual_value = nullptr;          // 12
        std::unique_ptr<DataObject<int32_t>> following_error_actual_value = nullptr; // 13
        std::unique_ptr<DataObject<int8_t>> mode_of_operation_display = nullptr;     // 14

        // configs
        std::unique_ptr<DataObject<uint32_t>> feed = nullptr;                          // 15
        std::unique_ptr<DataObject<uint32_t>> encoder_increments = nullptr;            // 16
        std::unique_ptr<DataObject<int32_t>> position_actual_internal_value = nullptr; // 17

        std::unique_ptr<DataObject<uint32_t>> max_motor_speed = nullptr;      // 18
        std::unique_ptr<DataObject<uint32_t>> max_profile_velocity = nullptr; // 19
        std::unique_ptr<DataObject<uint32_t>> max_acceleration = nullptr;     // 20
        std::unique_ptr<DataObject<uint32_t>> max_deceleration = nullptr;     // 21
        std::unique_ptr<DataObject<uint16_t>> max_torque = nullptr;           // 22

        std::unique_ptr<DataObject<int16_t>> quick_stop_option_code = nullptr;   // 23
        std::unique_ptr<DataObject<uint32_t>> quick_stop_deceleration = nullptr; // 24

        std::unique_ptr<DataObject<int32_t>> min_position_limit = nullptr; // 25
        std::unique_ptr<DataObject<int32_t>> max_position_limit = nullptr; // 26
        std::unique_ptr<DataObject<uint8_t>> polarity = nullptr;           // 27
        std::unique_ptr<DataObject<uint32_t>> digital_inputs = nullptr;    // 28
    };

} // namespace gravity::mover