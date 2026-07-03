#pragma once

#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>
#include <cassert>

// local
#include "ethercat/DictionaryEntity.hpp"

namespace gravity
{
    namespace limits
    {
        inline constexpr uint8_t U8_MIN = std::numeric_limits<uint8_t>::min(); // 0 (0x0)
        inline constexpr uint8_t U8_MAX = std::numeric_limits<uint8_t>::max(); // 255 (0xff)

        inline constexpr uint16_t U16_MIN = std::numeric_limits<uint16_t>::min(); //  0 (0x0)
        inline constexpr uint16_t U16_MAX = std::numeric_limits<uint16_t>::max(); // 65535 (0xffff)

        inline constexpr uint32_t U32_MIN = std::numeric_limits<uint32_t>::min(); // 0 (0x0)
        inline constexpr uint32_t U32_MAX = std::numeric_limits<uint32_t>::max(); // 4294967295 (0xffffffff)

        inline constexpr int8_t I8_MIN = std::numeric_limits<int8_t>::min(); // -128 (0xffffff80)
        inline constexpr int8_t I8_MAX = std::numeric_limits<int8_t>::max(); // 127 (0x7f)

        inline constexpr int16_t I16_MIN = std::numeric_limits<int16_t>::min(); //  -32768 (0xffff8000)
        inline constexpr int16_t I16_MAX = std::numeric_limits<int16_t>::max(); // 32767 (0x7fff)

        inline constexpr int32_t I32_MIN = std::numeric_limits<int32_t>::min(); // -2147483648 (0x80000000)
        inline constexpr int32_t I32_MAX = std::numeric_limits<int32_t>::max(); // 2147483647 (0x7fffffff)
    } // namespace limits

    class DictionaryMap
    {
    public:
        // rx
        DictionaryEntry<uint16_t> *control_word = nullptr;    // 1
        DictionaryEntry<int32_t> *target_position = nullptr;  // 2
        DictionaryEntry<int32_t> *position_offset = nullptr;  // 3
        DictionaryEntry<int32_t> *velocity_offset = nullptr;  // 4
        DictionaryEntry<int16_t> *target_torque = nullptr;    // 5
        DictionaryEntry<uint32_t> *torque_slope = nullptr;    // 6
        DictionaryEntry<int8_t> *mode_of_operation = nullptr; // 7

        // tx
        DictionaryEntry<uint16_t> *error_code = nullptr;                  // 8
        DictionaryEntry<uint16_t> *status_word = nullptr;                 // 9
        DictionaryEntry<int32_t> *position_actual_value = nullptr;        // 10
        DictionaryEntry<int32_t> *velocity_actual_value = nullptr;        // 11
        DictionaryEntry<int16_t> *torque_actual_value = nullptr;          // 12
        DictionaryEntry<int32_t> *following_error_actual_value = nullptr; // 13
        DictionaryEntry<int8_t> *mode_of_operation_display = nullptr;     // 14

        // configs
        DictionaryEntry<uint32_t> *feed = nullptr;                          // 15
        DictionaryEntry<uint32_t> *encoder_increments = nullptr;            // 16
        DictionaryEntry<int32_t> *position_actual_internal_value = nullptr; // 17

        DictionaryEntry<uint32_t> *max_motor_speed = nullptr;      // 18
        DictionaryEntry<uint32_t> *max_profile_velocity = nullptr; // 19
        DictionaryEntry<uint32_t> *max_acceleration = nullptr;     // 20
        DictionaryEntry<uint32_t> *max_deceleration = nullptr;     // 21
        DictionaryEntry<uint16_t> *max_torque = nullptr;           // 22

        DictionaryEntry<int16_t> *quick_stop_option_code = nullptr;   // 23
        DictionaryEntry<uint32_t> *quick_stop_deceleration = nullptr; // 24

        DictionaryEntry<int32_t> *min_position_limit = nullptr; // 25
        DictionaryEntry<int32_t> *max_position_limit = nullptr; // 26
        DictionaryEntry<uint8_t> *polarity = nullptr;           // 27
        DictionaryEntry<uint32_t> *digital_inputs = nullptr;    // 28

        DictionaryMap(const uint8_t &position)

        {
            using namespace limits;
            entries.reserve(ENTRY_COUNT);
            // rx
            control_word = add<uint16_t>("control_word", position, 0x6040, 0x0, U16_MIN, U16_MAX, 0, "_", MappingType::RX_PDO);
            target_position = add<int32_t>("target_position", position, 0x607A, 0x0, I32_MIN, I32_MAX, 0, "Uint", MappingType::RX_PDO);
            position_offset = add<int32_t>("position_offset", position, 0x60B0, 0x0, I32_MIN, I32_MAX, 0, "Uint", MappingType::RX_PDO);
            velocity_offset = add<int32_t>("velocity_offset", position, 0x60B1, 0x0, I32_MIN, I32_MAX, 0, "Uint/s", MappingType::RX_PDO);
            target_torque = add<int16_t>("target_torque", position, 0x6071, 0x0, I16_MIN, I16_MAX, 0, "0.1%", MappingType::RX_PDO);
            torque_slope = add<uint32_t>("torque_slope", position, 0x6087, 0x0, 1, I32_MAX, 5'000, "0.1%/s", MappingType::RX_PDO);
            mode_of_operation = add<int8_t>("mode_of_operation", position, 0x6060, 0x0, 1, 11, 8, "_", MappingType::RX_PDO);

            // tx
            error_code = add<uint16_t>("error_code", position, 0x603F, 0x0, U16_MIN, U16_MAX, 0, "_", MappingType::TX_PDO);
            status_word = add<uint16_t>("status_word", position, 0x6041, 0x0, U16_MIN, U16_MAX, 0, "_", MappingType::TX_PDO);
            position_actual_value = add<int32_t>("position_actual_value", position, 0x6064, 0x0, I32_MIN, I32_MAX, 0, "Uint", MappingType::TX_PDO);
            velocity_actual_value = add<int32_t>("velocity_actual_value", position, 0x606C, 0x0, I32_MIN, I32_MAX, 0, "Uint/s", MappingType::TX_PDO);
            torque_actual_value = add<int16_t>("torque_actual_value", position, 0x6077, 0x0, I16_MIN, I16_MAX, 0, "0.1%", MappingType::TX_PDO);
            following_error_actual_value = add<int32_t>("following_error_actual_value", position, 0x60F4, 0x0, I32_MIN, I32_MAX, 0, "Uint", MappingType::TX_PDO);
            mode_of_operation_display = add<int8_t>("mode_of_operation_display", position, 0x6061, 0x0, 1, 11, 0, "_", MappingType::TX_PDO);

            // configs
            feed = add<uint32_t>("feed", position, 0x6092, 0x01, 1, I32_MAX, 10'000, "Uint", MappingType::RX_PDO);                         // pulse per revolution
            encoder_increments = add<uint32_t>("encoder_increments", position, 0x608F, 0x01, 0, I32_MAX, 0, "Pulse", MappingType::TX_PDO); // smallest measurable change in pos
            position_actual_internal_value = add<int32_t>("position_actual_internal_value", position, 0x6063, 0x0, I32_MIN, I32_MAX, 0, "Pulse", MappingType::TX_PDO);

            max_motor_speed = add<uint32_t>("max_motor_speed", position, 0x6080, 0x0, U32_MIN, I32_MAX, 6'000, "r/min", MappingType::RX_PDO);
            max_profile_velocity = add<uint32_t>("max_profile_velocity", position, 0x607F, 0x0, U32_MIN, I32_MAX, I32_MAX, "Uint/s", MappingType::RX_PDO); // 36
            max_acceleration = add<uint32_t>("max_acceleration", position, 0x60C5, 0x0, 1, I32_MAX, 10'00'00'000, "Uint/s^2", MappingType::RX_PDO);
            max_deceleration = add<uint32_t>("max_deceleration", position, 0x60C6, 0x0, 1, I32_MAX, 10'00'00'000, "Uint/s^2", MappingType::RX_PDO);
            max_torque = add<uint16_t>("max_torque", position, 0x6072, 0x0, U16_MIN, U16_MAX, 3000, "0.1%", MappingType::RX_PDO);

            quick_stop_option_code = add<int16_t>("quick_stop_option_code", position, 0x605A, 0x0, 0, 7, 2, "_", MappingType::RX_PDO);
            quick_stop_deceleration = add<uint32_t>("quick_stop_deceleration", position, 0x6085, 0x0, 1, I32_MAX, 10'000'000, "Uint/s^2", MappingType::RX_PDO); // e-stop

            min_position_limit = add<int32_t>("min_position_limit", position, 0x607D, 0x01, I32_MIN, I32_MAX, 0, "Uint", MappingType::RX_PDO);
            max_position_limit = add<int32_t>("max_position_limit", position, 0x607D, 0x02, I32_MIN, I32_MAX, 0, "Uint", MappingType::RX_PDO);
            polarity = add<uint8_t>("polarity", position, 0x607E, 0x0, U8_MIN, U8_MAX, 0x0, "_", MappingType::RX_PDO);
            digital_inputs = add<uint32_t>("digital_inputs", position, 0x60FD, 0x0, U32_MIN, I32_MAX, 0x0, "_", MappingType::TX_PDO);

            assert(entries.size() == ENTRY_COUNT); // check size
        }

        DictionaryMap(const DictionaryMap &) = delete;
        DictionaryMap &operator=(const DictionaryMap &) = delete;
        DictionaryMap(DictionaryMap &&) noexcept = default;
        DictionaryMap &operator=(DictionaryMap &&) noexcept = default;

        const std::unordered_map<std::string, std::unique_ptr<DictionaryEntity>> &get_entries() const noexcept { return entries; }

    private:
        static constexpr size_t ENTRY_COUNT = 28;
        std::unordered_map<std::string, std::unique_ptr<DictionaryEntity>> entries;

        template <typename T>
        DictionaryEntry<T> *add(std::string name,
                                uint8_t position,
                                uint16_t index,
                                uint8_t subindex,
                                T min, T max, T def,
                                std::string uom,
                                MappingType mapping_type)
        {
            auto entry = std::make_unique<DictionaryEntry<T>>(
                name, index, subindex, position,
                min, max, def, std::move(uom), mapping_type);

            auto raw_ptr = entry.get();
            auto [it, inserted] = entries.emplace(std::move(name), std::move(entry));
            if (!inserted)
            {
                throw std::logic_error("DictionaryMap: duplicate entry name '" + it->first + "'");
            }
            return raw_ptr;
        }
    };

} // namespace gravity
