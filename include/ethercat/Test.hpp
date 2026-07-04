#pragma once

#include <array>
#include <cassert>
#include <limits>
#include <memory>
#include <string>

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

    // The canonical list of CoE objects this driver touches. This enum IS the
    // source of truth for how many entries exist -- entries below is sized
    // from Pdo::Count automatically, so there is no separate count to keep in
    // sync by hand. Add a new object by adding an enumerator (before Count)
    // and a matching add() call in the constructor; forget the add() call and
    // the assert loop at the end of the constructor catches it immediately.
    enum class Pdo : uint8_t
    {
        // rx
        ControlWord,
        TargetPosition,
        PositionOffset,
        VelocityOffset,
        TargetTorque,
        TorqueSlope,
        ModeOfOperation,
        // tx
        ErrorCode,
        StatusWord,
        PositionActualValue,
        VelocityActualValue,
        TorqueActualValue,
        FollowingErrorActualValue,
        ModeOfOperationDisplay,
        // configs
        Feed,
        EncoderIncrements,
        PositionActualInternalValue,
        MaxMotorSpeed,
        MaxProfileVelocity,
        MaxAcceleration,
        MaxDeceleration,
        MaxTorque,
        QuickStopOptionCode,
        QuickStopDeceleration,
        MinPositionLimit,
        MaxPositionLimit,
        Polarity,
        DigitalInputs,
        Count // sentinel -- must stay last
    };

    // Owns the CoE object dictionary for one EtherCAT slave/motor.
    //
    // Non-copyable, non-movable by design:
    //   - `entries` owns every DictionaryEntry through a unique_ptr.
    //   - The named pointers below (control_word, target_position, ...) are
    //     non-owning aliases into `entries`, bound once in the constructor.
    //   - Copying would mean deep-cloning every entry AND re-pointing every
    //     alias at the clone -- and two live copies of one motor's dictionary
    //     don't correspond to anything physically meaningful.
    //   - Moving is never required in practice: a DictionaryMap should only
    //     ever live as a member of a Motor that itself lives behind
    //     std::unique_ptr<Motor>, so it's constructed once, in place, and
    //     never relocated afterward.
    //
    // (A defaulted move would actually be safe if you ever needed one:
    // moving a std::array of unique_ptrs relocates the pointers, never the
    // heap objects they point at, so control_word etc. stay valid. Swap the
    // two deletes below for:
    //   DictionaryMap(DictionaryMap &&) noexcept = default;
    //   DictionaryMap &operator=(DictionaryMap &&) noexcept = default;
    // )
    class DictionaryMap
    {
    public:
        explicit DictionaryMap(uint8_t position) : position_(position)
        {
            using namespace limits;

            // rx
            control_word = add<uint16_t>(Pdo::ControlWord, {.name = "control_word", .index = 0x6040, .subindex = 0x0, .min = U16_MIN, .max = U16_MAX, .def = 0, .uom = "_", .mapping_type = MappingType::RX_PDO});
            target_position = add<int32_t>(Pdo::TargetPosition, {.name = "target_position", .index = 0x607A, .subindex = 0x0, .min = I32_MIN, .max = I32_MAX, .def = 0, .uom = "Uint", .mapping_type = MappingType::RX_PDO});
            position_offset = add<int32_t>(Pdo::PositionOffset, {.name = "position_offset", .index = 0x60B0, .subindex = 0x0, .min = I32_MIN, .max = I32_MAX, .def = 0, .uom = "Uint", .mapping_type = MappingType::RX_PDO});
            velocity_offset = add<int32_t>(Pdo::VelocityOffset, {.name = "velocity_offset", .index = 0x60B1, .subindex = 0x0, .min = I32_MIN, .max = I32_MAX, .def = 0, .uom = "Uint/s", .mapping_type = MappingType::RX_PDO});
            target_torque = add<int16_t>(Pdo::TargetTorque, {.name = "target_torque", .index = 0x6071, .subindex = 0x0, .min = I16_MIN, .max = I16_MAX, .def = 0, .uom = "0.1%", .mapping_type = MappingType::RX_PDO});
            torque_slope = add<uint32_t>(Pdo::TorqueSlope, {.name = "torque_slope", .index = 0x6087, .subindex = 0x0, .min = 1, .max = I32_MAX, .def = 5'000, .uom = "0.1%/s", .mapping_type = MappingType::RX_PDO});
            mode_of_operation = add<int8_t>(Pdo::ModeOfOperation, {.name = "mode_of_operation", .index = 0x6060, .subindex = 0x0, .min = 1, .max = 11, .def = 8, .uom = "_", .mapping_type = MappingType::RX_PDO});

            // tx
            error_code = add<uint16_t>(Pdo::ErrorCode, {.name = "error_code", .index = 0x603F, .subindex = 0x0, .min = U16_MIN, .max = U16_MAX, .def = 0, .uom = "_", .mapping_type = MappingType::TX_PDO});
            status_word = add<uint16_t>(Pdo::StatusWord, {.name = "status_word", .index = 0x6041, .subindex = 0x0, .min = U16_MIN, .max = U16_MAX, .def = 0, .uom = "_", .mapping_type = MappingType::TX_PDO});
            position_actual_value = add<int32_t>(Pdo::PositionActualValue, {.name = "position_actual_value", .index = 0x6064, .subindex = 0x0, .min = I32_MIN, .max = I32_MAX, .def = 0, .uom = "Uint", .mapping_type = MappingType::TX_PDO});
            velocity_actual_value = add<int32_t>(Pdo::VelocityActualValue, {.name = "velocity_actual_value", .index = 0x606C, .subindex = 0x0, .min = I32_MIN, .max = I32_MAX, .def = 0, .uom = "Uint/s", .mapping_type = MappingType::TX_PDO});
            torque_actual_value = add<int16_t>(Pdo::TorqueActualValue, {.name = "torque_actual_value", .index = 0x6077, .subindex = 0x0, .min = I16_MIN, .max = I16_MAX, .def = 0, .uom = "0.1%", .mapping_type = MappingType::TX_PDO});
            following_error_actual_value = add<int32_t>(Pdo::FollowingErrorActualValue, {.name = "following_error_actual_value", .index = 0x60F4, .subindex = 0x0, .min = I32_MIN, .max = I32_MAX, .def = 0, .uom = "Uint", .mapping_type = MappingType::TX_PDO});
            mode_of_operation_display = add<int8_t>(Pdo::ModeOfOperationDisplay, {.name = "mode_of_operation_display", .index = 0x6061, .subindex = 0x0, .min = 1, .max = 11, .def = 0, .uom = "_", .mapping_type = MappingType::TX_PDO});

            // configs
            feed = add<uint32_t>(Pdo::Feed, {.name = "feed", .index = 0x6092, .subindex = 0x01, .min = 1, .max = I32_MAX, .def = 10'000, .uom = "Uint", .mapping_type = MappingType::RX_PDO});                                      // pulse per revolution
            encoder_increments = add<uint32_t>(Pdo::EncoderIncrements, {.name = "encoder_increments", .index = 0x608F, .subindex = 0x01, .min = 0, .max = I32_MAX, .def = 0, .uom = "Pulse", .mapping_type = MappingType::TX_PDO}); // smallest measurable change in pos
            position_actual_internal_value = add<int32_t>(Pdo::PositionActualInternalValue, {.name = "position_actual_internal_value", .index = 0x6063, .subindex = 0x0, .min = I32_MIN, .max = I32_MAX, .def = 0, .uom = "Pulse", .mapping_type = MappingType::TX_PDO});

            max_motor_speed = add<uint32_t>(Pdo::MaxMotorSpeed, {.name = "max_motor_speed", .index = 0x6080, .subindex = 0x0, .min = U32_MIN, .max = I32_MAX, .def = 6'000, .uom = "r/min", .mapping_type = MappingType::RX_PDO});
            max_profile_velocity = add<uint32_t>(Pdo::MaxProfileVelocity, {.name = "max_profile_velocity", .index = 0x607F, .subindex = 0x0, .min = U32_MIN, .max = I32_MAX, .def = I32_MAX, .uom = "Uint/s", .mapping_type = MappingType::RX_PDO}); // 36
            max_acceleration = add<uint32_t>(Pdo::MaxAcceleration, {.name = "max_acceleration", .index = 0x60C5, .subindex = 0x0, .min = 1, .max = I32_MAX, .def = 10'00'00'000, .uom = "Uint/s^2", .mapping_type = MappingType::RX_PDO});
            max_deceleration = add<uint32_t>(Pdo::MaxDeceleration, {.name = "max_deceleration", .index = 0x60C6, .subindex = 0x0, .min = 1, .max = I32_MAX, .def = 10'00'00'000, .uom = "Uint/s^2", .mapping_type = MappingType::RX_PDO});
            max_torque = add<uint16_t>(Pdo::MaxTorque, {.name = "max_torque", .index = 0x6072, .subindex = 0x0, .min = U16_MIN, .max = U16_MAX, .def = 3000, .uom = "0.1%", .mapping_type = MappingType::RX_PDO});

            quick_stop_option_code = add<int16_t>(Pdo::QuickStopOptionCode, {.name = "quick_stop_option_code", .index = 0x605A, .subindex = 0x0, .min = 0, .max = 7, .def = 2, .uom = "_", .mapping_type = MappingType::RX_PDO});
            quick_stop_deceleration = add<uint32_t>(Pdo::QuickStopDeceleration, {.name = "quick_stop_deceleration", .index = 0x6085, .subindex = 0x0, .min = 1, .max = I32_MAX, .def = 10'000'000, .uom = "Uint/s^2", .mapping_type = MappingType::RX_PDO}); // e-stop

            min_position_limit = add<int32_t>(Pdo::MinPositionLimit, {.name = "min_position_limit", .index = 0x607D, .subindex = 0x01, .min = I32_MIN, .max = I32_MAX, .def = 0, .uom = "Uint", .mapping_type = MappingType::RX_PDO});
            max_position_limit = add<int32_t>(Pdo::MaxPositionLimit, {.name = "max_position_limit", .index = 0x607D, .subindex = 0x02, .min = I32_MIN, .max = I32_MAX, .def = 0, .uom = "Uint", .mapping_type = MappingType::RX_PDO});
            polarity = add<uint8_t>(Pdo::Polarity, {.name = "polarity", .index = 0x607E, .subindex = 0x0, .min = U8_MIN, .max = U8_MAX, .def = 0x0, .uom = "_", .mapping_type = MappingType::RX_PDO});
            digital_inputs = add<uint32_t>(Pdo::DigitalInputs, {.name = "digital_inputs", .index = 0x60FD, .subindex = 0x0, .min = U32_MIN, .max = I32_MAX, .def = 0x0, .uom = "_", .mapping_type = MappingType::TX_PDO});

            for (const auto &e : entries)
            {
                assert(e && "DictionaryMap: a Pdo entry was declared but never add()-ed");
            }
        }

        DictionaryMap(const DictionaryMap &) = delete;
        DictionaryMap &operator=(const DictionaryMap &) = delete;
        DictionaryMap(DictionaryMap &&) = delete;
        DictionaryMap &operator=(DictionaryMap &&) = delete;

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

        // Generic access by id -- O(1) array index, no string hashing. Handy
        // for logging/diagnostics code that wants to walk "every entry"
        // without caring about the concrete type.
        const DictionaryEntity *get(Pdo id) const noexcept { return entries[static_cast<size_t>(id)].get(); }

        auto begin() const noexcept { return entries.begin(); }
        auto end() const noexcept { return entries.end(); }

    private:
        uint8_t position_;
        std::array<std::unique_ptr<DictionaryEntity>, static_cast<size_t>(Pdo::Count)> entries{};

        template <typename T>
        struct EntrySpec
        {
            std::string name;
            uint16_t index;
            uint8_t subindex;
            T min;
            T max;
            T def;
            std::string uom;
            MappingType mapping_type;
        };

        template <typename T>
        DictionaryEntry<T> *add(Pdo id, EntrySpec<T> spec)
        {
            auto entry = std::make_unique<DictionaryEntry<T>>(
                std::move(spec.name), spec.index, spec.subindex, position_,
                spec.min, spec.max, spec.def, std::move(spec.uom), spec.mapping_type);

            auto *raw_ptr = entry.get();
            entries[static_cast<size_t>(id)] = std::move(entry);
            return raw_ptr;
        }
    };

} // namespace gravity