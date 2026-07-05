#include "motor/Motor.hpp"

void gravity::MotorBase::build_data_objects()
{
    constexpr uint8_t U8_MIN = std::numeric_limits<uint8_t>::min();    // 0 (0x0)
    constexpr uint8_t U8_MAX = std::numeric_limits<uint8_t>::max();    // 255 (0xff)
    constexpr uint16_t U16_MIN = std::numeric_limits<uint16_t>::min(); //  0 (0x0)
    constexpr uint16_t U16_MAX = std::numeric_limits<uint16_t>::max(); // 65535 (0xffff)
    constexpr uint32_t U32_MIN = std::numeric_limits<uint32_t>::min(); // 0 (0x0)
    constexpr uint32_t U32_MAX = std::numeric_limits<uint32_t>::max(); // 4294967295 (0xffffffff)
    constexpr int8_t I8_MIN = std::numeric_limits<int8_t>::min();      // -128 (0xffffff80)
    constexpr int8_t I8_MAX = std::numeric_limits<int8_t>::max();      // 127 (0x7f)
    constexpr int16_t I16_MIN = std::numeric_limits<int16_t>::min();   //  -32768 (0xffff8000)
    constexpr int16_t I16_MAX = std::numeric_limits<int16_t>::max();   // 32767 (0x7fff)
    constexpr int32_t I32_MIN = std::numeric_limits<int32_t>::min();   // -2147483648 (0x80000000)
    constexpr int32_t I32_MAX = std::numeric_limits<int32_t>::max();   // 2147483647 (0x7fffffff)

    // rx
    control_word = std::make_unique<DataObject<uint16_t>>("control_word", position, 0x6040, 0x0, U16_MIN, U16_MAX, 0, "_", MappingType::RX_PDO);
    target_position = std::make_unique<DataObject<int32_t>>("target_position", position, 0x607A, 0x0, I32_MIN, I32_MAX, 0, "Uint", MappingType::RX_PDO);
    position_offset = std::make_unique<DataObject<int32_t>>("position_offset", position, 0x60B0, 0x0, I32_MIN, I32_MAX, 0, "Uint", MappingType::RX_PDO);
    velocity_offset = std::make_unique<DataObject<int32_t>>("velocity_offset", position, 0x60B1, 0x0, I32_MIN, I32_MAX, 0, "Uint/s", MappingType::RX_PDO);
    target_torque = std::make_unique<DataObject<int16_t>>("target_torque", position, 0x6071, 0x0, I16_MIN, I16_MAX, 0, "0.1%", MappingType::RX_PDO);
    torque_slope = std::make_unique<DataObject<uint32_t>>("torque_slope", position, 0x6087, 0x0, 1, I32_MAX, 5'000, "0.1%/s", MappingType::RX_PDO);
    mode_of_operation = std::make_unique<DataObject<int8_t>>("mode_of_operation", position, 0x6060, 0x0, 1, 11, 8, "_", MappingType::RX_PDO);

    // tx
    error_code = std::make_unique<DataObject<uint16_t>>("error_code", position, 0x603F, 0x0, U16_MIN, U16_MAX, 0, "_", MappingType::TX_PDO);
    status_word = std::make_unique<DataObject<uint16_t>>("status_word", position, 0x6041, 0x0, U16_MIN, U16_MAX, 0, "_", MappingType::TX_PDO);
    position_actual_value = std::make_unique<DataObject<int32_t>>("position_actual_value", position, 0x6064, 0x0, I32_MIN, I32_MAX, 0, "Uint", MappingType::TX_PDO);
    velocity_actual_value = std::make_unique<DataObject<int32_t>>("velocity_actual_value", position, 0x606C, 0x0, I32_MIN, I32_MAX, 0, "Uint/s", MappingType::TX_PDO);
    torque_actual_value = std::make_unique<DataObject<int16_t>>("torque_actual_value", position, 0x6077, 0x0, I16_MIN, I16_MAX, 0, "0.1%", MappingType::TX_PDO);
    following_error_actual_value = std::make_unique<DataObject<int32_t>>("following_error_actual_value", position, 0x60F4, 0x0, I32_MIN, I32_MAX, 0, "Uint", MappingType::TX_PDO);
    mode_of_operation_display = std::make_unique<DataObject<int8_t>>("mode_of_operation_display", position, 0x6061, 0x0, 1, 11, 0, "_", MappingType::TX_PDO);

    // configs
    feed = std::make_unique<DataObject<uint32_t>>("feed", position, 0x6092, 0x01, 1, I32_MAX, 10'000, "Uint", MappingType::RX_PDO);                                                     // 15
    encoder_increments = std::make_unique<DataObject<uint32_t>>("encoder_increments", position, 0x608F, 0x01, 0, I32_MAX, 0, "Pulse", MappingType::TX_PDO);                             // 16
    position_actual_internal_value = std::make_unique<DataObject<int32_t>>("position_actual_internal_value", position, 0x6063, 0x0, I32_MIN, I32_MAX, 0, "Pulse", MappingType::TX_PDO); // 17

    max_motor_speed = std::make_unique<DataObject<uint32_t>>("max_motor_speed", position, 0x6080, 0x0, U32_MIN, I32_MAX, 6'000, "r/min", MappingType::RX_PDO);              // 18
    max_profile_velocity = std::make_unique<DataObject<uint32_t>>("max_profile_velocity", position, 0x607F, 0x0, U32_MIN, I32_MAX, I32_MAX, "Uint/s", MappingType::RX_PDO); // 36// 19
    max_acceleration = std::make_unique<DataObject<uint32_t>>("max_acceleration", position, 0x60C5, 0x0, 1, I32_MAX, 10'00'00'000, "Uint/s^2", MappingType::RX_PDO);        // 20
    max_deceleration = std::make_unique<DataObject<uint32_t>>("max_deceleration", position, 0x60C6, 0x0, 1, I32_MAX, 10'00'00'000, "Uint/s^2", MappingType::RX_PDO);        // 21
    max_torque = std::make_unique<DataObject<uint16_t>>("max_torque", position, 0x6072, 0x0, U16_MIN, U16_MAX, 3000, "0.1%", MappingType::RX_PDO);                          // 22

    quick_stop_option_code = std::make_unique<DataObject<int16_t>>("quick_stop_option_code", position, 0x605A, 0x0, 0, 7, 2, "_", MappingType::RX_PDO); // 23
    quick_stop_deceleration = std::make_unique<DataObject<uint32_t>>("quick_stop_deceleration", position, 0x6085, 0x0, 1, I32_MAX, 10'000'000, "Uint/s^2", MappingType::RX_PDO);

    min_position_limit = std::make_unique<DataObject<int32_t>>("min_position_limit", position, 0x607D, 0x01, I32_MIN, I32_MAX, 0, "Uint", MappingType::RX_PDO); // 25
    max_position_limit = std::make_unique<DataObject<int32_t>>("max_position_limit", position, 0x607D, 0x02, I32_MIN, I32_MAX, 0, "Uint", MappingType::RX_PDO); // 26
    polarity = std::make_unique<DataObject<uint8_t>>("polarity", position, 0x607E, 0x0, U8_MIN, U8_MAX, 0x0, "_", MappingType::RX_PDO);                         // 27
    digital_inputs = std::make_unique<DataObject<uint32_t>>("digital_inputs", position, 0x60FD, 0x0, U32_MIN, I32_MAX, 0x0, "_", MappingType::TX_PDO);          // 28
}
