#pragma once

#include <stdint.h>

namespace gravity
{
    enum class AbsoluteEncoderMode : uint32_t
    {
        INCREMENTAL = 0,
        MULTI_TURN_LINEAR = 1,
        MULTI_TURN_ROTARY = 2,
        SINGLE_TURN_ABSOLUTE = 3,
        CLEAR_MULTI_TURN_ALARM_ACTIVATE_MULTI_TURN_ABSOLUTE = 5,
        CLEAR_MULTI_TURN_POSITION_RESET_MULTI_TURN_ALARM_ACTIVATE_MULTI_TURN_ABSOLUTE = 9
    };

    enum class SLAVE_CONTROL_WORD : uint16_t
    {
        FAULTLESS = 0x0000,          // Drive not ready yet -> 0x0250
        READY = 0x0006,              // Switch on disabled, -> 0x0231
        WAITING_ENABLE = 0x0007,     // Ready to switch on -> 0x0233
        RUNNING = 0x000F,            // Operation enabled -> 0x0237
        QUICK_STOP = 0x0002,         // Quick stop active -> 0x0217, fault stop -> 0x021F
        FAULT_TO_FAULTLESS = 0x0080, // Fault reset -> 0x0250
    };

    enum class AUTO_GAIN_TYPE : int32_t
    {
        RIGID_MAN = 0x000,
        RIGID_STD = 0x001, // Gain switch not used
        RIGID_POS = 0x002, // needs torque command additional value
        HIGH_INERTIA_MAN = 0x010,
        HIGH_INERTIA_STD = 0x011,
        HIGH_INERTIA_POS = 0x012,
        FLEX_MAN = 0x020,
        FLEX_STD = 0x021,
        FLEX_POS = 0x022
    };

    enum class StatusWordFlag : uint16_t
    {
        None = 0,
        ServoReady = 1 << 0,          // 0x0001
        Start = 1 << 1,               // 0x0002
        ServoRunning = 1 << 2,        // 0x0004
        Fault = 1 << 3,               // 0x0008
        MainCircuitPower = 1 << 4,    // 0x0010
        QuickStop = 1 << 5,           // 0x0020 (1: Invalid/Normal, 0: Active/Stopping)
        ServoCannotRun = 1 << 6,      // 0x0040
        Warning = 1 << 7,             // 0x0080
        RemoteControl = 1 << 9,       // 0x0200
        ArrivedAtPosition = 1 << 10,  // 0x0400
        InternalLimitValid = 1 << 11, // 0x0800
        OriginFound = 1 << 15         // 0x8000
    };

    inline StatusWordFlag to_status_word_enum(const uint16_t &status)
    {
        return static_cast<StatusWordFlag>(status);
    }

    struct status_word_entity
    {
        bool servo_ready{};          // Bit 0
        bool start{};                // Bit 1
        bool servo_running{};        // Bit 2
        bool fault{};                // Bit 3
        bool main_circuit_power{};   // Bit 4
        bool quick_stop{};           // Bit 5 (0: active, 1: invalid/inactive)
        bool servo_cannot_run{};     // Bit 6
        bool warning{};              // Bit 7
        bool remote_control{};       // Bit 9
        bool arrived_at_position{};  // Bit 10
        bool internal_limit_valid{}; // Bit 11
        bool origin_found{};         // Bit 15
    };

    inline status_word_entity decode_status_word(const uint16_t &status)
    {
        status_word_entity s{};
        // 0 1 2 3
        s.servo_ready = (status & (1 << 0)) != 0;
        s.start = (status & (1 << 1)) != 0;
        s.servo_running = (status & (1 << 2)) != 0;
        s.fault = (status & (1 << 3)) != 0;
        // 4 5 6 7
        s.main_circuit_power = (status & (1 << 4)) != 0;
        s.quick_stop = (status & (1 << 5)) != 0; // 0 : valid, 1: invalid
        s.servo_cannot_run = (status & (1 << 6)) != 0;
        s.warning = (status & (1 << 7)) != 0;
        // 9 10 11 15
        s.remote_control = (status & (1 << 9)) != 0; // on because receiving commands via network
        s.arrived_at_position = (status & (1 << 10)) != 0;
        s.internal_limit_valid = (status & (1 << 11)) != 0;
        s.origin_found = (status & (1 << 15)) != 0;
        return s;
    }
} // namespace gravity::mover