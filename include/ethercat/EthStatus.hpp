#pragma once

#include <stdint.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <iomanip>
#include "ethercat/Util.hpp"

namespace gravity
{
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
    // BOOST_DESCRIBE_STRUCT(
    //     status_word_entity,
    //     (),
    //     (servo_ready, start, servo_running, fault, main_circuit_power, quick_stop, servo_cannot_run, warning, remote_control, arrived_at_position, internal_limit_valid, origin_found))

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

    // Lets nlohmann::json know how to convert status_word_entity -> json
    inline nlohmann::json to_json(const uint16_t &status)
    {
        const status_word_entity s = decode_status_word(status);
        nlohmann::json j = nlohmann::json{
            {"value", hex_str(status)},
            {"servo_ready", s.servo_ready},
            {"start", s.start},
            {"servo_running", s.servo_running},
            {"fault", s.fault},
            {"main_circuit_power", s.main_circuit_power},
            {"quick_stop", s.quick_stop}, // invert quick stop
            {"servo_cannot_run", s.servo_cannot_run},
            {"warning", s.warning},
            {"remote_control", s.remote_control},
            {"arrived_at_position", s.arrived_at_position},
            {"internal_limit_valid", s.internal_limit_valid},
            {"origin_found", s.origin_found}};
        return j;
    }

    // Your actual to_string, using the ADL conversion above
    inline std::string status_word_str(const uint16_t &status)
    {
        nlohmann::json j = to_json(status); // calls to_json automatically
        return j.dump(4);                   // 4-space pretty print
    }

} // namespace gravity
