#pragma once

#include <stdint.h>

namespace gravity
{
    enum class AbsoluteEncoderMode : uint32_t
    {
        INCREMENTAL = 0x0,
        MULTI_TURN_LINEAR = 0x1,
        MULTI_TURN_ROTARY = 0x2,
        SINGLE_TURN_ABSOLUTE = 0x3,
        CLEAR_MULTI_TURN_ALARM_ACTIVATE_MULTI_TURN_ABSOLUTE = 0x5,
        CLEAR_MULTI_TURN_POSITION_RESET_MULTI_TURN_ALARM_ACTIVATE_MULTI_TURN_ABSOLUTE = 0x9
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

    enum class SLAVE_STATUS_WORD : uint16_t
    {
        POWER_ON = 0x0000,       // power on
        FAULTLESS = 0x0250,      // Drive not ready yet -> 0x0250
        READY = 0x0231,          // Switch on disabled, -> 0x0231
        WAITING_ENABLE = 0x0233, // Ready to switch on -> 0x0233
        RUNNING = 0x0237,        // Operation enabled -> 0x0237
        QUICK_STOP = 0x0217,     // Quick stop active -> 0x0217
        FAULT_STOP = 0x021F,     // fault stop -> 0x021F
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

} // namespace gravity::mover