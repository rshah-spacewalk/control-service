#pragma once
#include <array>
#include <cstddef>
#include <stdint.h>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace gravity::config
{
    inline constexpr size_t DOF = 6;
    inline constexpr int PDO_INTERVAL = 1000; // microsecond, only multiplier of 250us.
    inline const char *ETH_DEVICE = "/dev/EtherCAT0";

    // motor settings ---------------------------------------------------------------------------------/

    inline constexpr double MOTOR_PULSE_PER_REVOLUTION = 10'000.0;
    inline constexpr std::array<uint8_t, DOF> MOTOR_DIR{128, 128, 0, 128, 128, 128};
    inline constexpr std::array<double, DOF> GEAR_RATIOS{120, 120, 120, 100, 100, 100};
    inline constexpr std::array<double, DOF> RATED_MOTOR_TORQUES{2.39, 3.18, 2.39, 1.27, 1.27, 0.32}; // N.m
    inline constexpr std::array<double, DOF> RATED_MOTOR_POWERS{750, 1000, 750, 400, 400, 100};       // W

    template <typename IntT>
    inline IntT rad_to_gear_pulse(const double rad, const int joint_index)
    {
        static_assert(std::is_integral_v<IntT>, "IntT must be an integral type");

        const double pulses = (MOTOR_PULSE_PER_REVOLUTION * GEAR_RATIOS[joint_index] * rad) / (2.0 * M_PI);
        const double rounded = std::round(pulses);

        // Guard against wrapping when converting to a narrower or unsigned type
        if (rounded < static_cast<double>(std::numeric_limits<IntT>::min()) ||
            rounded > static_cast<double>(std::numeric_limits<IntT>::max()))
        {
            throw std::overflow_error("rad_to_gear_pulse: computed pulse value out of range for target type");
        }

        return static_cast<IntT>(rounded);
    }

    template <typename IntT>
    inline double gear_pulse_to_rad(const IntT pulse, const int joint_index)
    {
        static_assert(std::is_integral_v<IntT>, "Type must be an integral type");

        if (joint_index < 0 || static_cast<std::size_t>(joint_index) >= GEAR_RATIOS.size())
        {
            throw std::out_of_range("gear_pulse_to_rad: joint_index out of range");
        }

        const double pulses_per_radian_factor = MOTOR_PULSE_PER_REVOLUTION * GEAR_RATIOS[joint_index];
        if (pulses_per_radian_factor == 0.0)
        {
            return 0.0;
        }
        return (static_cast<double>(pulse) * 2.0 * M_PI) / pulses_per_radian_factor;
    }

    inline double joint_torque_nm(const int16_t raw_motor_torque, const int joint_index)
    {
        if (joint_index < 0 || static_cast<std::size_t>(joint_index) >= GEAR_RATIOS.size())
        {
            throw std::out_of_range("joint_torque_nm: joint_index out of range");
        }
        constexpr double efficiency = 0.85; // typically 0.85–0.95 for harmonic drives, 0.90–0.95 for planetary
        const double motor_torque_nm = (static_cast<double>(raw_motor_torque) / 1000.0) * RATED_MOTOR_TORQUES[joint_index];
        return motor_torque_nm * GEAR_RATIOS[joint_index] * efficiency;
    }

    // -------------------------------------------------------------------------------------------------/
}