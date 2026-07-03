#pragma once
#include <array>
#include <cmath>
#include <cstddef>
#include <stdint.h>

namespace gravity::config::kepler
{
    inline constexpr size_t DOF = 6;
    inline constexpr int PDO_INTERVAL = 500; // microsecond, only multiplier of 250us.
    inline const char *ETH_DEVICE = "/dev/EtherCAT0";

    // Joint settings -------------------------------------------------------------------------------/

    inline constexpr std::array<double, DOF> MIN_JOINT_LIMITS{
        -M_PI, 0.0, -2.09439510239, -M_PI, -2.09439510239, -M_PI}; // rad
    inline constexpr std::array<double, DOF> MAX_JOINT_LIMITS{M_PI, M_PI, 2.09439510239, M_PI, 2.09439510239, M_PI};
    // rad

    inline constexpr std::array<double, DOF> MAX_JOINT_VEL{0.6, 0.6, 0.6, 0.6, 0.6, 0.6};       //[rad/s] ~ M_PI/6
    inline constexpr std::array<double, DOF> MIN_JOINT_VEL{-0.6, -0.6, -0.6, -0.6, -0.6, -0.6}; //[rad/s]

    inline constexpr std::array<double, DOF> MAX_JOINT_ACL{3.0, 3.0, 3.0, 3.0, 3.0, 3.0};       //[rad/s²] ~ 2 * M_PI
    inline constexpr std::array<double, DOF> MIN_JOINT_ACL{-3.0, -3.0, -3.0, -3.0, -3.0, -3.0}; //[rad/s²]

    inline constexpr std::array<double, DOF> MAX_JOINT_JERK{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}; // rad/s³

    // inline constexpr std::array<double, DOF> MAX_JOINT_JERK{0.5, 0.5, 0.5, 0.5, 0.5, 0.5};      //[rad/s3]
    // inline constexpr std::array<double, DOF> MAX_JOINT_VEL{2.0, 2.0, 2.0, 2.0, 2.0, 2.0};        // rad/s
    // inline constexpr std::array<double, DOF> MAX_JOINT_ACL{5.0, 5.0, 5.0, 5.0, 5.0, 5.0};        // rad/s²

    // motor settings ---------------------------------------------------------------------------------/

    inline constexpr double MOTOR_PULSE_PER_REVOLUTION = 10'000.0;
    inline constexpr std::array<uint8_t, DOF> MOTOR_DIR{128, 128, 0, 128, 128, 128};
    inline constexpr std::array<double, DOF> GEAR_RATIOS{120, 120, 120, 100, 100, 100};
    inline constexpr std::array<double, DOF> RATED_MOTOR_TORQUES{2.39, 3.18, 2.39, 1.27, 1.27, 0.32}; // N.m
    inline constexpr std::array<double, DOF> RATED_MOTOR_POWERS{750, 1000, 750, 400, 400, 100};       // W

    // rated angular velocity, W = P_rated / T_rated
    // rpm = (w*60)/2*pi

    constexpr double rad_to_gear_pulse(const double rad, const int joint_index)
    {
        return (MOTOR_PULSE_PER_REVOLUTION * GEAR_RATIOS[joint_index] * rad) / (2.0 * M_PI);
    }

    constexpr double gear_pulse_to_rad(const double pulse, const int joint_index)
    {
        const double pulses_per_radian_factor = MOTOR_PULSE_PER_REVOLUTION * GEAR_RATIOS[joint_index];
        if (pulses_per_radian_factor == 0.0)
        {
            return 0.0;
        }
        return (pulse * 2.0 * M_PI) / pulses_per_radian_factor;
    }

    constexpr double joint_torque_nm(const int16_t raw_torque, const int joint_index)
    {
        constexpr double efficiency = 0.85; // typically 0.85–0.95 for harmonic drives, 0.90–0.95 for planetary
        const double motor_torque_nm = (raw_torque / 1000.0) * RATED_MOTOR_TORQUES[joint_index];
        return motor_torque_nm * GEAR_RATIOS[joint_index] * efficiency;
    }

    // -------------------------------------------------------------------------------------------------/
}