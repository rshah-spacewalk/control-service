#pragma once

#include "ethercat/Master.hpp"
#include "motor/Motor.hpp"
#include "motor/Config.hpp"

namespace gravity
{
    class Controller
    {
    private:
        const size_t DOF = 6;
        bool map_pdos{false};
        std::array<double, 6> motor_position{};
        std::shared_ptr<EthercatMaster> master;
        std::shared_ptr<MotorConfig> motor_config;
        std::vector<std::unique_ptr<MotorBase>> motors;

        std::shared_ptr<spdlog::logger> _log;

    public:
        explicit Controller(bool _map_pdos = false);

        ~Controller();

        bool enable();
        bool disable();
        bool config_cycle();

        bool quick_stop();
        bool release_quick_stop();

        bool handle_error();
        bool handle_status_word();

        bool setup(bool strict = true);
        bool loop();
    };
}

// contructor order : top -> bottom
// destructor order : bottom -> top