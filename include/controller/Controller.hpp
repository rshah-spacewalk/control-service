#pragma once

#include "ethercat/Master.hpp"
#include "motor/Motor.hpp"
#include "motor/Config.hpp"

namespace gravity
{
    class Controller
    {
    private:
        std::vector<uint8_t> enabled = {1};
        std::shared_ptr<EthercatMaster> master;
        std::shared_ptr<MotorConfig> motor_config;
        std::vector<std::unique_ptr<MotorBase>> motors;

        std::shared_ptr<spdlog::logger> _log;

    public:
        explicit Controller();

        ~Controller();

        bool enable();
        bool disable();
        bool quick_stop();
        bool release_quick_stop();

        bool setup(bool map_pdos = false);
        bool loop();
    };
}
