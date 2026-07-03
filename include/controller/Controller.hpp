#pragma once

#include "ethercat/Master.hpp"
#include "motor/Motor.hpp"
#include "motor/Config.hpp"

namespace gravity
{
    class Controller
    {
    private:
        std::vector<MotorBase> motors;
        std::shared_ptr<EthercatMaster> master;
        std::shared_ptr<MotorConfig> motor_config;
        std::shared_ptr<spdlog::logger> _log;

    public:
        explicit Controller()
        {
        }
    };
}