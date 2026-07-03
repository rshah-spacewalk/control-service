#pragma once

#include <ecrt.h>
#include <stdint.h>
#include <vector>
#include <algorithm>

#include "ethercat/Master.hpp"
#include "ethercat/SdoEx.hpp"
#include "motor/Motor.hpp"
#include <gravity/Clock.hpp>
#include "Kepler.hpp"
#include "ethercat/EthEnums.hpp"

#include <gravity/fmt_config.hpp>
#include <gravity/Logger.hpp>

namespace gravity
{
    class MotorConfig
    {
    private:
        std::vector<uint8_t> enabled;
        std::vector<MotorBase> motors;
        std::shared_ptr<EthercatMaster> master;
        std::shared_ptr<spdlog::logger> _log;

    public:
        explicit MotorConfig(
            const std::shared_ptr<EthercatMaster> &_master,
            std::vector<MotorBase> &_motors,
            const std::vector<uint8_t> &_enabled)
            : master(_master), motors(_motors), enabled(_enabled),
              _log(make_class_logger("MotorConfig"))
        {
        }
        static void map_custom_pdo(
            const uint16_t slave,
            const std::vector<uint32_t> &rxpdo_entries,
            const std::vector<uint32_t> &txpdo_entries);

        void map_pdos();
        void register_pdos();
        void apply_configs();
        void reset_encoder();
        void reset_errors();
    };

}