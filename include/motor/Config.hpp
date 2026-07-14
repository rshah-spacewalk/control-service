#pragma once

#include <ecrt.h>
#include <stdint.h>
#include <vector>
#include <algorithm>

#include "ethercat/Master.hpp"
#include "ethercat/SdoEx.hpp"
#include "motor/Motor.hpp"
#include <gravity/Clock.hpp>
#include "ethercat/EthEnums.hpp"
#include "controller/MoverConfig.hpp"

#include <gravity/entity/Params.hpp>
#include <gravity/fmt_config.hpp>
#include <gravity/Logger.hpp>

namespace gravity
{
    class MotorConfig
    {
    private:
        std::vector<MotorBase *> motors;
        gravity::trajectory_params params;
        std::shared_ptr<EthercatMaster> master;
        std::shared_ptr<spdlog::logger> _log;

        static void map_custom_pdo(
            const uint16_t slave,
            const std::vector<uint32_t> &rxpdo_entries,
            const std::vector<uint32_t> &txpdo_entries);

    public:
        explicit MotorConfig(
            std::vector<MotorBase *> _motors,
            const trajectory_params &_params,
            const std::shared_ptr<EthercatMaster> &_master)
            : motors(_motors), params(_params), master(_master),
              _log(make_class_logger("MotorConfig"))
        {
        }

        void read_configs();
        void apply_configs();
        void reset_encoders();
        void reset_errors();

        void map_pdos();
        void register_pdos_to_domain();
    };

}