#pragma once
#include <string>
#include <ecrt.h>

#include <gravity/fmt_config.hpp>
#include <gravity/Logger.hpp>

#include "ethercat/Util.hpp"

namespace gravity
{
    class EthercatMaster
    {
    public:
        uint16_t master_index;
        std::string ec_device; //"/dev/EtherCAT0"
        ec_master_t *ec_master_ptr;
        ec_domain_t *ec_domain_ptr;

        explicit EthercatMaster(const std::string &device_name, const uint16_t &_master_index = 0)
            : master_index(_master_index),
              ec_device(device_name),
              ec_master_ptr(nullptr),
              ec_domain_ptr(nullptr),
              _log(make_class_logger("Master"))
        {
        }

        EthercatMaster(EthercatMaster &&) = delete;
        EthercatMaster(const EthercatMaster &) = delete;
        EthercatMaster &operator=(EthercatMaster &&) = delete;
        EthercatMaster &operator=(const EthercatMaster &) = delete;

        bool request_master();
        bool release_master();
        bool create_domain();

        // ecrt_open_master(master_index) get master
        bool get_ec_master_info(ec_master_info_t &info);
        bool get_ec_master_state(ec_master_state_t &state);
        int set_master_state(std::string_view state);

        bool is_activated();
        bool is_requested();

        ~EthercatMaster();

    private:
        std::shared_ptr<spdlog::logger> _log;
    };
}

// ec_domain_state_t
// ecrt_domain_size
