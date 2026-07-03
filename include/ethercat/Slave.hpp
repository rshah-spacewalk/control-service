#pragma once

#include "ethercat/Util.hpp"
#include "gravity/Logger.hpp"
#include <gravity/Transforms.hpp>
#include <ecrt.h>
#include <stdint.h>

namespace gravity
{

    class SlaveBase
    {
        std::shared_ptr<spdlog::logger> _log;

    public:
        uint16_t alias;
        uint16_t position;
        uint32_t vendor_id;
        uint32_t product_code;
        ec_slave_info_t ec_slave_info{};

        SlaveBase(const uint16_t alias,
                  const uint16_t position,
                  const uint32_t vendor_id,
                  const uint32_t product_code) : alias(alias),
                                                 position(position),
                                                 vendor_id(vendor_id),
                                                 product_code(product_code)

        {
            _log = make_class_logger("SlaveBase");
        }

        SlaveBase(ec_master_t *ec_master_ptr, int position)
        {
            _log = make_class_logger("SlaveBase");

            ec_slave_info_t _ec_slave_info{};
            if (ecrt_master_get_slave(ec_master_ptr, position, &_ec_slave_info) != 0)
            {
                _log->error("Failed to get info for slave index {}", position);
            }
            else
            {
                this->alias = _ec_slave_info.alias;
                this->position = _ec_slave_info.position;
                this->vendor_id = _ec_slave_info.vendor_id;
                this->product_code = _ec_slave_info.product_code;
                this->ec_slave_info = _ec_slave_info;
            }
        }

        ec_slave_info_t get_info(ec_master_t *ec_master_ptr)
        {
            if (ecrt_master_get_slave(ec_master_ptr, position, &ec_slave_info) != 0)
            {
                _log->error("Failed to get info for slave index {}", position);
            }
            return ec_slave_info;
        }

        ec_slave_config_t *get_slave_config_ptr(ec_master_t *ec_master_ptr)
        {
            ec_slave_config_t *cfg = ecrt_master_slave_config(
                ec_master_ptr,
                ec_slave_info.alias,
                ec_slave_info.position,
                ec_slave_info.vendor_id,
                ec_slave_info.product_code);
            if (!cfg)
            {
                _log->error("Failed to get config for slave index {}", ec_slave_info.position);
            }
            return cfg;
        }

        std::string str() const
        {
            std::ostringstream oss;
            auto slave = ec_slave_info;
            oss << "Slave: " << slave.position << " \n"
                << "  Alias      : " << hex_str(slave.alias, 2) << "\n"
                << "  Position   : " << slave.position << "\n"
                << "  Name       : " << slave.name << "\n"
                << "  Product    : " << hex_str(slave.product_code, 8) << "\n"
                << "  Vendor     : " << hex_str(slave.vendor_id, 8) << "\n"
                << "  State.     : " << util::tf::enum_str(slave.al_state) << "\n"
                << "  SDO #      : " << slave.sdo_count << "\n";
            return oss.str();
        }

        ~SlaveBase() = default;
    };

} // namespace gravity::mover