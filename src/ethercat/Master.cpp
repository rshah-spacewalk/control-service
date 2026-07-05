#include "ethercat/Master.hpp"

bool gravity::EthercatMaster::request_master()
{
    _log->info("Initializing EtherCAT Master...");

    // 1. Check if EtherCAT device is accessible
    if (access(ec_device.c_str(), R_OK | W_OK) != 0)
    {
        const int saved_errno = errno;
        const std::string cmd = "sudo depmod && sudo ethercatctl restart";
        auto err = fmt::format(
            "Cannot access '{}': {}. Suggested fix: {}",
            ec_device, std::strerror(saved_errno), cmd);
        _log->error("{}", err);
        throw std::runtime_error(err);
    }

    // 2. Prevent double initialization
    if (ec_master_ptr)
    {
        _log->warn("EtherCAT master instance already requested — skipping re-initialization.");
        return true;
    }

    // 3. Request EtherCAT master
    ec_master_ptr = ecrt_request_master(master_index);
    if (!ec_master_ptr)
    {
        auto err = "Failed to request EtherCAT master.";
        _log->error("{}", err);
        throw std::runtime_error(err);
    }
    _log->info("EtherCAT Master instance [{}] initialized.", master_index);

    return true;
}

bool gravity::EthercatMaster::release_master()
{
    if (ec_master_ptr == nullptr)
    {
        _log->warn("close_master: nothing to close, ec_master_ptr is null");
        return false;
    }

    ecrt_release_master(ec_master_ptr);
    ec_master_ptr = nullptr;
    ec_domain_ptr = nullptr;
    _log->info("EtherCAT Master Released");
    return true;
}

bool gravity::EthercatMaster::create_domain()
{
    ec_domain_ptr = ecrt_master_create_domain(ec_master_ptr);
    if (!ec_domain_ptr)
    {
        ecrt_release_master(ec_master_ptr);
        ec_master_ptr = nullptr;
        throw std::runtime_error("Failed to create EtherCAT domain. Master released.");
    }
    _log->info("EtherCAT domain created successfully.");
    return true;
}

bool gravity::EthercatMaster::get_ec_master_info(ec_master_info_t &info)
{
    if (ec_master_ptr == nullptr)
    {
        throw std::runtime_error("EtherCAT Master not found!");
    }
    else if (ecrt_master(ec_master_ptr, &info))
    {
        _log->error("Failed to get master info: {}", std::strerror(errno));
        return false;
    }
    return true;
}

bool gravity::EthercatMaster::get_ec_master_state(ec_master_state_t &state)
{
    if (ec_master_ptr == nullptr)
    {
        throw std::runtime_error("get_master_state: EtherCAT Master not found!");
    }
    else if (ecrt_master_state(ec_master_ptr, &state))
    {
        _log->error("Failed to get master state: {}", std::strerror(errno));
        return false;
    }
    return true;
}

int gravity::EthercatMaster::set_master_state(std::string_view state)
{
    std::string cmd = "ethercat state " + std::string(state);
    int resp = std::system(cmd.c_str());
    _log->warn("[{}] {}", (resp ? "FAILED" : "SUCCESS"), cmd);
    return resp;
}

bool gravity::EthercatMaster::is_activated()
{
    ec_master_state_t state;
    if (get_ec_master_state(state))
    {
        return state.al_states == 8;
    }
    else
        return false;
}

bool gravity::EthercatMaster::is_requested()
{
    ec_master_state_t state;
    if (get_ec_master_state(state))
    {
        return state.al_states == 2 || state.al_states == 4;
    }
    else
        return false;
}

gravity::EthercatMaster::~EthercatMaster()
{
    release_master();
    _log->info("Removing Master instance");
}
