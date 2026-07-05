#include "controller/Controller.hpp"

bool gravity::Controller::config_cycle()
{
    try
    {
        if (master->is_requested())
        {
            // 0. optionally map pdos
            if (map_pdos)
            {
                motor_config->map_pdos();
                master->release_master();
                gravity::Clock::fromSeconds(5 * motors.size()).sleepFor();
                master->request_master();
                master->create_domain();
                _log->info("PDO Mapping completed");
            }

            // 1. create domain
            master->create_domain();
            gravity::Clock(2, 0).sleepFor();

            // 2. Register motor pdos with domain
            motor_config->register_pdos_to_domain();
            return true;
        }
        else
        {
            _log->error("Master not requested");
            return false;
        }
    }
    catch (const std::exception &e)
    {
        _log->error("{}", e.what());
    }
    return false;
}

bool gravity::Controller::enable()
{
    if (master->is_activated())
    {
        _log->error("Master already Activated");
        return false;
    }

    try
    {
    }
    catch (const std::exception &e)
    {
    }
    return false;
}

bool gravity::Controller::disable()
{
    return true;
}
