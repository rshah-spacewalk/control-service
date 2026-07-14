#include "controller/Controller.hpp"

bool gravity::Controller::config_cycle()
{
    if (!master->is_requested())
    {
        _log->error("Master not requested");
        return false;
    }

    // 1. optionally map pdos
    if (map_pdos)
    {
        motor_config->map_pdos();
        master->release_master();
        gravity::Clock::fromSeconds(5 * motors.size()).sleepFor();
        master->request_master();
        _log->info("PDO Mapping completed");
    }

    // 2. create domain
    master->create_domain();
    gravity::Clock(2, 0).sleepFor();

    // 3. Register motor pdos with domain
    motor_config->register_pdos_to_domain();
    return true;
}

bool gravity::Controller::enable()
{
    _log->info("Enabling Controller");

    // 0. check if activated
    if (master->is_activated())
    {
        _log->error("Master already Activated");
        return false;
    }

    if (!config_cycle())
    {
        return false;
    }

    // 1. enable master - sets PRE-OP
    master->activate_master();

    // 2. get domain data pointer call after ecrt_master_activate()
    domain_pdm = ecrt_domain_data(master->ec_domain_ptr);
    if (!domain_pdm)
    {
        _log->error("Failed to get domain process data pointer.");
        return false;
    }

    _log->info("Entering cyclic phase with cycle frequency: {} microseconds",
               config::PDO_INTERVAL);

    return true;
}

bool gravity::Controller::disable()
{
    _log->info("Disabling Controller");
    if (master->is_activated())
    {

        // remove domain, deactivate master
        domain_pdm = nullptr;
        master->deactivate_master();
        // master->set_master_state("PREOP");
        return true;
    }
    else
    {
        _log->warn("Master not activated");
    }

    return false;
}

bool gravity::Controller::activate_motors()
{
    _log->info("activating motors");
    for (auto &motor : motors)
    {
        motor->enable();
    }
    return true;
}

bool gravity::Controller::deactivate_motors()
{
    _log->info("deactivating motors");
    for (auto &motor : motors)
    {
        motor->disable();
    }
    return true;
}

bool gravity::Controller::quick_stop()
{
    for (int i = 0; i < motors.size(); i++)
    {
        motors[i]->quick_stop();
    }
    _log->warn("Motors Quick Stop");
    return true;
}

bool gravity::Controller::release_quick_stop()
{
    for (int i = 0; i < motors.size(); i++)
    {
        motors[i]->release_quick_stop();
    }
    _log->warn("Motors Quick Stop Released");
    return true;
}

//  start_cyclic_thread(controller->motors);--
//    activate_master_cycle(); // sets pre-op
//     keep_running.store(true);
//     get_master_state();
//     cyclic_thread = std::thread(cyclic_loop, std::ref(motors));
//         domain -> pdo cyle

// controller->enable_cyclic();    ptr->enable_cyclic();// motor cyclic