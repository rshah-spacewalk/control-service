#include "controller/Controller.hpp"
#include "ethercat/ThreadUtil.hpp"

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
        // master->create_domain();
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

    // 1. enable master
    master->activate_master();

    // 2. get domain data pointer
    domain_pdm = ecrt_domain_data(master->ec_domain_ptr);
    if (!domain_pdm)
    {
        _log->error("Failed to get domain process data pointer.");
        return false;
    }

    // 3. config motor pdos & enable motor cycle
    ThreadGroup init_threads(motors.size());
    for (auto &motor : motors)
    {
        init_threads.threads.emplace_back(
            [raw_motor = motor.get()]()
            {
                raw_motor->enable();
            });
    }

    _log->info("Entering cyclic phase with cycle frequency: {} microseconds",
               config::PDO_INTERVAL);

    ec_master_state_t state = get_master_state();
    _log->info("Master State [{}]", al_state_str(state.al_states));

    return true;
}

bool gravity::Controller::disable()
{
    _log->info("Disabling Controller");
    if (master->is_activated())
    {
        // 2. disable motor cycle
        ThreadGroup stop_threads(motors.size());
        for (auto &motor : motors)
        {
            stop_threads.threads.emplace_back(
                [raw_motor = motor.get()]()
                {
                    raw_motor->disable();
                });
        }
        master->deactivate_master();
        master->set_master_state("PREOP");
        quick_stop_on.store(false);
        return true;
    }
    else
    {
        _log->warn("Master not activated");
    }

    return false;
}

bool gravity::Controller::quick_stop()
{
    for (int i = 0; i < motors.size(); i++)
    {
        motors[i]->quick_stop();
    }
    quick_stop_on.store(true);
    _log->warn("Motors Quick Stop");
    return true;
}

bool gravity::Controller::release_quick_stop()
{
    for (int i = 0; i < motors.size(); i++)
    {
        motors[i]->release_quick_stop();
    }
    quick_stop_on.store(false);
    _log->warn("Motors Quick Stop Released");
    return true;
}
