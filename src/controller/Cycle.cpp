#include "controller/Controller.hpp"
#include "ethercat/ThreadUtil.hpp"

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
    _log->info("Enabling Controller");
    try
    {
        if (master->is_activated())
        {
            _log->error("Master already Activated");
            return false;
        }

        if (config_cycle())
        {
            // 0. check cyclic thread
            if (cyclic_thread.joinable() || cyclic_loop_active.load())
            {
                _log->error("Cyclic phase still active");
                return false;
            }

            // 1. config motor pdos & enable motor cycle
            ThreadGroup init_threads(motors.size());
            for (auto &motor : motors)
            {
                init_threads.threads.emplace_back(
                    [raw_motor = motor.get()]()
                    {
                        raw_motor->enable_cycle();
                    });
            }

            _log->info("Entering cyclic phase with cycle frequency: {} microseconds",
                       config::kepler::PDO_INTERVAL);

            // 2. enable master
            master->activate_master();

            //  3. start cyclic thread
            cyclic_loop_active.store(true);
            cyclic_thread = std::thread(&Controller::cyclic_loop, this);

            struct sched_param sch_params;
            sch_params.sched_priority = 80;
            if (pthread_setschedparam(cyclic_thread.native_handle(), SCHED_FIFO, &sch_params) != 0)
            {
                _log->warn("Failed to set RT priority: {}", std::strerror(errno));
            }
            _log->warn("Cyclic thread started!");
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return false;
}

bool gravity::Controller::disable()
{
    _log->info("Disabling Controller");
    try
    {
        if (master->is_activated())
        {
            // 1. stop cylic thread
            cyclic_loop_active.store(false);

            if (cyclic_thread.joinable())
            {
                if (std::this_thread::get_id() == cyclic_thread.get_id())
                {
                    _log->warn("Disable called from within cyclic thread. Skipping join.");
                    return true;
                }
                else
                {
                    cyclic_thread.join();
                    _log->warn("Cylic thread stopped");
                }
            }

            // 2. disable motor cycle
            ThreadGroup stop_threads(motors.size());
            for (auto &motor : motors)
            {
                stop_threads.threads.emplace_back(
                    [raw_motor = motor.get()]()
                    {
                        raw_motor->disable_cycle();
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
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return false;
}
