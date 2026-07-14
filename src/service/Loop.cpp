#include "service/App.hpp"

void gravity::App::cyclic_loop()
{
    gravity::Clock interval = gravity::Clock::fromMicroseconds(config::PDO_INTERVAL);
    while (cyclic_loop_active.load(std::memory_order_relaxed))
    {
        gravity::Clock start = gravity::Clock::now();
        try
        {
            auto current_pos = controller->cycle(task_manager->get_position());
            if (controller->is_running())
            {
                task_manager->process_task_loop(current_pos);
            }

            if (controller->is_faulted() || controller->is_stopped())
            {
                throw std::runtime_error("Cyclic loop error - controller at fault or stopped");
            }
        }
        catch (const std::exception &e)
        {
            handle_cycle_error(e.what());
            break;
        }

        // sleep for interval
        gravity::Clock elapsed = (gravity::Clock::now() - start);
        if (elapsed > interval)
        {
            auto delayed_us = (elapsed - interval).toMicroseconds();
            _log->warn("Cycle loop overran by {} us", delayed_us);
            cycle_overun_count++;
        }
        else
        {
            gravity::Clock remaining = (interval - elapsed);
            remaining.sleepFor();
        }
    }
}

void gravity::App::handle_cycle_error(const std::string &err)
{
    _log->error(err);
    controller->disable();
}
