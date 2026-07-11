#include "service/App.hpp"

void gravity::App::cyclic_loop()
{
    gravity::Clock interval = gravity::Clock::fromMicroseconds(config::PDO_INTERVAL);
    while (cyclic_loop_active.load(std::memory_order_relaxed))
    {
        gravity::Clock start = gravity::Clock::now();
        try
        {
            if (!controller->is_faulted() && !controller->is_stopped())
            {
                auto current_pos = controller->cycle(task_manager->get_position());
                task_manager->process_task_loop(current_pos);
            }
            else
            {
                _log->error("Controller at fault or stopped!");
            }
        }
        catch (const std::exception &e)
        {
            _log->error("Cyclic loop exception: {}", e.what());
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
