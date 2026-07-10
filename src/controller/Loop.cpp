#include "controller/Controller.hpp"

void gravity::Controller::cyclic_loop()
{

    gravity::Clock interval = gravity::Clock::fromMicroseconds(config::PDO_INTERVAL);
    while (cyclic_loop_active.load(std::memory_order_relaxed))
    {
        // execute task
        gravity::Clock start = gravity::Clock::now();
        try
        {
            cycle(current_position_pulse);
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

void gravity::Controller::cycle(const std::array<int32_t, 6> &target_position_pulse)
{
    // 0. lock
    std::scoped_lock _lock(cycle_mtx);

    // 1. get domain pointer
    uint8_t *domain_pdm = ecrt_domain_data(master->ec_domain_ptr);
    if (!domain_pdm)
    {
        _log->error("Failed to get domain process data pointer.");
        return;
    }

    // 2. receive data
    ecrt_master_receive(master->ec_master_ptr);
    ecrt_domain_process(master->ec_domain_ptr);

    // 3. read data
    for (int i = 0; i < motors.size(); i++)
    {
        // 3.1 handle events
        handle_motor_error(motors[i]->error_code->read_pdo(), i);
        handle_motor_status(motors[i]->status_word->read_pdo(), i);

        // 3.2 read position
        current_position_pulse[i] = motors[i]->position_actual_value->read_pdo();
    }

    // 4. write data
    for (int i = 0; i < motors.size(); i++)
    {
        if (!quick_stop_on.load(std::memory_order_relaxed))
        {
            motors[i]->target_position->write_pdo(target_position_pulse[i]);
        }
    }

    // 5. pdo cycle
    for (auto &motor : motors)
    {
        motor->cycle(domain_pdm);
    }

    // 6. send data
    ecrt_domain_queue(master->ec_domain_ptr);
    ecrt_master_send(master->ec_master_ptr);
}
