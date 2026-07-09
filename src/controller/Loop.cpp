#include "controller/Controller.hpp"

void gravity::Controller::cyclic_loop()
{
    uint8_t *domain_pdm = ecrt_domain_data(master->ec_domain_ptr);
    gravity::Clock interval = gravity::Clock::fromMicroseconds(config::kepler::PDO_INTERVAL);

    if (!domain_pdm)
    {
        _log->error("Failed to get domain process data pointer.");
        return;
    }

    //
    while (cyclic_loop_active.load(std::memory_order_relaxed))
    {
        // execute task
        gravity::Clock start = gravity::Clock::now();
        try
        {
            ecrt_master_receive(master->ec_master_ptr);
            ecrt_domain_process(master->ec_domain_ptr);

            for (int i = 0; i < motors.size(); i++)
            {
                handle_motor_error(motors[i]->error_code->read_pdo(), i);
                handle_motor_status(motors[i]->status_word->read_pdo(), i); 
                // use rounding for double to int32_t 

                //     // write gear pulses
                //     // double motor_pos_pulse = gravity::config::kepler::rad_to_gear_pulse(task_manager->get_position()[i], i);
                //     // motors[i]->target_position->write_pdo(motor_pos_pulse);

                //     motor_position[i] = motors[i]->position_actual_value->read_pdo();
                // }

                // if (!quick_stop_on.load(std::memory_order_relaxed))
                // {
                //     // // read radians
                //     // gravity::routine::gear_pulse_to_radians(motor_position);
                //     // task_manager->process_task_loop(motor_current_positions);

                motors[i]->target_position->write_pdo(motor_position[i]);
            }

            for (auto &motor : motors)
            {
                motor->cycle(domain_pdm);
            }
            ecrt_domain_queue(master->ec_domain_ptr);
            ecrt_master_send(master->ec_master_ptr);
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
            _log->warn("PDO Cycle loop overran by {} us", delayed_us);
            cycle_overun_count++;
        }
        else
        {
            gravity::Clock remaining = (interval - elapsed);
            remaining.sleepFor();
        }
    }
}
