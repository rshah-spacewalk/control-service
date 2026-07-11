#include "controller/Controller.hpp"

std::array<double, 6> gravity::Controller::cycle(const std::array<double, 6> &target_pos_rad)
{
    // 0. lock
    std::scoped_lock _lock(cycle_mtx);

    // 2. receive data
    ecrt_master_receive(master->ec_master_ptr);
    ecrt_domain_process(master->ec_domain_ptr);

    // 3. read data
    std::array<double, 6> current_pos_rad{};

    for (int i = 0; i < motors.size(); i++)
    {
        // 3.1 handle events
        handle_motor_error(motors[i]->error_code->read_pdo(), i);
        handle_motor_status(motors[i]->status_word->read_pdo(), i);

        // 3.2 read position
        current_position_pulse[i] = motors[i]->position_actual_value->read_pdo();
        current_pos_rad[i] = config::gear_pulse_to_rad<int32_t>(current_position_pulse[i], active_joints[i]);
    }

    // 5. write data
    for (int i = 0; i < motors.size(); i++)
    {
        int32_t target_joint_pos = config::rad_to_gear_pulse<int32_t>(target_pos_rad[i], active_joints[i]);
        motors[i]->target_position->write_pdo(target_joint_pos);
    }

    // 6. motor pdo cycle
    for (auto &motor : motors)
    {
        motor->cycle(domain_pdm);
    }

    // 7. send data
    ecrt_domain_queue(master->ec_domain_ptr);
    ecrt_master_send(master->ec_master_ptr);

    // 8. return current position
    return current_pos_rad;
}
