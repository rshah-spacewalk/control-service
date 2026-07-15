#include "controller/Controller.hpp"
#include "ethercat/EthStatus.hpp"

bool gravity::Controller::is_op_mode()
{
    return domain_state.wc_state == EC_WC_COMPLETE;
}

bool gravity::Controller::is_running()
{
    for (int i = 0; i < motors.size(); i++)
    {
        auto status = motor_status[i];
        status_word_entity sw = decode_status_word(status);
        if (!sw.servo_running || sw.quick_stop)
        {
            // _log->warn("Motor [{}] not running", i);
            return false;
        }
    }
    return true;
}

bool gravity::Controller::is_faulted()
{
    for (int i = 0; i < motors.size(); i++)
    {
        auto status = motor_status[i];
        status_word_entity sw = decode_status_word(status);
        if (sw.fault)
        {
            _log->warn("Motor [{}] faulted", i);
            return true;
        }
    }
    return false;
}
bool gravity::Controller::is_stopped()
{
    // for (int i = 0; i < motors.size(); i++)
    // {
    //     auto status = motor_status[i];
    //     status_word_entity sw = decode_status_word(status);
    //     if (sw.quick_stop == 0)
    //     {
    //         _log->warn("Motor [{}] stopped", i);
    //         return true;
    //     }
    // }
    return false;
}

bool gravity::Controller::fetch_current_state(msg::MachineStateInfo &machine_info)
{
    msg::JointStateInfo joint_info;
    auto master_state = get_master_state();
    machine_info.al_state(master_state.al_states);

    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    machine_info.ms(ms);

    if (master->is_activated())
    {
        for (int i = 0; i < motors.size(); i++)
        {
            machine_info.motor_state()[i].control_word(motors[i]->control_word->read_pdo());
            machine_info.motor_state()[i].status_word(motors[i]->status_word->read_pdo());
            machine_info.motor_state()[i].error_code(motors[i]->error_code->read_pdo());
            machine_info.motor_state()[i].fp_err(motors[i]->following_error_actual_value->read_pdo());

            auto pos_raw = motors[i]->position_actual_value->read_pdo();
            auto vel_raw = motors[i]->velocity_actual_value->read_pdo();
            auto trq_raw = motors[i]->torque_actual_value->read_pdo();

            machine_info.motor_state()[i].position(pos_raw);
            machine_info.motor_state()[i].velocity(vel_raw);
            machine_info.motor_state()[i].torque(trq_raw);

            machine_info.joint_state().position()[i] = config::gear_pulse_to_rad<int32_t>(pos_raw, active_joints[i]);
            machine_info.joint_state().velocity()[i] = config::gear_pulse_to_rad<int32_t>(vel_raw, active_joints[i]);
            machine_info.joint_state().torque()[i] = config::joint_torque_nm(trq_raw, active_joints[i]);
        }
        return true;
    }
    else if (master->is_requested())
    {
        for (int i = 0; i < motors.size(); i++)
        {
            machine_info.motor_state()[i].control_word(motors[i]->control_word->read_sdo());
            machine_info.motor_state()[i].status_word(motors[i]->status_word->read_sdo());
            machine_info.motor_state()[i].error_code(motors[i]->error_code->read_sdo());
            machine_info.motor_state()[i].fp_err(motors[i]->following_error_actual_value->read_sdo());

            auto pos_raw = motors[i]->position_actual_value->read_sdo();
            auto vel_raw = motors[i]->velocity_actual_value->read_sdo();
            auto trq_raw = motors[i]->torque_actual_value->read_sdo();

            machine_info.motor_state()[i].position(pos_raw);
            machine_info.motor_state()[i].velocity(vel_raw);
            machine_info.motor_state()[i].torque(trq_raw);

            machine_info.joint_state().position()[i] = config::gear_pulse_to_rad<int32_t>(pos_raw, active_joints[i]);
            machine_info.joint_state().velocity()[i] = config::gear_pulse_to_rad<int32_t>(vel_raw, active_joints[i]);
            machine_info.joint_state().torque()[i] = config::joint_torque_nm(trq_raw, active_joints[i]);
        }
        return true;
    }

    return false;
}
