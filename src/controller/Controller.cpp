#include "controller/Controller.hpp"

gravity::Controller::Controller(
    const trajectory_params &_params,
    const std::vector<uint16_t> &_active_joints,
    const bool _map_pdos)
    : params(_params),
      active_joints(_active_joints),
      map_pdos(_map_pdos),
      _log(make_class_logger("Controller"))
{
    master = std::make_shared<gravity::EthercatMaster>("/dev/EtherCAT0");
    master->request_master();
    _log->info("Initialized!");
}

gravity::Controller::~Controller()
{
    _log->info("Released!");
}

bool gravity::Controller::setup(bool strict)
{
    _log->info("Setting up!");

    ec_master_info_t info;
    if (master->get_ec_master_info(info))
    {
        _log->info("Motors available {}", info.slave_count);

        if (info.slave_count == 0)
        {
            throw std::runtime_error("Controller Error: No motor found");
        }
        if (info.slave_count != config::DOF && strict)
        {
            auto err = fmt::format("Motor count {} does not match dof {}",
                                   info.slave_count, config::DOF);
            std::cout << err << std::endl;
            throw std::runtime_error(err);
        }

        // initialize motors & config
        if (active_joints.size() != info.slave_count)
        {
            auto err = fmt::format("Motor count {} does not match active joints count {}",
                                   info.slave_count, active_joints.size());
            std::cout << err << std::endl;
            throw std::runtime_error(err);
        }

        motors.clear();
        motor_refs.clear();

        motors.reserve(info.slave_count);
        motor_refs.reserve(info.slave_count);

        for (int i = 0; i < info.slave_count; i++)
        {
            motors.emplace_back(std::make_unique<MotorBase>(master->ec_master_ptr, i, active_joints[i]));
            motor_refs.push_back(motors[i].get());
        }

        motor_config = std::make_shared<gravity::MotorConfig>(motor_refs, params, master);
        motor_config->apply_configs();
        motor_config->read_configs();

        // position setup
        for (int i = 0; i < motors.size(); i++)
        {
            current_position_pulse[i] = motors[i]->position_actual_value->read_sdo();
            handle_motor_status(motors[i]->status_word->read_sdo(), i);
            handle_motor_error(motors[i]->error_code->read_sdo(), i);
        }
        _log->info("Current Position: {}", current_position_pulse);
    }

    _log->info("Setup completed");
    allow_publishing.store(true);

    return true;
}
