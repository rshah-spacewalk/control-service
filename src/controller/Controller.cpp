#include "controller/Controller.hpp"

gravity::Controller::Controller(bool _map_pdos)
    : map_pdos(_map_pdos), _log(make_class_logger("Controller"))
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
            throw std::runtime_error("No motor found");
        }
        if (info.slave_count != DOF && strict)
        {
            auto err = fmt::format("Motor count {} does not match dof {}",
                                   info.slave_count, config::kepler::DOF);
            throw std::runtime_error(err);
        }

        // initialize motors
        motors.clear();
        motors.reserve(info.slave_count);
        for (int i = 0; i < info.slave_count; i++)
        {
            motors.push_back(std::make_unique<MotorBase>(master->ec_master_ptr, i));
        }

        // initialize config
        std::vector<gravity::MotorBase *> motor_ref;
        motor_ref.reserve(motors.size());
        for (const auto &m : motors)
        {
            motor_ref.push_back(m.get());
        }
        motor_config = std::make_shared<gravity::MotorConfig>(motor_ref, master);
        motor_config->apply_configs();

        // position setup
        for (int i = 0; i < motors.size(); i++)
        {
            motor_position[i] = motors[i]->position_actual_value->read_sdo();
        }
        _log->info("Current Position: {}", motor_position);
    }

    _log->info("Setup completed");

    return true;
}
