#include "controller/Controller.hpp"

gravity::Controller::Controller()
{
    auto master = std::make_shared<gravity::EthercatMaster>("/dev/EtherCAT0");
    master->request_master();

    std::vector<gravity::MotorBase *> out;
    out.reserve(motors.size());
    for (const auto &m : motors)
    {
        out.push_back(m.get());
    }

    motor_config = std::make_shared<gravity::MotorConfig>(out, enabled, master);

    _log->info("Initialized!");
}

gravity::Controller::~Controller()
{
    master->release_master();
    _log->info("Released!");
}

bool gravity::Controller::setup(bool map_pdos)
{
    return false;
}
