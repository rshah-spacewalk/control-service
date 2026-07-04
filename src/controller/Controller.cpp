#include "controller/Controller.hpp"

gravity::Controller::Controller()
{
    auto master = std::make_shared<gravity::EthercatMaster>("/dev/EtherCAT0");
    motor_config = std::make_shared<gravity::MotorConfig>(master, motors, enabled);
    master->request_master();
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
