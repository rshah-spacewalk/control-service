#include "controller/Controller.hpp"
#include "Controller.hpp"

bool gravity::Controller::quick_stop()
{
    for (int i = 0; i < motors.size(); i++)
    {
        motors[i]->quick_stop();
    }
    quick_stop_on.store(true);
    _log->warn("Motors Quick Stop");
    return true;
}

bool gravity::Controller::release_quick_stop()
{
    for (int i = 0; i < motors.size(); i++)
    {
        motors[i]->quick_stop_to_running();
    }
    quick_stop_on.store(false);
    _log->warn("Motors Quick Stop Released");
    return true;
}

