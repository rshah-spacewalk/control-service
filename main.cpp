#include <iostream>
#include <yaml-cpp/yaml.h>
#include "controller/Controller.hpp"
#include <gravity/entity/Params.hpp>

int main()
{
    try
    {
        YAML::Node config = YAML::LoadFile("/home/ravi/libs/gravity_libs/assets/conf/config.yaml");
        auto params = config["trajectory_conf"].as<gravity::trajectory_params>();

        auto controller = std::make_unique<gravity::Controller>(params, false);
        controller->setup(false);
        // controller->enable();
        std::this_thread::sleep_for(std::chrono::seconds(4));
        // controller->disable();
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl;
    }
    return 0;
}