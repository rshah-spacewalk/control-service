#include <iostream>
#include "service/App.hpp"

int main()
{
    try
    {
        const std::string conf = "/home/ravi/libs/gravity_libs/assets/conf/config.yaml";
        const std::string urdf = "/home/ravi/libs/gravity_libs/assets/urdf/kepler_stl.urdf";
        auto app = std::make_unique<gravity::App>(conf, urdf);
        // app->enable(1);
        // std::this_thread::sleep_for(std::chrono::seconds(10));
        // app->disable(1);
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl;
    }
    return 0;
}

// #include <yaml-cpp/yaml.h>
// #include "controller/Controller.hpp"
// #include <gravity/entity/Params.hpp>

// const bool map_pdos = false;
// const std::vector<uint16_t> active_joints_indices = {4};
// YAML::Node config = YAML::LoadFile("/home/ravi/libs/gravity_libs/assets/conf/config.yaml");
// auto params = config["trajectory_conf"].as<gravity::trajectory_params>();

// auto controller = std::make_shared<gravity::Controller>(params, active_joints_indices, map_pdos);
// controller->setup(false);
// // controller->enable();
// std::this_thread::sleep_for(std::chrono::seconds(4));
// // controller->disable();