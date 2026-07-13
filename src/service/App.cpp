#include "service/App.hpp"

gravity::App::App(const std::string &conf, const std::string &urdf)
    : _log(make_class_logger("App"))
{
    // 1. trajectory configuration
    YAML::Node config = YAML::LoadFile(conf);
    auto params = config["trajectory_conf"].as<gravity::trajectory_params>();
    std::cout << param_str(params) << std::endl;

    // 2. controller
    controller = std::make_shared<gravity::Controller>(params, active_joints_indices, map_pdos);
    controller->setup(false);

    // 3. task manager
    msg::MachineStateInfo machine_info;
    if (controller->fetch_current_state(machine_info))
    {
        task_manager = std::make_unique<planner::routine::TaskManager>(
            params,
            urdf,
            machine_info.joint_state().position(),
            spdlog::level::debug);
    }
    allow_publishing.store(true);
    _log->info("App Initialized");
}

gravity::App::~App()
{
    _log->info("App Removed");
}

std::string gravity::App::name() const { return "Mover"; }

bool gravity::App::current_state(msg::MachineStateInfo &info)
{
    return controller->fetch_current_state(info);
}