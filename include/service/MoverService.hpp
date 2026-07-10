#pragma once

#include "controller/Controller.hpp"
#include <gravity/models/MachineInterface.hpp>
#include <gravity/task/TaskManager.hpp>

namespace gravity
{
    class MoverService final : public ClientInterface
    {
    private:
        const bool map_pdos = false;
        const std::vector<uint16_t> active_joints_indices = {4};

        msg::JointStateInfo joint_info;
        msg::MachineStateInfo machine_info;

        std::shared_ptr<Controller> controller;
        std::unique_ptr<planner::routine::TaskManager> task_manager;

    public:
        explicit MoverService(const std::string &conf, const std::string &urdf_obj)
        {

            YAML::Node config = YAML::LoadFile(conf);
            auto params = config["trajectory_conf"].as<gravity::trajectory_params>();

            auto controller = std::make_shared<gravity::Controller>(params, active_joints_indices, map_pdos);
            controller->setup(false);
            // controller->enable();
            // std::this_thread::sleep_for(std::chrono::seconds(4));
            // controller->disable();

            if (controller->fetch_current_state(machine_info, joint_info))
            {
                task_manager = std::make_unique<planner::routine::TaskManager>(
                    params, urdf_obj, joint_info.position(), spdlog::level::debug);
            }
        }

        bool is_available() override
        {
            return true;
        }
        std::string name() const override
        {
            return "MoverService";
        }
        bool current_joint_state(msg::JointStateInfo &info) override
        {
            info = joint_info;
            return true;
        }
        task_response_t enable(const uint64_t &task_id) override
        {
            controller->enable();
        }
        task_response_t disable(const uint64_t &task_id) override
        {
            controller->disable();
        }
        task_response_t e_stop(const uint64_t &task_id) override
        {
            controller->quick_stop();
        }
        task_response_t release_e_stop(const uint64_t &task_id) override
        {
            controller->release_quick_stop();
        }
        task_response_t cancel_task(const uint64_t &task_id) override;

        task_response_t process_task(const task_request_t &task_request) override;

        bool has_active_task() const override;

        void set_task_response_cb(task_response_cb_t cb) override;

        bool fetch_task_response(task_response_t &out) override;
    }
}
