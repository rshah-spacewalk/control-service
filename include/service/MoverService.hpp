// #pragma once

// #include "controller/Controller.hpp"
// #include <gravity/models/MachineInterface.hpp>
// #include <gravity/task/TaskManager.hpp>

// namespace gravity
// {
//     class MoverService final : public ClientInterface
//     {
//     private:
//         std::shared_ptr<Controller> controller;
//         std::unique_ptr<planner::routine::TaskManager> task_manager;

//     public:
//         explicit MoverService(const std::shared_ptr<Controller> &_controller)
//         {
//             std::array<int32_t, 6> joint_pos = controller->fetch_current_pos();

//             task_manager = std::make_unique<planner::routine::TaskManager>(
//                 params, urdf_obj, joint_state.position(), spdlog::level::debug);
//         }

//         bool is_available() override
//         {
//             return true;
//         }
//         std::string name() const override
//         {
//             return "MoverService";
//         }
//         bool current_joint_state(msg::JointStateInfo &info) override
//         {
//         }
//         task_response_t enable(const uint64_t &task_id) override
//         {
//             controller->enable_controller();
//         }
//         task_response_t disable(const uint64_t &task_id) override
//         {
//             controller->disable_controller();
//         }
//         task_response_t e_stop(const uint64_t &task_id) override
//         {
//             controller->quick_stop();
//         }
//         task_response_t release_e_stop(const uint64_t &task_id) override
//         {
//             controller->release_stop();
//         }
//         task_response_t cancel_task(const uint64_t &task_id) override;
//         task_response_t process_task(const task_request_t &task_request) override;
//         bool has_active_task() const override;
//         void set_task_response_cb(task_response_cb_t cb) override;
//         bool fetch_task_response(task_response_t &out) override;
//     }
// }
