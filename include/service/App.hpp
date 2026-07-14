#pragma once

#include "controller/Controller.hpp"
#include <gravity/models/MachineInterface.hpp>
#include <gravity/task/TaskManager.hpp>

// handle all exceptions in app class only

namespace gravity
{
    class App final : public ClientInterface
    {
    private:
        // controller
        const bool map_pdos = false;
        std::shared_ptr<Controller> controller;
        const std::vector<uint16_t> active_joints_indices = {4};

        std::thread cyclic_thread;
        uint32_t cycle_overun_count{0};
        std::atomic_bool cyclic_loop_active{false};
        std::atomic_bool allow_publishing{false};

        // task manager
        const trajectory_params params;
        task_response_cb_t on_task_response;
        std::unique_ptr<planner::routine::TaskManager> task_manager;
        std::shared_ptr<spdlog::logger> _log;

        // cycle
        void cyclic_loop();
        void handle_cycle_error(const std::string &err);

    public:
        explicit App(const std::string &conf, const std::string &urdf);

        ~App();

        std::string name() const override;

        bool current_state(msg::MachineStateInfo &info) override;

        task_response_t enable(const uint64_t &task_id) override;

        task_response_t disable(const uint64_t &task_id) override;

        task_response_t e_stop(const uint64_t &task_id) override;

        task_response_t release_e_stop(const uint64_t &task_id) override;

        task_response_t cancel_task(const uint64_t &task_id) override;

        task_response_t process_task(const task_request_t &task_request) override;

        bool has_active_task() const override;

        void set_task_response_cb(task_response_cb_t cb) override;

        bool fetch_task_response(task_response_t &out) override;
    };
}
