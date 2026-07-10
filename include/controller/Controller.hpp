#pragma once

#include <mutex> 
#include "ethercat/Master.hpp"
#include "motor/Motor.hpp"
#include "motor/Config.hpp"
#include "controller/MoverConfig.hpp"

namespace gravity
{
    class Controller
    {
    private:
        bool map_pdos{false};
        uint32_t cycle_overun_count{0};

        trajectory_params params;
        std::shared_ptr<EthercatMaster> master;

        std::array<uint16_t, 6> motor_error{};
        std::array<uint16_t, 6> motor_status{};
        const std::vector<uint16_t> active_joints;
        std::array<int32_t, 6> current_position_pulse{};

        std::shared_ptr<MotorConfig> motor_config;
        std::vector<gravity::MotorBase *> motor_refs;
        std::vector<std::unique_ptr<MotorBase>> motors;

        std::mutex cycle_mtx; 
        std::thread cyclic_thread;
        std::atomic_bool quick_stop_on{false};
        std::atomic_bool cyclic_loop_active{false};

        std::shared_ptr<spdlog::logger> _log;

        bool config_cycle();
        void cyclic_loop();
        void cycle(const std::array<int32_t, 6> &target_pos_pulse);

        bool handle_motor_error(const uint16_t &error, const uint16_t &position);
        bool handle_motor_status(const uint16_t &status, const uint16_t &position);

    public:
        explicit Controller(
            const trajectory_params &_params,
            const std::vector<uint16_t> &_active_joints,
            const bool _map_pdos);

        ~Controller();

        bool setup(const bool strict);
        bool enable();
        bool disable();

        bool quick_stop();
        bool release_quick_stop();
        std::array<int32_t, 6> &fetch_current_pos() { return current_position_pulse; }

        std::atomic_bool allow_publishing{false};
    };
}

// contructor order : top -> bottom
// destructor order : bottom -> top
// find ~/vcpkg/installed/arm64-linux-release -iname "*DomainParticipant*"
