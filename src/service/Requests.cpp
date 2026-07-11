#include "service/App.hpp"

gravity::task_response_t gravity::App::enable(const uint64_t &task_id)
{
    task_response_t resp{};
    resp.id = task_id;

    // 0. check cyclic thread
    if (cyclic_thread.joinable() || cyclic_loop_active.load())
    {
        _log->error("Cyclic phase still active");
        resp.status = task_status_t::ABORTED;
    }

    if (controller->enable())
    {
        //  3. start cyclic thread
        cyclic_loop_active.store(true);
        cyclic_thread = std::thread(&App::cyclic_loop, this);

        struct sched_param sch_params;
        sch_params.sched_priority = 80;
        if (pthread_setschedparam(cyclic_thread.native_handle(), SCHED_FIFO, &sch_params) != 0)
        {
            _log->warn("Failed to set RT priority: {}", std::strerror(errno));
        }
        _log->warn("Cyclic thread started!");
        resp.status = task_status_t::SUCCESS;
    }
    else
    {
        resp.status = task_status_t::ABORTED;
    }

    return resp;
}

gravity::task_response_t gravity::App::disable(const uint64_t &task_id)
{
    task_response_t resp{};
    resp.id = task_id;

    // 1. stop cylic thread
    cyclic_loop_active.store(false);
    if (cyclic_thread.joinable())
    {
        cyclic_thread.join();
        _log->warn("Cylic thread stopped");
    }

    if (controller->disable())
    {
        resp.status = task_status_t::SUCCESS;
    }
    else
    {
        resp.status = task_status_t::ABORTED;
    }
    return resp;
}

gravity::task_response_t gravity::App::e_stop(const uint64_t &task_id)
{
    task_response_t resp{};
    resp.id = task_id;
    if (controller->quick_stop())
    {
        resp.status = task_status_t::SUCCESS;
    }
    else
    {
        resp.status = task_status_t::ABORTED;
    }
    return resp;
}

gravity::task_response_t gravity::App::release_e_stop(const uint64_t &task_id)
{
    task_response_t resp{};
    resp.id = task_id;
    if (controller->release_quick_stop())
    {
        resp.status = task_status_t::SUCCESS;
    }
    else
    {
        resp.status = task_status_t::ABORTED;
    }
    return resp;
}
