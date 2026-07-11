#include "service/App.hpp"

gravity::task_response_t gravity::App::cancel_task(const uint64_t &task_id)
{
    task_response_t resp;
    resp.id = task_id;
    resp.status = task_status_t::SUCCESS;
    return resp;
}

gravity::task_response_t gravity::App::process_task(const task_request_t &task_request)
{
    task_response_t resp;
    if (task_manager->add_task(task_request, resp))
    {
        _log->info("[{}] Processing Task", task_request.id);
    }
    if (resp.status != task_status_t::IDLE)
    {
        const std::string msg = "Task Request Error : " +
                                gravity::util::tf::enum_str<task_exception_t>(resp.exception);
        _log->error(msg);
    }
    return resp;
}

bool gravity::App::has_active_task() const
{
    return task_manager->has_active_task() || task_manager->task_response_queue_size() > 0;
}

void gravity::App::set_task_response_cb(task_response_cb_t cb)
{
    on_task_response = std::move(cb);
}

bool gravity::App::fetch_task_response(task_response_t &out)
{
    return task_manager->fetch_task_response(out);
}
