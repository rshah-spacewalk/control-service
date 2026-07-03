#include <thread>
#include <vector>

namespace gravity
{

    struct ThreadGroup
    {
        ThreadGroup(int max)
        {
            threads.reserve(max);
        }
        std::vector<std::thread> threads;
        ~ThreadGroup()
        {
            for (auto &t : threads)
                if (t.joinable())
                    t.join();
        }
    };
} // namespace gravity
