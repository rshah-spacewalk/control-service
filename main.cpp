#include <iostream>
#include "controller/Controller.hpp"

int main()
{
    try
    {
        auto controller = std::make_unique<gravity::Controller>(false);
        controller->setup(false);
        std::this_thread::sleep_for(std::chrono::seconds(4));
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl;
    }
    return 0;
}