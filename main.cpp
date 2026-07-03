#include <iostream>
#include "ethercat/Master.hpp"

int main()
{
    try
    {
        auto master = std::make_shared<gravity::EthercatMaster>("/dev/EtherCAT0");
        master->request_master();
        std::cout << "Initialized: " << master->is_requested() << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(4));
        master->release_master();
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl;
    }
    return 0;
}