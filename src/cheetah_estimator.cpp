
// STL
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <thread>
#include <chrono>
#include <fstream>
#include <string>
#include <memory>
#include <iostream>
#include <chrono>
#include "utils/cheetah_data_t.hpp"
#include "communication/lcm_handler.hpp"
#include "system/cheetah_system.hpp"

// Boost
#include <boost/algorithm/string.hpp>
// Threading
#include <boost/thread/condition.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>

#define LCM_MULTICAST_URL "udpm://239.255.76.67:7667?ttl=2"
using namespace std::chrono;



int main(int argc, char **argv)
{
    while (true) {
        // Initialize LCM
        lcm::LCM lcm(LCM_MULTICAST_URL);
        if (!lcm.good())
        {
            // ROS_ERROR_STREAM("LCM init failed.");
            std::cerr << "LCM init failed" << std::endl;
            return -1;
        }

        // Threading
        boost::mutex cdata_mtx;
        cheetah_lcm_data_t cheetah_input_data;
        bool reinit_cmd = false;
        
        std::cout << "Subscribing to LCM channels" << std::endl;
        cheetah_inekf_lcm::lcm_handler lcm_subscriber_node(&lcm, &cheetah_input_data, &cdata_mtx, &reinit_cmd);
        std::cout << "Subscribed to LCM channels" << std::endl;
        
        // Set noise parameters
        inekf::NoiseParams params;

        // Initialize CheetahSystem
        std::cout << "Initializing Cheetah System" << std::endl;
        CheetahSystem *system = new CheetahSystem(&lcm, &cdata_mtx, &cheetah_input_data);
        // system->setEstimator(std::make_shared<BodyEstimator>());
        std::cout << "Cheetah System is initialized" << std::endl;

        auto start = high_resolution_clock::now();
        auto stop = high_resolution_clock::now();

        while (lcm.handle() == 0)
        {
            // Reinitialize the whole system if receive reinitialize command:

            if (reinit_cmd) {
                break;
            }
            
            system->step();
            stop = high_resolution_clock::now(); 

            auto duration = duration_cast<microseconds>(stop - start);
            std::cout << "Frequency: " << (double) (1000000.0 / duration.count()) << "Hz" << std::endl;
            start = high_resolution_clock::now();
        }

    }
    
    return 0;
}