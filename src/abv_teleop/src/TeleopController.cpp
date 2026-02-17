
#include "TeleopController.h"
#include <iostream>
#include "RosTopicManager.h"

TeleopController::TeleopController(std::shared_ptr<IControlDevice> aControlDevice) : mControlDevice(aControlDevice)
{

}

TeleopController::~TeleopController()
{
    // do nothing rn
}

void TeleopController::run()
{
    mCommandPublishThread = std::thread(&TeleopController::commandPublishLoop, this); 

    // TODO: put this in a config file 
    // Desired frequency in Hz
    const double frequency = 2.0;
    // Calculate the loop duration
    const std::chrono::duration<double> loop_duration(1.0 / frequency);
    
    while(true)
    {
        auto start = std::chrono::high_resolution_clock::now(); 
        mControlDevice->handleInput(); 

        // Calculate the time taken for the loop iteration
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = end - start;

        // Sleep for the remaining time to maintain the frequency
        if (elapsed < loop_duration) {
            std::this_thread::sleep_for(loop_duration - elapsed);
        } else {
            std::cerr << "Loop overrun! Elapsed time: " 
                        << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()
                        << " ms\n";
        }
    }
     
}

void TeleopController::commandPublishLoop()
{
    // TODO: put this in a config file 
    // Desired frequency in Hz
    const double frequency = 25.0;
    const std::chrono::duration<double> loop_duration(1.0 / frequency);

    auto commsHandler = RosTopicManager::getInstance(); 
    
    while(true)
    {
        auto start = std::chrono::high_resolution_clock::now(); 
        auto data = mControlDevice->getCommand(); 

        commsHandler->publishMessage<abv_msgs::msg::AbvCommand>("abv/command", CommsUtils::commandMsgToIdl(data)); 

        // Calculate the time taken for the loop iteration
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = end - start;

        // Sleep for the remaining time to maintain the frequency
        if (elapsed < loop_duration) {
            std::this_thread::sleep_for(loop_duration - elapsed);
        } else {
            std::cerr << "Loop overrun! Elapsed time: " 
                        << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()
                        << " ms\n";
        }
    }
}

