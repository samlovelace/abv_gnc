#ifndef THRUSTERDRIVERFACTORY_HPP
#define THRUSTERDRIVERFACTORY_HPP

#include <memory> 

#include <plog/Log.h>
#include <yaml-cpp/yaml.h>

#include "abv_controller/IThrusterDriver.hpp"
#include "abv_controller/UdpThrusterDriver.h"
#include "abv_controller/I2cThrusterDriver.h"

#if defined(ARCH_ARM)
    #include "abv_controller/GpioThrusterDriver.h"
#endif 

class ThrusterDriverFactory 
{ 
public:
    static std::unique_ptr<IThrusterDriver> create(const std::string& aType, const YAML::Node& aConfig)
    {
        if("UDP" == aType)
        {
            return std::make_unique<UdpThrusterDriver>(aConfig); 
        }
        else if ("I2C" == aType)
        {
            return std::make_unique<I2cThrusterDriver>(aConfig); 
        }
        #if defined(ARCH_ARM)
        else if ("JETGPIO" == aType)
        {
            return std::make_unique<GpioThrusterDriver>(aConfig); 
        }
        #endif 
        else
        {
            LOGE << "Unsupported thruster driver type: " << aType; 
            return nullptr; 
        }
    }

private:
    
    ThrusterDriverFactory();
    ~ThrusterDriverFactory();

};
#endif //THRUSTERDRIVERFACTORY_HPP