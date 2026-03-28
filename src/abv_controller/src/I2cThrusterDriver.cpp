
#include <bitset>
#include <string>
#include <algorithm>

#include "abv_controller/I2cThrusterDriver.h"


I2cThrusterDriver::I2cThrusterDriver(const YAML::Node& aConfig) : mDriver(nullptr)
{
    mDevice = aConfig["device"].as<std::string>(); 
}

I2cThrusterDriver::~I2cThrusterDriver()
{

}

bool I2cThrusterDriver::init()
{
    mDriver = std::make_unique<PCA9685::PCA9685>(mDevice); 
    
    if(nullptr == mDriver)
    {
        return false; 
    } 

    return true; 
}

bool I2cThrusterDriver::send(const std::string& aThrusterCommand)
{
    std::string reversed = aThrusterCommand; 
    std::reverse(reversed.begin(), reversed.end());
    std::bitset<8> bits(reversed);

    for(int i = 0; i < bits.size(); i++) 
    {
        mDriver->set_channel(i, bits[i]); 
    }
    
    return true; 
}

bool I2cThrusterDriver::fini()
{

}

