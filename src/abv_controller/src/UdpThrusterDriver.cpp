
#include "abv_controller/UdpThrusterDriver.h"

UdpThrusterDriver::UdpThrusterDriver(const YAML::Node& aConfig)
{       
    mServerIp = aConfig["ip"].as<std::string>(); 
    mPort = aConfig["port"].as<int>(); 
}

UdpThrusterDriver::~UdpThrusterDriver()
{

}

bool UdpThrusterDriver::init()
{
    mClient = std::make_unique<UdpClient>(mServerIp, mPort);
    
    if(nullptr == mClient)
    {
        return false; 
    } 

    return true; 
}

bool UdpThrusterDriver::fini()
{
    return true; 
}

bool UdpThrusterDriver::send(const std::string& aThrusterCommand)
{
    return mClient->send(aThrusterCommand); 
}