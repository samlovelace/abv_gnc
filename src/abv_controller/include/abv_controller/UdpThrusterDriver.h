#ifndef UDPTHRUSTERDRIVER_H
#define UDPTHRUSTERDRIVER_H

#include <memory> 
#include "abv_common/UdpClient.h"
#include "IThrusterDriver.hpp"

class UdpThrusterDriver : public IThrusterDriver
{
public: 
    UdpThrusterDriver(const YAML::Node& aConfig); 
    ~UdpThrusterDriver(); 

    bool init() override; 
    bool fini() override; 

    bool send(const std::string& aThrusterCommand) override;
private: 

    std::string mServerIp; 
    int mPort; 
    std::unique_ptr<UdpClient> mClient; 
};
#endif 