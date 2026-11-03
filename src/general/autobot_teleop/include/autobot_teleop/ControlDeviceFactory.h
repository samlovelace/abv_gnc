
#include "IControlDevice.h"
#include "SfmlControlDevice.h"
#include <memory>
#include <cstring> 

namespace ControlDeviceFactory
{
    std::shared_ptr<IControlDevice> create(const std::string& aDeviceType)
    {
        if(strcmp("sfml", aDeviceType.c_str()) == 0)
        {
            return std::make_shared<SfmlControlDevice>();
        }
        else{
            return nullptr; 
        }
    }
}