#include "plog/Log.h"

#include "autobot_config/ManipConfigManager.h"

#include "autobot_manip_hardware/ManipulatorFactory.h"
#include "autobot_manip_hardware/RosManipComms.h"
#include "autobot_manip_hardware/DynamixelManipComms.h"
#include "autobot_manip_hardware/SimManipComms.hpp"

std::shared_ptr<IManipComms> ManipulatorFactory::create()
{
    auto config = ManipConfigManager::getInstance()->getConfig();

    if("ros" == config.commsMode)
    {
        LOGW << "Using RosManipComms";
        return std::make_shared<RosManipComms>(config);
    }
    else if ("dynamixel" == config.manipType && "dynamixel" == config.commsMode)
    {
        LOGW << "Using DynamixelManipComms";
        return std::make_shared<DynamixelManipComms>(ManipConfigManager::getInstance()->getRawNode()["comms_configs"],
                                                       static_cast<int>(config.manipulator.jointNames.size()));
    }
    else if("simulated" == config.commsMode)
    {
        // any manip type can use this comms type
        return std::make_shared<SimManipComms>(config.manipulator.jointNames,
                                                 config.gripper ? config.gripper->jointNames : std::vector<std::string>{});
    }
    else
    {
        LOGE << "Manip type: " << config.manipType << " not yet supported";
    }

}
