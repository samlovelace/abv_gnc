#ifndef CONTROLLERSTATUSCONVERTOR
#define CONTROLLERSTATUSCONVERTOR
 
#include "autobot_msgs/msg/controller_status.hpp"
 
class ControllerStatusConvertor 
{ 
public:
    ControllerStatusConvertor(const std::string& anIncomingTopic, const std::string& anOutgoingTopic);
    ~ControllerStatusConvertor();

private:
    void convert(autobot_msgs::msg::ControllerStatus::SharedPtr aStatus); 

private: 
    std::string mIncomingTopic; 
    std::string mOutgoingTopic; 
   
};
#endif //CONTROLLERSTATUSCONVERTOR