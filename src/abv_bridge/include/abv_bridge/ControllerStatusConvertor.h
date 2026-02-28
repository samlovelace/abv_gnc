#ifndef CONTROLLERSTATUSCONVERTOR
#define CONTROLLERSTATUSCONVERTOR
 
#include "abv_msgs/msg/abv_controller_status.hpp"
 
class ControllerStatusConvertor 
{ 
public:
    ControllerStatusConvertor(const std::string& anIncomingTopic, const std::string& anOutgoingTopic);
    ~ControllerStatusConvertor();

private:
    void convert(abv_msgs::msg::AbvControllerStatus::SharedPtr aStatus); 

private: 
    std::string mIncomingTopic; 
    std::string mOutgoingTopic; 
   
};
#endif //CONTROLLERSTATUSCONVERTOR