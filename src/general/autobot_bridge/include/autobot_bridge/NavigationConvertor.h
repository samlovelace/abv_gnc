#ifndef NAVIGATIONCONVERTOR_H
#define NAVIGATIONCONVERTOR_H
 
#include "autobot_msgs/msg/state.hpp"
#include "ptera_msgs/msg/robot_state.hpp"
 
class NavigationConvertor 
{ 
public:
    NavigationConvertor(const std::string& anIncomingTopic, const std::string& anOutgoingTopic);
    ~NavigationConvertor();

private: 
    void convert(const autobot_msgs::msg::State::SharedPtr& anState); 

private:
    std::string mIncomingTopic; 
    std::string mOutgoingTopic; 
   
};
#endif //NAVIGATIONCONVERTOR_H  