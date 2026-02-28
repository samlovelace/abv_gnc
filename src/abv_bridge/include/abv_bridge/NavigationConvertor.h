#ifndef NAVIGATIONCONVERTOR_H
#define NAVIGATIONCONVERTOR_H
 
#include "abv_msgs/msg/abv_state.hpp"
#include "robot_idl/msg/robot_state.hpp"
 
class NavigationConvertor 
{ 
public:
    NavigationConvertor(const std::string& anIncomingTopic, const std::string& anOutgoingTopic);
    ~NavigationConvertor();

private: 
    void convert(const abv_msgs::msg::AbvState::SharedPtr& anAbvState); 

private:
    std::string mIncomingTopic; 
    std::string mOutgoingTopic; 
   
};
#endif //NAVIGATIONCONVERTOR_H  