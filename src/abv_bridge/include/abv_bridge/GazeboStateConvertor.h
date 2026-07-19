#ifndef GAZEBOSTATECONVERTOR_H
#define GAZEBOSTATECONVERTOR_H

#include "abv_msgs/msg/abv_state.hpp"
#include "ptera_msgs/msg/robot_state.hpp"

// Converts Gazebo's ptera_msgs::msg::RobotState (published by ptera_sim's
// PosePublisher plugin) into abv_msgs::msg::AbvState, so abv_simulator can
// consume simulated physics feedback as its native message type without
// depending on ptera_msgs itself.
class GazeboStateConvertor
{
public:
    GazeboStateConvertor(const std::string& anIncomingTopic, const std::string& anOutgoingTopic);
    ~GazeboStateConvertor();

private:
    void convert(const ptera_msgs::msg::RobotState::SharedPtr& aRobotState);

private:
    std::string mIncomingTopic;
    std::string mOutgoingTopic;

};
#endif //GAZEBOSTATECONVERTOR_H
