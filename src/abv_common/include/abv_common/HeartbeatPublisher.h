#ifndef HEARTBEATPUBLISHER_H
#define HEARTBEATPUBLISHER_H

#include <string>
#include <rclcpp/rclcpp.hpp>

// Publishes an AbvHeartbeat{node_name} on the shared "abv/heartbeat" topic
// at the rate configured in Heartbeat.Rate, for as long as this object is
// kept alive. Construct one of these in a node's main() before the
// blocking run()/spin call so it lives for the process's lifetime.
class HeartbeatPublisher
{
public:
    explicit HeartbeatPublisher(const std::string& aNodeName);
    ~HeartbeatPublisher();

private:
    void tick();

    std::string mNodeName;
    rclcpp::TimerBase::SharedPtr mTimer;
};
#endif //HEARTBEATPUBLISHER_H
