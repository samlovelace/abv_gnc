#ifndef ROSTOPICMANAGER_H
#define ROSTOPICMANAGER_H

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <atomic>

class RosTopicManager : public rclcpp::Node
{

public:

    static RosTopicManager* getInstance(const std::string& aNodeName = "Default")
    {
        static RosTopicManager instance(aNodeName);
        return &instance;
    }

    void init();
    void spinNode();
    bool isROSInitialized() { return rclcpp::ok(); }

    rclcpp::PublisherBase::SharedPtr getPublisher(const std::string& aTopicName)
    {
        if(mPublishers.find(aTopicName) != mPublishers.end())
        {
            return mPublishers[aTopicName];
        }
        else {
            // 
        }
    }

    template<typename T>
    void createPublisher(const std::string& topicName) 
    {
        auto publisher = this->create_publisher<T>(topicName, 10);
        mPublishers[topicName] = std::dynamic_pointer_cast<rclcpp::PublisherBase>(publisher);
    }

    template<typename T>
    void publishMessage(const std::string& topicName, const T& message) 
    {
        auto it = mPublishers.find(topicName);
        
        if (it != mPublishers.end()) 
        {
            // Cast PublisherBase back to Publisher<T>
            auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<T>>(it->second);
            if (pub) 
            {
                pub->publish(message);
            } 
            else 
            {
                if(rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to cast publisher for topic: %s", topicName.c_str());
                }
            }
        } 
        else 
        {
            RCLCPP_ERROR(this->get_logger(), "Publisher not found for topic: %s", topicName.c_str());
        }
    }

    template<typename T>
    void createSubscriber(const std::string& aTopicName, std::function<void(const typename T::SharedPtr)> aCallback)
    {
        auto subscriber = this->create_subscription<T>(aTopicName, 10, aCallback);

        // mSubscribers holds every subscription for a topic, not just the
        // latest one - a plain map<string, SharedPtr> here would let a
        // second createSubscriber() call for the same topic silently
        // overwrite (and thus destroy/unsubscribe) an earlier one, since
        // rclcpp::Subscription lifetime is refcounted via this shared_ptr
        // and nothing else keeps it alive.
        mSubscribers[aTopicName].push_back(std::dynamic_pointer_cast<rclcpp::SubscriptionBase>(subscriber));
    }

private:

    RosTopicManager(const std::string& aNodeName);
    ~RosTopicManager();

    std::map<std::string, rclcpp::PublisherBase::SharedPtr> mPublishers;
    std::map<std::string, std::vector<rclcpp::SubscriptionBase::SharedPtr>> mSubscribers;
    std::atomic<bool> mSpinning{false};
};

#endif