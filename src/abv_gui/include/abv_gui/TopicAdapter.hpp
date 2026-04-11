
#ifndef TOPICADAPTER_HPP
#define TOPICADAPTER_HPP

#include "abv_common/RosTopicManager.h"

class TopicAdapterBase : public QObject {
    Q_OBJECT
public:
    using QObject::QObject;

signals:
    void newDataVariant(const QVariant& data);
};

template<typename MsgT, typename OutT>
class TopicAdapter : public TopicAdapterBase {
public:
    using ConvertFn = std::function<OutT(const MsgT&)>;

    TopicAdapter(const std::string& topic,
                 ConvertFn converter,
                 QObject* parent = nullptr)
        : TopicAdapterBase(parent),
          mConverter(converter)
    {
        RosTopicManager::getInstance()->createSubscriber<MsgT>(
            topic, 
            std::bind(&TopicAdapter::callback, this, std::placeholders::_1)); 
    }

private: 
    void callback(const typename MsgT::SharedPtr msg) {
        OutT out = mConverter(*msg); 
        QVariant v = QVariant::fromValue(out); 
        emit newDataVariant(v); 
    }

private:
    ConvertFn mConverter;
};

#endif // TOPICADAPTER_HPP