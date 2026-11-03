#pragma once

#include <QWidget>
#include <QComboBox>
#include <QLabel>
#include <QVariant>

#include <rclcpp/rclcpp.hpp>

#include "autobot_msgs/srv/policy_switch.hpp"

// Lets the operator see which control-policy nodes (autobot_control -> "control",
// autobot_rl-cpp -> "rl") are currently alive and switch which one is active, via
// the abv/policy/switch service hosted by autobot_policy_registry. The active
// policy itself is displayed elsewhere (see ActivePolicyBanner, top-center of
// the main window) - this panel is switching-only plus the ack/nak result.
class PolicySelectorPanel : public QWidget
{
    Q_OBJECT
public:
    explicit PolicySelectorPanel(QWidget* parent = nullptr);

public slots:
    // aData is a QString encoding of PolicyRegistry built by the
    // TopicAdapter converter wired up in main.cpp: "name:alive;name:alive;..."
    // (alive is "1"/"0").
    void onRegistryUpdate(const QVariant& aData);

private:
    void onSwitchClicked();

    QComboBox* mSelector{nullptr};
    QLabel* mStatusLabel{nullptr};

    rclcpp::Client<autobot_msgs::srv::PolicySwitch>::SharedPtr mSwitchClient;
};
