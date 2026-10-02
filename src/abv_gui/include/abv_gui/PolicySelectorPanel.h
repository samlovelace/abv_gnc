#pragma once

#include <QWidget>
#include <QComboBox>
#include <QLabel>
#include <QVariant>

#include <rclcpp/rclcpp.hpp>

#include "abv_msgs/srv/abv_policy_switch.hpp"

// Lets the operator see which control-policy nodes (abv_control -> "control",
// abv_rl-cpp -> "rl") are currently alive and switch which one is active, via
// the abv/policy/switch service hosted by abv_policy_registry.
class PolicySelectorPanel : public QWidget
{
    Q_OBJECT
public:
    explicit PolicySelectorPanel(QWidget* parent = nullptr);

public slots:
    // aData is a QString encoding of AbvPolicyRegistry built by the
    // TopicAdapter converter wired up in main.cpp: "name:alive;name:alive;..."
    // (alive is "1"/"0").
    void onRegistryUpdate(const QVariant& aData);

    // aData is the active policy name (empty string = none active)
    void onActivePolicyChanged(const QVariant& aData);

private:
    void onSwitchClicked();

    QComboBox* mSelector{nullptr};
    QLabel* mActiveLabel{nullptr};
    QLabel* mStatusLabel{nullptr};

    rclcpp::Client<abv_msgs::srv::AbvPolicySwitch>::SharedPtr mSwitchClient;
};
