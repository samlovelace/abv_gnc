#pragma once
#include <QWidget>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QStackedWidget>
#include <rclcpp/rclcpp.hpp>

#include "abv_msgs/msg/abv_controller_command.hpp"

class CommandPanel : public QWidget
{
    Q_OBJECT
public:
    explicit CommandPanel(QWidget* parent = nullptr);

private:
    QWidget* makePosePanel();
    QWidget* makeVelocityPanel();
    QWidget* makePathPanel();
    QWidget* makeThrusterPanel();  

    QWidget* makeThrusterButton(const std::string& name, int number);
    QWidget* makeAxisButton(const std::string& name, int axis);
    abv_msgs::msg::AbvControllerCommand makeCommand(int axis);
    abv_msgs::msg::AbvControllerCommand makeThruster(int thruster);


    void onSendPose();
    void onSendVelocity();
    void onSendPath();

    QDoubleSpinBox* mPoseX{nullptr};
    QDoubleSpinBox* mPoseY{nullptr};
    QDoubleSpinBox* mPoseYaw{nullptr};

    QDoubleSpinBox* mVelX{nullptr};
    QDoubleSpinBox* mVelY{nullptr};
    QDoubleSpinBox* mVelYaw{nullptr};

    QComboBox*      mPathType{nullptr};
    QDoubleSpinBox* mPathDuration{nullptr};

    rclcpp::TimerBase::SharedPtr mFireTimer{nullptr};
};