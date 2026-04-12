#pragma once
#include <QWidget>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QStackedWidget>
#include <rclcpp/rclcpp.hpp>

class CommandPanel : public QWidget
{
    Q_OBJECT
public:
    explicit CommandPanel(QWidget* parent = nullptr);

private:
    QWidget*        makePosePanel();
    QWidget*        makeVelocityPanel();
    QWidget*        makePathPanel();

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
};