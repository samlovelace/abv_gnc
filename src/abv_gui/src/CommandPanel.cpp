#include "abv_gui/CommandPanel.h"
#include <QVBoxLayout>
#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QStackedWidget>
#include <QFrame>

#include "abv_common/RosTopicManager.h"
#include "abv_msgs/msg/abv_controller_command.hpp"
#include "abv_msgs/msg/abv_guidance_command.hpp"

#include "abv_gui/ButtonAdapter.hpp"

static QDoubleSpinBox* makeSpinBox(double min, double max, double step = 0.05)
{
    auto* sb = new QDoubleSpinBox();
    sb->setRange(min, max);
    sb->setSingleStep(step);
    sb->setDecimals(3);
    sb->setValue(0.0);
    return sb;
}

CommandPanel::CommandPanel(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(4, 4, 4, 4);
    layout->setSpacing(8);

    // Separator
    auto* line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    layout->addWidget(line);
    
    // Command type selector
    auto* typeLabel = new QLabel("Command");
    auto* selector  = new QComboBox();
    selector->addItems({"Pose", "Velocity", "Path"});
    layout->addWidget(typeLabel);
    layout->addWidget(selector);

    // Stacked panels
    auto* stack = new QStackedWidget();
    stack->addWidget(makePosePanel());
    stack->addWidget(makeVelocityPanel());
    stack->addWidget(makePathPanel()); 
    layout->addWidget(stack);

    connect(selector, &QComboBox::currentIndexChanged,
            stack,    &QStackedWidget::setCurrentIndex);
}

QWidget* CommandPanel::makePosePanel()
{
    auto* panel  = new QWidget();
    auto* layout = new QVBoxLayout(panel);
    auto* form   = new QFormLayout();

    mPoseX = makeSpinBox(-100.0, 100.0);
    mPoseY = makeSpinBox(-100.0, 100.0);
    mPoseYaw = makeSpinBox(-100.0, 100.0);

    form->addRow("X:", mPoseX);
    form->addRow("Y:", mPoseY);
    form->addRow("Yaw:", mPoseYaw);

    auto* sendBtn = new ButtonAdapter("Send Pose", 
            std::bind(&CommandPanel::onSendPose, this), 
            ButtonStyle::primary());
    sendBtn->button()->setMinimumHeight(50); 

    layout->addLayout(form);
    layout->addWidget(sendBtn);
    layout->addStretch(); 
    return panel;
}

QWidget* CommandPanel::makeVelocityPanel()
{
    auto* panel  = new QWidget();
    auto* layout = new QVBoxLayout(panel);
    auto* form   = new QFormLayout();

    mVelX = makeSpinBox(-10.0, 10.0);
    mVelY = makeSpinBox(-10.0, 10.0);
    mVelYaw = makeSpinBox(-10.0, 10.0);

    form->addRow("X:", mVelX);
    form->addRow("Y:", mVelY);
    form->addRow("Yaw:", mVelYaw);

    auto* sendBtn = new ButtonAdapter("Send Velocity", 
            std::bind(&CommandPanel::onSendVelocity, this), 
            ButtonStyle::primary());
    sendBtn->button()->setMinimumHeight(50); 

    layout->addLayout(form);
    layout->addWidget(sendBtn);
    layout->addStretch(); 
    return panel;
}

QWidget* CommandPanel::makePathPanel()
{
    auto* panel  = new QWidget();
    auto* layout = new QVBoxLayout(panel);
    auto* form   = new QFormLayout();

    mPathType = new QComboBox();
    mPathType->addItems({"File"});
    form->addRow("Path Type:", mPathType);

    mPathDuration = makeSpinBox(0, 1000.0, 1);
    form->addRow("Duration (s):", mPathDuration); 

    auto* sendBtn = new ButtonAdapter("Send Path", 
            std::bind(&CommandPanel::onSendPath, this), 
            ButtonStyle::primary());
    sendBtn->button()->setMinimumHeight(50); 

    layout->addLayout(form);
    layout->addWidget(sendBtn);
    layout->addStretch(); 
    return panel;
}

void CommandPanel::onSendPose()
{
    abv_msgs::msg::AbvVec3 pose; 
    pose.set__x(mPoseX->value()); 
    pose.set__y(mPoseY->value()); 
    pose.set__yaw(mPoseYaw->value()); 

    abv_msgs::msg::AbvControllerCommand cmd; 
    cmd.set__type("pose");
    cmd.set__data(pose); 
    
    RosTopicManager::getInstance()->publishMessage("abv/controller/command", cmd); 
}

void CommandPanel::onSendVelocity()
{
    abv_msgs::msg::AbvVec3 vel; 
    vel.set__x(mVelX->value()); 
    vel.set__y(mVelY->value()); 
    vel.set__yaw(mVelYaw->value()); 

    abv_msgs::msg::AbvControllerCommand cmd; 
    cmd.set__type("vel");
    cmd.set__data(vel); 
    
    RosTopicManager::getInstance()->publishMessage("abv/controller/command", cmd);
}

void CommandPanel::onSendPath()
{
    abv_msgs::msg::AbvGuidanceCommand cmd; 
    cmd.set__type(mPathType->currentText().toStdString()); 
    cmd.set__duration(mPathDuration->value()); 

    RosTopicManager::getInstance()->publishMessage("abv/guidance/command", cmd); 
}