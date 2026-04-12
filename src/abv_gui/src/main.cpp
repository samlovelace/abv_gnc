#include <QApplication>
#include <QMainWindow>
#include <QLineSeries>
#include <QChart>
#include <QChartView>
#include <QTimer>
#include <QPalette>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QGroupBox>
#include <QDebug>
#include <cmath>

#include "abv_msgs/msg/abv_controller_command.hpp"
#include "abv_msgs/msg/abv_guidance_command.hpp"

#include "abv_gui/LivePlot.h"
#include "abv_gui/TopicAdapter.hpp"
#include "abv_gui/TopicConversions.hpp"

#include "abv_gui/ButtonActions.hpp"
#include "abv_gui/ButtonAdapter.hpp"

#include "abv_gui/CommandPanel.h"

#include "abv_gui/StatusIndicator.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(0, nullptr); 
    RosTopicManager::getInstance("abv_gui")->spinNode();
    RosTopicManager::getInstance()->createPublisher<abv_msgs::msg::AbvControllerCommand>("abv/controller/command"); 
    RosTopicManager::getInstance()->createPublisher<abv_msgs::msg::AbvGuidanceCommand>("abv/guidance/command"); 

    QApplication app(argc, argv);
    app.setStyle("Fusion"); 

    // --- Create plots ---
    auto *posPlot  = new LivePlot("Position",       0.0,  2.0,  {"x", "y", "yaw"});
    auto *velPlot  = new LivePlot("Velocity",      -2.0,  2.0,  {"vx", "vy", "w"});
    auto *ctrlPlot = new LivePlot("Control Input", -10.0, 10.0, {"fx", "fy", "tz"});

    posPlot->setReadoutVisible(true); 
    velPlot->setReadoutVisible(true); 

    auto* posAdapter  = 
        new TopicAdapter<abv_msgs::msg::AbvState, QVector<double>>(
            "/abv/state",   
            &conversions::navigationPositionConvertor);

    auto* velAdapter = 
        new TopicAdapter<abv_msgs::msg::AbvState, QVector<double>>(
            "abv/state",
            &conversions::navigationVelocityConvertor); 

    auto* ctrlAdapter = 
        new TopicAdapter<abv_msgs::msg::AbvControllerStatus, QVector<double>>(
            "abv/controller/status",
            &conversions::controllerStatusConvertor);

    posPlot->connectTo(posAdapter); 
    velPlot->connectTo(velAdapter); 
    ctrlPlot->connectTo(ctrlAdapter); 

    auto* central = new QWidget(); 
    auto* mainLayout = new QHBoxLayout(central);
    auto* leftLayout = new QVBoxLayout();
    auto* rightLayout = new QVBoxLayout();

    mainLayout->addLayout(leftLayout, 4);   // plots
    mainLayout->addLayout(rightLayout, 1);  // values panel

    leftLayout->addWidget(posPlot);
    leftLayout->addWidget(velPlot);
    leftLayout->addWidget(ctrlPlot);

    // STOP BUTTON 
    auto stopBtn = new ButtonAdapter("Stop", std::bind(&btn::action::stop), ButtonStyle::danger());
    stopBtn->resize(50, 50); 
     
    rightLayout->addWidget(stopBtn); 

    CommandPanel* panel = new CommandPanel(); 
    rightLayout->addWidget(panel); 

    QMainWindow window; 
    window.setWindowTitle("ABV Ground Station"); 
    window.resize(1280, 720); 
    window.setCentralWidget(central); 

    QPalette dark;
    dark.setColor(QPalette::Window,          QColor(30, 30, 30));
    dark.setColor(QPalette::WindowText,      Qt::white);
    dark.setColor(QPalette::Base,            QColor(20, 20, 20));
    dark.setColor(QPalette::AlternateBase,   QColor(40, 40, 40));
    dark.setColor(QPalette::Text,            Qt::white);
    dark.setColor(QPalette::Button,          QColor(50, 50, 50));
    dark.setColor(QPalette::ButtonText,      Qt::white);
    dark.setColor(QPalette::Highlight,       QColor(0, 120, 215));
    dark.setColor(QPalette::HighlightedText, Qt::white);
    app.setPalette(dark);

    window.show(); 
    app.exec();

    rclcpp::shutdown(); 
    return 0;                                                                       
}