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
#include <QMenu>
#include <QCursor>
#include <QDebug>
#include <cmath>

#include "autobot_msgs/msg/controller_command.hpp"
#include "autobot_msgs/msg/guidance_command.hpp"
#include "autobot_msgs/msg/guidance_status.hpp"
#include "autobot_msgs/msg/heartbeat.hpp"
#include "autobot_msgs/msg/obstacle_array.hpp"
#include "autobot_msgs/msg/path.hpp"
#include "autobot_msgs/msg/thruster_status.hpp"
#include "autobot_msgs/msg/policy_active.hpp"
#include "autobot_msgs/msg/policy_registry.hpp"

#include "autobot_config/CommonConfigManager.h"
#include "autobot_config/GncConfigManager.h"

#include "autobot_gui/LivePlot.h"
#include "autobot_gui/TopicAdapter.hpp"
#include "autobot_gui/TopicConversions.hpp"

#include "autobot_gui/ButtonActions.hpp"
#include "autobot_gui/ButtonAdapter.hpp"

#include "autobot_gui/CommandPanel.h"
#include "autobot_gui/StatusPanel.h"
#include "autobot_gui/NodeHealthPanel.h"
#include "autobot_gui/TableTopView.h"
#include "autobot_gui/PolicySelectorPanel.h"
#include "autobot_gui/ActivePolicyBanner.h"

int main(int argc, char *argv[])
{
    CommonConfigManager::getInstance()->loadConfiguration();
    GncConfigManager::getInstance()->loadConfiguration();

    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("autobot_gui")->spinNode();
    RosTopicManager::getInstance()->createPublisher<autobot_msgs::msg::ControllerCommand>("abv/controller/command");
    RosTopicManager::getInstance()->createPublisher<autobot_msgs::msg::GuidanceCommand>("abv/guidance/command");
    RosTopicManager::getInstance()->createPublisher<autobot_msgs::msg::ObstacleArray>("abv/scene/obstacles");

    QApplication app(argc, argv);
    app.setStyle("Fusion"); 

    // --- Create plots ---
    auto *posPlot  = new LivePlot("Position",       0.0,  2.0, {"x", "y", "yaw"});
    auto *velPlot  = new LivePlot("Velocity",      -2.0,  2.0, {"vx", "vy", "w"});
    auto *ctrlPlot = new LivePlot("Control Input", -0.5,  0.5, {"fx", "fy", "tz"});

    posPlot->setReadoutVisible(true); 
    velPlot->setReadoutVisible(true); 

    auto* posAdapter  = 
        new TopicAdapter<autobot_msgs::msg::State, QVector<double>>(
            "/abv/state",   
            &conversions::navigationPositionConvertor);

    auto* velAdapter = 
        new TopicAdapter<autobot_msgs::msg::State, QVector<double>>(
            "abv/state",
            &conversions::navigationVelocityConvertor); 

    auto* ctrlAdapter =
        new TopicAdapter<autobot_msgs::msg::ThrusterStatus, QVector<double>>(
            "abv/thrusters/status",
            &conversions::thrusterStatusConvertor);

    posPlot->connectTo(posAdapter); 
    velPlot->connectTo(velAdapter); 
    ctrlPlot->connectTo(ctrlAdapter); 

    auto* central = new QWidget();
    auto* outerLayout = new QVBoxLayout(central);
    // zeroed so the banner below spans edge-to-edge; mainLayout restores the
    // usual breathing room for everything underneath it
    outerLayout->setContentsMargins(0, 0, 0, 0);
    outerLayout->setSpacing(0);

    auto* activePolicyBanner = new ActivePolicyBanner();
    outerLayout->addWidget(activePolicyBanner);

    auto* mainLayout = new QHBoxLayout();
    mainLayout->setContentsMargins(9, 9, 9, 9);
    auto* leftLayout = new QVBoxLayout();
    auto* rightLayout = new QVBoxLayout();

    mainLayout->addLayout(leftLayout, 3);   // plots
    mainLayout->addLayout(rightLayout, 1);  // values panel
    outerLayout->addLayout(mainLayout);

    leftLayout->addWidget(posPlot);
    leftLayout->addWidget(velPlot);
    leftLayout->addWidget(ctrlPlot);

    // STOP BUTTON
    auto stopBtn = new ButtonAdapter("Stop", std::bind(&btn::action::stop), ButtonStyle::danger());
    stopBtn->resize(50, 50);

    rightLayout->addWidget(stopBtn);

    CommandPanel* panel = new CommandPanel();
    rightLayout->addWidget(panel);

    StatusPanel* status = new StatusPanel();
    rightLayout->addWidget(status);

    PolicySelectorPanel* policyPanel = new PolicySelectorPanel();
    rightLayout->addWidget(policyPanel);

    auto* policyRegistryAdapter =
        new TopicAdapter<autobot_msgs::msg::PolicyRegistry, QString>("abv/policy/registry",
            [](const autobot_msgs::msg::PolicyRegistry& msg) {
                QStringList parts;
                for (const auto& entry : msg.policies) {
                    parts << QString::fromStdString(entry.policy_name) + ":" + (entry.alive ? "1" : "0");
                }
                return parts.join(";");
            });
    QObject::connect(policyRegistryAdapter, &TopicAdapterBase::newDataVariant,
                      policyPanel, &PolicySelectorPanel::onRegistryUpdate);

    auto* policyActiveAdapter =
        new TopicAdapter<autobot_msgs::msg::PolicyActive, QString>("abv/policy/active",
            [](const autobot_msgs::msg::PolicyActive& msg) {
                return QString::fromStdString(msg.policy_name);
            });
    QObject::connect(policyActiveAdapter, &TopicAdapterBase::newDataVariant,
                      activePolicyBanner, &ActivePolicyBanner::setActivePolicy);

    QString robotIp = QString::fromStdString(
        GncConfigManager::getInstance()->getNavigationConfig().mLocalIp);

    auto* healthPanel = new NodeHealthPanel({"goal_manager", "navigation", "guidance", "bridge", "control", "thrusters"}, robotIp);
    rightLayout->addWidget(healthPanel);

    auto* heartbeatAdapter =
        new TopicAdapter<autobot_msgs::msg::Heartbeat, QString>("abv/heartbeat",
            [](const autobot_msgs::msg::Heartbeat& msg) {
                return QString::fromStdString(msg.node_name);
            });
    QObject::connect(heartbeatAdapter, &TopicAdapterBase::newDataVariant,
                      healthPanel, &NodeHealthPanel::onHeartbeat);
    auto* poseSync =
        new TopicAdapter<autobot_msgs::msg::State, QVector<double>>(
            "abv/state", &conversions::navigationPositionConvertor);

    // Connect (rather than mutating the panel's widgets inside the
    // converter above) so the update is delivered on the GUI thread instead
    // of racing the ROS subscription thread against the user editing the
    // same spin boxes.
    QObject::connect(poseSync, &TopicAdapterBase::newDataVariant,
                      panel, &CommandPanel::onPoseSync);

    auto* tableView = new TableTopView(CommonConfigManager::getInstance()->getTableViewConfig());

    auto* tableStateAdapter =
        new TopicAdapter<autobot_msgs::msg::State, QVector<double>>(
            "abv/state", &conversions::navigationStateConvertor);
    QObject::connect(tableStateAdapter, &TopicAdapterBase::newDataVariant,
                      tableView, &TableTopView::onPoseUpdate);

    auto* thrusterStateAdapter =
        new TopicAdapter<autobot_msgs::msg::ThrusterStatus, QString>("abv/thrusters/status",
            [](const autobot_msgs::msg::ThrusterStatus& msg) {
                return QString::fromStdString(msg.thrusters);
            });
    QObject::connect(thrusterStateAdapter, &TopicAdapterBase::newDataVariant,
                      tableView, &TableTopView::onThrusterState);

    auto* pathAdapter =
        new TopicAdapter<autobot_msgs::msg::Path, QVector<QPointF>>(
            "abv/guidance/path", &conversions::pathConvertor);
    QObject::connect(pathAdapter, &TopicAdapterBase::newDataVariant,
                      tableView, &TableTopView::onPathUpdate);

    // Full-replace obstacle set (no per-obstacle IDs) - main() is the source
    // of truth for what's currently published on abv/scene/obstacles.
    static std::vector<autobot_msgs::msg::Obstacle> obstacles;

    auto publishObstacles = []() {
        autobot_msgs::msg::ObstacleArray msg;
        msg.set__obstacles(obstacles);
        RosTopicManager::getInstance()->publishMessage<autobot_msgs::msg::ObstacleArray>(
            "abv/scene/obstacles", msg);
    };

    auto refreshObstacleView = [tableView]() {
        QVector<PlacedObstacle> view;
        view.reserve(static_cast<int>(obstacles.size()));
        for (const auto& o : obstacles)
        {
            view.push_back(PlacedObstacle{o.x, o.y, o.radius});
        }
        tableView->setObstacles(view);
    };

    // Left-click-drag-release on the table proposes a goal pose (ghost shown
    // by tableView itself); on release we pop a small confirm menu and only
    // publish if "Send Goal" is chosen. tableView knows nothing about ROS -
    // it just reports the proposed pose. If sent, the ghost is left in
    // place as a static marker of the commanded goal (cleared only if the
    // user places a new one); if the menu is dismissed without choosing
    // "Send Goal", the ghost is cleared since nothing was commanded.
    QObject::connect(tableView, &TableTopView::goalPoseSelected,
                      [tableView, panel](double x, double y, double yaw) {
        QMenu menu;
        QAction* sendAction = menu.addAction(
            QString("Send Goal (%1, %2, %3\xC2\xB0)")
                .arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(yaw * 180.0 / M_PI, 0, 'f', 0));

        QAction* chosen = menu.exec(QCursor::pos());
        if (chosen == sendAction)
        {
            panel->sendPoseCommand(x, y, yaw);
        }
        else
        {
            tableView->clearGoalGhost();
        }
    });

    // Right-click-drag-release on the table proposes a circular obstacle
    // (ghost shown by tableView itself); on release we pop a confirm menu,
    // same shape as the goal-pose flow above. Confirming appends to the full
    // obstacle set and republishes it (full-replace semantics - no
    // per-obstacle IDs) on abv/scene/obstacles; cancelling just clears the
    // ghost. A plain right-click (no drag) skips this signal entirely -
    // tableView offers "Clear Obstacles" itself in that case (see
    // clearObstaclesRequested below).
    QObject::connect(tableView, &TableTopView::obstaclePlaced,
                      [tableView, panel, publishObstacles, refreshObstacleView](double x, double y, double radius) {
        QMenu menu;
        QAction* addAction = menu.addAction(
            QString("Add Obstacle (%1, %2, r=%3)")
                .arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(radius, 0, 'f', 2));

        QAction* chosen = menu.exec(QCursor::pos());
        if (chosen == addAction)
        {
            autobot_msgs::msg::Obstacle obstacle;
            obstacle.set__x(x);
            obstacle.set__y(y);
            obstacle.set__radius(radius);
            obstacles.push_back(obstacle);

            publishObstacles();
            panel->setObstaclesPresent(!obstacles.empty());
        }

        tableView->clearObstacleGhost();
        refreshObstacleView();
    });

    // Right-click "Clear Obstacles" on the table (a plain right-click with no
    // drag - see TableTopView::mouseReleaseEvent) republishes an empty
    // obstacle set.
    QObject::connect(tableView, &TableTopView::clearObstaclesRequested,
                      [panel, publishObstacles, refreshObstacleView]() {
        obstacles.clear();
        publishObstacles();
        panel->setObstaclesPresent(!obstacles.empty());
        refreshObstacleView();
    });

    mainLayout->insertWidget(0, tableView, 1);  // always-visible table view, left of the plots

    auto* gdnceStatus =
        new TopicAdapter<autobot_msgs::msg::GuidanceStatus, QString>("abv/guidance/status", 
            [status](const autobot_msgs::msg::GuidanceStatus& msg){

                status->setGuidanceSmState(QString::fromStdString(msg.status.node_state)); 
                return ""; 
            });

    auto* ctrlStatus = 
        new TopicAdapter<autobot_msgs::msg::ControllerStatus, QString>("abv/controller/status", 
            [status](const autobot_msgs::msg::ControllerStatus& msg) {
                
                QString smState = QString::fromStdString(msg.status.node_state); 
                status->setControllerSmState(smState); 

                const auto idle =  autobot_msgs::msg::ControllerStatus::IDLE; 
                const auto running = autobot_msgs::msg::ControllerStatus::RUNNING; 
                const auto arrived = autobot_msgs::msg::ControllerStatus::ARRIVED; 

                QString ctrlState; 
                
                switch (msg.arrival)
                {
                case autobot_msgs::msg::ControllerStatus::IDLE:
                    ctrlState = "IDLE"; 
                    break;
                case autobot_msgs::msg::ControllerStatus::RUNNING: 
                    ctrlState = "RUNNING"; 
                    break; 
                case autobot_msgs::msg::ControllerStatus::ARRIVED:
                    ctrlState = "ARRIVED"; 
                    break; 
                default:
                    "---";
                }

                status->setControllerCtrlState(ctrlState); 
                return ""; 
        });

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