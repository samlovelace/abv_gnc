#include "abv_gui/NodeHealthPanel.h"
#include "abv_common/ConfigurationManager.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>

NodeHealthPanel::NodeHealthPanel(const QStringList& aExpectedNodes, const QString& aRobotIp, QWidget* parent)
    : QWidget(parent)
{
    HeartbeatConfig config = ConfigurationManager::getInstance()->getHeartbeatConfig();
    mStaleAfterSec = config.mStaleAfter;

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(4, 4, 4, 4);
    layout->setSpacing(4);

    auto* header = new QLabel("Health");
    header->setStyleSheet("color: #ffffff; font-weight: bold;");
    layout->addWidget(header);

    auto* row = new QHBoxLayout();
    row->setSpacing(4);

    // Comms indicator first - the more fundamental "is the robot even
    // reachable on the network" check, independent of any per-node status
    mCommsIndicator = new StatusIndicator("Comms");
    row->addWidget(mCommsIndicator);

    for (const QString& name : aExpectedNodes)
    {
        auto* indicator = new StatusIndicator(name);
        mIndicators[name] = indicator;
        mEverSeen[name] = false;
        mLastSeen[name] = QElapsedTimer();
        row->addWidget(indicator);
    }

    row->addStretch();
    layout->addLayout(row);
    layout->addStretch();

    mPinger = new NetworkPinger(aRobotIp, config.mPingIntervalMs, this);
    connect(mPinger, &NetworkPinger::reachabilityChanged, this, &NodeHealthPanel::onCommsReachabilityChanged);

    mPollTimer = new QTimer(this);
    connect(mPollTimer, &QTimer::timeout, this, &NodeHealthPanel::checkStaleness);
    mPollTimer->start(500);
}

void NodeHealthPanel::onHeartbeat(const QVariant& aNodeName)
{
    QString name = aNodeName.toString();

    if (!mIndicators.contains(name))
    {
        // not one of the expected nodes - fixed list, ignore
        return;
    }

    mEverSeen[name] = true;
    mLastSeen[name].restart();
    mIndicators[name]->setState(StatusIndicator::State::Connected);
}

void NodeHealthPanel::checkStaleness()
{
    for (auto it = mIndicators.begin(); it != mIndicators.end(); ++it)
    {
        const QString& name = it.key();

        if (mEverSeen[name] && mLastSeen[name].elapsed() > mStaleAfterSec * 1000)
        {
            it.value()->setState(StatusIndicator::State::Disconnected);
        }
    }
}

void NodeHealthPanel::onCommsReachabilityChanged(bool reachable)
{
    mCommsIndicator->setState(reachable ? StatusIndicator::State::Connected : StatusIndicator::State::Disconnected);
}
