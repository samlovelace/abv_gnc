#include "abv_gui/NodeHealthPanel.h"
#include "abv_common/ConfigurationManager.h"

#include <QVBoxLayout>
#include <QGridLayout>
#include <QLabel>

namespace
{
constexpr int kGridColumns = 2;
}

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

    // 2xN grid rather than one wide row (forces this panel's column wider
    // than intended, stealing horizontal space from the plots next to it)
    // or one tall single-file stack (looked visually unbalanced) - this
    // keeps a compact footprint in both directions.
    auto* grid = new QGridLayout();
    grid->setSpacing(4);

    int nextSlot = 0;
    auto placeInGrid = [&](StatusIndicator* aIndicator)
    {
        grid->addWidget(aIndicator, nextSlot / kGridColumns, nextSlot % kGridColumns);
        nextSlot++;
    };

    // Comms indicator first - the more fundamental "is the robot even
    // reachable on the network" check, independent of any per-node status
    mCommsIndicator = new StatusIndicator("Comms");
    placeInGrid(mCommsIndicator);

    for (const QString& name : aExpectedNodes)
    {
        auto* indicator = new StatusIndicator(name);
        mIndicators[name] = indicator;
        mEverSeen[name] = false;
        mLastSeen[name] = QElapsedTimer();
        placeInGrid(indicator);
    }

    layout->addLayout(grid);
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
