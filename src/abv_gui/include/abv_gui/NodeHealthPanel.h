#pragma once

#include <QWidget>
#include <QString>
#include <QStringList>
#include <QMap>
#include <QVariant>
#include <QElapsedTimer>
#include <QTimer>

#include "abv_gui/StatusIndicator.hpp"
#include "abv_gui/NetworkPinger.h"

// Shows one health indicator per expected node (fixed list - starts
// Disconnected, turns Connected on first heartbeat, back to Disconnected if
// heartbeats stop arriving), plus a separate "Comms" indicator driven by
// NetworkPinger (a genuine network-layer reachability check to the robot
// host, independent of the per-node heartbeat topic).
class NodeHealthPanel : public QWidget
{
    Q_OBJECT
public:
    NodeHealthPanel(const QStringList& aExpectedNodes, const QString& aRobotIp, QWidget* parent = nullptr);

public slots:
    // connect a heartbeat TopicAdapter's newDataVariant signal here (the
    // adapter's converter must stay a pure function - do not mutate this
    // panel from inside the converter itself)
    void onHeartbeat(const QVariant& aNodeName);

private slots:
    void checkStaleness();
    void onCommsReachabilityChanged(bool reachable);

private:
    QMap<QString, StatusIndicator*> mIndicators;
    QMap<QString, QElapsedTimer> mLastSeen;
    QMap<QString, bool> mEverSeen;

    StatusIndicator* mCommsIndicator{nullptr};
    NetworkPinger* mPinger{nullptr};

    QTimer* mPollTimer{nullptr};
    double mStaleAfterSec;
};
