#include "abv_gui/NetworkPinger.h"

NetworkPinger::NetworkPinger(const QString& aHost, int aIntervalMs, QObject* parent)
    : QObject(parent), mHost(aHost)
{
    mTimer = new QTimer(this);
    connect(mTimer, &QTimer::timeout, this, &NetworkPinger::pingOnce);
    mTimer->start(aIntervalMs);
}

void NetworkPinger::pingOnce()
{
    if (mProcess)
    {
        // previous ping hasn't finished yet - skip this tick rather than overlap
        return;
    }

    mProcess = new QProcess(this);
    connect(mProcess, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &NetworkPinger::onPingFinished);
    mProcess->start("ping", {"-c", "1", "-W", "1", mHost});
}

void NetworkPinger::onPingFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
    bool reachable = (exitStatus == QProcess::NormalExit && exitCode == 0);
    emit reachabilityChanged(reachable);

    mProcess->deleteLater();
    mProcess = nullptr;
}
