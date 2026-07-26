#pragma once

#include <QObject>
#include <QString>
#include <QTimer>
#include <QProcess>

// Periodically shells out to the system `ping` binary to check whether a
// host is reachable on the network. Fully async (QProcess::start doesn't
// block, results arrive via the finished() signal on the Qt event loop) -
// this is a genuine network-layer reachability check, independent of
// ROS/DDS, so it stays meaningful even if DDS discovery itself is broken.
class NetworkPinger : public QObject
{
    Q_OBJECT
public:
    explicit NetworkPinger(const QString& aHost, int aIntervalMs, QObject* parent = nullptr);

signals:
    void reachabilityChanged(bool reachable);

private slots:
    void pingOnce();
    void onPingFinished(int exitCode, QProcess::ExitStatus exitStatus);

private:
    QString mHost;
    QTimer* mTimer;
    QProcess* mProcess{nullptr};
};
