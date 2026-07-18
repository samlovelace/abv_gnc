#pragma once

#include <QWidget>
#include <QVariant>
#include <QString>
#include <QPointF>
#include <QRectF>

#include "abv_common/Configurations.h"

// Top-down, to-scale render of the physical table with the robot's live
// pose: gridded table surface and a heading-aware robot glyph (square
// aluminum-extrusion body + front camera housing + thruster nozzles that
// light up when firing). Pose arrives via onPoseUpdate, meant to be
// connected to a TopicAdapter<AbvState, QVector<double>>'s newDataVariant
// signal (see navigationStateConvertor) so updates land on the GUI thread
// rather than the ROS subscription thread. Thruster state arrives via
// onThrusterState the same way, from AbvThrusterStatus.thrusters.
class TableTopView : public QWidget
{
    Q_OBJECT
public:
    explicit TableTopView(const TableViewConfig& aConfig, QWidget* parent = nullptr);

public slots:
    // Expects a QVector<double>{x, y, yaw, valid} as produced by
    // conversions::navigationStateConvertor.
    void onPoseUpdate(const QVariant& aData);

    // Expects a QString holding the 8-char '0'/'1' AbvThrusterStatus.thrusters
    // string, index i = thruster (i+1) (see Control.Thrusters.Allocation).
    void onThrusterState(const QVariant& aData);

protected:
    void paintEvent(QPaintEvent* aEvent) override;

private:
    QRectF tableToWidget() const;
    double worldScale(const QRectF& aTableRect) const;
    QPointF worldToPixel(const QRectF& aTableRect, double aX, double aY) const;

    void drawGrid(QPainter& aPainter, const QRectF& aTableRect) const;
    void drawRobot(QPainter& aPainter, const QRectF& aTableRect) const;
    void drawReadout(QPainter& aPainter) const;

    TableViewConfig mConfig;

    double mX{0.0};
    double mY{0.0};
    double mYaw{0.0};
    bool mHasPose{false};

    QString mThrusterState{"00000000"};
};
