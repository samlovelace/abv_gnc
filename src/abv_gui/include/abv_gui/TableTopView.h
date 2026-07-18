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

    // Click-interaction mode: what a press/drag/release gesture on the table
    // does. Only SetGoalPose is implemented today; this exists so a future
    // mode (e.g. click-to-place a collision obstacle) can be added as a new
    // enum value + a new branch in the mouse handlers, without touching the
    // goal-pose code path.
    enum class InteractionMode { SetGoalPose };
    void setInteractionMode(InteractionMode aMode);

public slots:
    // Expects a QVector<double>{x, y, yaw, valid} as produced by
    // conversions::navigationStateConvertor.
    void onPoseUpdate(const QVariant& aData);

    // Expects a QString holding the 8-char '0'/'1' AbvThrusterStatus.thrusters
    // string, index i = thruster (i+1) (see Control.Thrusters.Allocation).
    void onThrusterState(const QVariant& aData);

    // Hides the proposed-goal ghost. Called after the caller has resolved
    // (sent or cancelled) a goalPoseSelected signal.
    void clearGoalGhost();

signals:
    // Emitted on mouse release after a SetGoalPose press/drag gesture. This
    // widget doesn't know about ROS/CommandPanel - the caller decides what
    // "sending" a goal pose means (see main.cpp).
    void goalPoseSelected(double aX, double aY, double aYaw);

protected:
    void paintEvent(QPaintEvent* aEvent) override;
    void mousePressEvent(QMouseEvent* aEvent) override;
    void mouseMoveEvent(QMouseEvent* aEvent) override;
    void mouseReleaseEvent(QMouseEvent* aEvent) override;

private:
    QRectF tableToWidget() const;
    double worldScale(const QRectF& aTableRect) const;
    QPointF worldToPixel(const QRectF& aTableRect, double aX, double aY) const;
    QPointF pixelToWorld(const QRectF& aTableRect, const QPointF& aPixel) const;

    void drawGrid(QPainter& aPainter, const QRectF& aTableRect) const;
    void drawRobot(QPainter& aPainter, const QRectF& aTableRect) const;
    void drawGoalGhost(QPainter& aPainter, const QRectF& aTableRect) const;
    void drawReadout(QPainter& aPainter) const;

    TableViewConfig mConfig;

    double mX{0.0};
    double mY{0.0};
    double mYaw{0.0};
    bool mHasPose{false};

    QString mThrusterState{"00000000"};

    InteractionMode mInteractionMode{InteractionMode::SetGoalPose};
    bool mDraggingGoal{false};
    bool mHasGoalGhost{false};
    double mGoalX{0.0};
    double mGoalY{0.0};
    double mGoalYaw{0.0};
    QPointF mDragStartWorld;
};
