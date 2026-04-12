#pragma once
#include <QWidget>
#include <QLabel>

class StatusPanel : public QWidget
{
    Q_OBJECT
public:
    explicit StatusPanel(QWidget* parent = nullptr);

    void setGuidanceSmState(const QString& state);
    void setControllerSmState(const QString& state);
    void setControllerCtrlState(const QString& state);

private:
    static QLabel* makeValueLabel();
    static QWidget* makeSectionHeader(const QString& title);

    QLabel* mGuidanceSmState{nullptr};
    QLabel* mControllerSmState{nullptr};
    QLabel* mControllerCtrlState{nullptr};
};