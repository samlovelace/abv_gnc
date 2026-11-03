#pragma once

#include <QWidget>
#include <QString>
#include <QPainter>

class StatusIndicator : public QWidget {
    Q_OBJECT
public:
    enum class State { Connected, Disconnected, Warning };

    explicit StatusIndicator(const QString& label, QWidget* parent = nullptr)
        : QWidget(parent), mLabel(label), mState(State::Disconnected)
    {
        setFixedSize(120, 32);
    }

public slots:
    void setState(State state) {
        mState = state;
        update(); // triggers repaint
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);

        // Pick color based on state
        QColor fill;
        switch (mState) {
            case State::Connected:    fill = QColor(76, 175, 80);   break; // green
            case State::Disconnected: fill = QColor(211, 47,  47);  break; // red
            case State::Warning:      fill = QColor(255, 160,  0);  break; // amber
        }

        // Draw pill-shaped background
        QRectF r = rect().adjusted(1, 1, -1, -1);
        p.setBrush(fill);
        p.setPen(fill.darker(130));
        p.drawRoundedRect(r, r.height() / 2, r.height() / 2);

        // Draw label
        p.setPen(Qt::white);
        QFont f = p.font();
        f.setBold(true);
        f.setPointSize(9);
        p.setFont(f);
        p.drawText(r, Qt::AlignCenter, mLabel);
    }

private:
    QString mLabel;
    State   mState;
};