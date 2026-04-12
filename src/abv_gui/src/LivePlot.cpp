#include "abv_gui/LivePlot.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QChartView>
#include <QGraphicsLayout>
#include <QPainter>
#include <iostream>

static const QList<QColor> kColors = {
    QColor("#e71111"),
    QColor("#14da14"),
    QColor("#1a1ad1"),
};

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

LivePlot::LivePlot(const QString& title,
                   double yMin, double yMax,
                   QStringList seriesNames,
                   double windowSecs,
                   QWidget* parent)
    : QWidget(parent)
    , mWindowSecs(windowSecs)
    , mSeriesNames(seriesNames)
{
    // ── Axes ──────────────────────────────────────────────────────────────────
    mXAxis = new QValueAxis();
    mXAxis->setRange(0, mWindowSecs);
    mXAxis->setLabelFormat("%.1f");

    mYAxis = new QValueAxis();
    mYAxis->setRange(yMin, yMax);

    // After mYAxis setup, grab the chart's font
    QFont chartFont = mXAxis->labelsFont();

    // ── Chart ─────────────────────────────────────────────────────────────────
    mChart = new QChart();
    mChart->setTitle(title);
    mChart->setTheme(QChart::ChartThemeDark);
    mChart->setBackgroundBrush(QColor(30, 30, 30));
    mChart->addAxis(mXAxis, Qt::AlignBottom);
    mChart->addAxis(mYAxis, Qt::AlignLeft);
    mChart->legend()->setVisible(true);
    mChart->legend()->setAlignment(Qt::AlignRight);
    mChart->setMargins(QMargins(0, 0, 0, 0));
    mChart->layout()->setContentsMargins(0, 0, 0, 0);

    // ── Series ────────────────────────────────────────────────────────────────
    for (int i = 0; i < seriesNames.size(); ++i) {
        const QString& name = seriesNames[i];
        QColor color = kColors[i % kColors.size()];

        auto* s = new QLineSeries();
        s->setName(name);
        s->setColor(color);

        QPen pen = s->pen();
        pen.setWidth(2);
        s->setPen(pen);

        mChart->addSeries(s);
        s->attachAxis(mXAxis);
        s->attachAxis(mYAxis);

        mSeries[name] = s;
        mSeriesOrder.append(name);      // preserve insertion order
        mSeriesColors.append(color);    // parallel color list
    }

    // ── Chart view ────────────────────────────────────────────────────────────
    auto* view = new QChartView(mChart, this);
    view->setRenderHint(QPainter::Antialiasing);
    view->setRubberBand(QChartView::RectangleRubberBand);
    view->setInteractive(true);
    view->setDragMode(QGraphicsView::ScrollHandDrag);

    // ── Readout panel ─────────────────────────────────────────────────────────
    mReadoutPanel = new QWidget(this);
    mReadoutPanel->setStyleSheet("background-color: #1e1e1e;");

    auto* readoutLayout = new QHBoxLayout(mReadoutPanel);
    readoutLayout->setContentsMargins(8, 4, 8, 4);
    readoutLayout->setSpacing(24);

    for (int i = 0; i < mSeriesOrder.size(); ++i) {
        const QString& name = mSeriesOrder[i];
        const QColor&  color = mSeriesColors[i];

        auto* label = new QLabel(this);
        label->setFont(chartFont);
        label->setTextFormat(Qt::RichText);
        label->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
        label->setText(makeReadoutText(name, "---"));

        mReadoutLabels[name] = label;
        readoutLayout->addWidget(label);
    }

    readoutLayout->addStretch();
    mReadoutPanel->setVisible(false);

    // ── Top-level layout ──────────────────────────────────────────────────────
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    layout->addWidget(view);
    layout->addWidget(mReadoutPanel);
}

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

void LivePlot::setReadoutVisible(bool visible)
{
    mShowReadout = visible;
    mReadoutPanel->setVisible(visible);
}

void LivePlot::connectTo(TopicAdapterBase* adapter)
{
    static std::chrono::steady_clock::time_point mStartTime;
    mStartTime = std::chrono::steady_clock::now();

    QObject::connect(adapter, &TopicAdapterBase::newDataVariant,
                     this, [this](const QVariant& v) {
        double t = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - mStartTime
        ).count();
        onNewData(t, v.value<QVector<double>>());
    });
}

QVector<double> LivePlot::latestValues() const
{
    QVector<double> values;
    for (const QString& name : mSeriesOrder) {
        auto* s = mSeries[name];
        values.push_back(s->count() > 0 ? s->at(s->count() - 1).y() : 0.0);
    }
    return values;
}

// ─────────────────────────────────────────────────────────────────────────────
// Slots
// ─────────────────────────────────────────────────────────────────────────────

void LivePlot::onNewData(double aTime, QVector<double> aValues)
{
    double batchMin = std::numeric_limits<double>::max();
    double batchMax = std::numeric_limits<double>::lowest();

    for (int i = 0; i < aValues.size() && i < mSeriesOrder.size(); ++i) {
        auto* s = mSeries[mSeriesOrder[i]];
        s->append(aTime, aValues[i]);

        batchMin = std::min(batchMin, aValues[i]);
        batchMax = std::max(batchMax, aValues[i]);
    }

    // ── Autoscale Y ───────────────────────────────────────────────────────────
    if (mAutoScaleY) {
        bool changed = false;
        if (batchMin < mYMinSeen) { mYMinSeen = batchMin; changed = true; }
        if (batchMax > mYMaxSeen) { mYMaxSeen = batchMax; changed = true; }

        if (changed) {
            double range = mYMaxSeen - mYMinSeen;
            double pad   = range * mYPadding;
            mYAxis->setRange(mYMinSeen - pad, mYMaxSeen + pad);
        }
    }

    mXAxis->setRange(0, aTime);

    // ── Update readouts ───────────────────────────────────────────────────────
    if (mShowReadout) {
        for (int i = 0; i < aValues.size() && i < mSeriesOrder.size(); ++i) {
            const QString& name  = mSeriesOrder[i];
            const QColor&  color = mSeriesColors[i];
            QString        val   = QString::number(aValues[i], 'f', 2);
            mReadoutLabels[name]->setText(makeReadoutText(name, val));
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

QString LivePlot::makeReadoutText(const QString& name, const QString& value)
{
    return QString(
        "<span style='color:#ffffff; font-weight:bold;'>%1</span>"
        "<span style='color:#aaaaaa;'>: </span>"
        "<span style='color:#ffffff;'>%2</span>"
    ).arg(name, value);
}