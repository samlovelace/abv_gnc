
#include "abv_gui/LivePlot.h"
#include <iostream>

#include <QVBoxLayout>
#include <QChartView> 
#include <QGraphicsLayout>

// static const QList<QColor> kColors = {
//     QColor("#c62828"),  // balanced red
//     QColor("#2e7d32"),  // balanced green
//     QColor("#1565c0"),  // balanced blue
// };

// static const QList<QColor> kColors = {
//     QColor("#e53935"),  // lighter red
//     QColor("#43a047"),  // lighter green
//     QColor("#1e88e5"),  // lighter blue
// };

static const QList<QColor> kColors = {
    QColor("#e71111"),  
    QColor("#14da14"),
    QColor("#1a1ad1"), 
};

LivePlot::LivePlot(const QString &title,
                   double yMin, double yMax,
                   QStringList seriesNames,
                   double windowSecs,
                   QWidget *parent)
: QWidget(parent), mWindowSecs(windowSecs), mSeriesNames(seriesNames)
{
    mXAxis = new QValueAxis();
    mXAxis->setRange(0, mWindowSecs);
    mXAxis->setLabelFormat("%.1f");
    //mXAxis->setTitleText("Time (s)");

    mYAxis = new QValueAxis();
    mYAxis->setRange(yMin, yMax);

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

    // Create one series per name
    int i = 0;
    for (const QString &name : seriesNames) 
    {
        QLineSeries* s = new QLineSeries();
        s->setName(name);
        s->setColor(kColors[i++ % kColors.size()]);

        mChart->addSeries(s);
        s->attachAxis(mXAxis);
        s->attachAxis(mYAxis);

        QPen pen = s->pen();
        pen.setWidth(2);
        s->setPen(pen); 

        mSeries[name] = s;
    }

    auto *view = new QChartView(mChart, this);
    view->setRenderHint(QPainter::Antialiasing);
    view->setRubberBand(QChartView::RectangleRubberBand);
    view->setInteractive(true);
    view->setDragMode(QGraphicsView::ScrollHandDrag);

    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(view);
}

void LivePlot::onNewData(double aTime, QVector<double> aValues)
{
    auto keys = mSeries.keys(); 

    double batchMin = std::numeric_limits<double>::max();
    double batchMax = std::numeric_limits<double>::lowest();

    for (int i = 0; i < aValues.size() && i < keys.size(); i++)
    {
        auto* s = mSeries[keys[i]];
        s->append(aTime, aValues[i]);

        // Track min/max for this batch
        batchMin = std::min(batchMin, aValues[i]);
        batchMax = std::max(batchMax, aValues[i]);
    }

    // Expand-only autoscaling
    if (mAutoScaleY)
    {
        bool changed = false;

        if (batchMin < mYMinSeen) {
            mYMinSeen = batchMin;
            changed = true;
        }
        if (batchMax > mYMaxSeen) {
            mYMaxSeen = batchMax;
            changed = true;
        }

        if (changed)
        {
            double range = mYMaxSeen - mYMinSeen;
            double pad = range * mYPadding;

            mYAxis->setRange(mYMinSeen - pad, mYMaxSeen + pad);
        }
    }

    mXAxis->setRange(0, aTime);
}

QVector<double> LivePlot::latestValues() const
{
    QVector<double> values;

    auto keys = mSeries.keys();
    for (const auto& k : keys)
    {
        auto* s = mSeries[k];
        if (s->count() > 0)
            values.push_back(s->at(s->count() - 1).y());
        else
            values.push_back(0.0);
    }

    return values;
}

// Wire this plot to any adapter that emits QVector<double>
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