#pragma once

#include <QWidget>
#include <QChart>
#include <QLineSeries>
#include <QValueAxis>
#include <QLabel>
#include <QMap>
#include <QVector>
#include <QVariant>
#include <QStringList>
#include <chrono>

#include "abv_gui/TopicAdapter.hpp"

class LivePlot : public QWidget
{
    Q_OBJECT

public:
    explicit LivePlot(const QString& title,
                      double yMin, double yMax,
                      QStringList seriesNames,
                      double windowSecs = 10.0,
                      QWidget* parent = nullptr);

    void connectTo(TopicAdapterBase* adapter);
    void setReadoutVisible(bool visible);

    QVector<double> latestValues() const;

public slots:
    void onNewData(double aTime, QVector<double> aValues);

private:
    static QString makeReadoutText(const QString& name, const QString& value);

    QChart*                     mChart{nullptr};
    QValueAxis*                 mXAxis{nullptr};
    QValueAxis*                 mYAxis{nullptr};
    QMap<QString, QLineSeries*> mSeries;
    QList<QString>              mSeriesOrder;   // insertion-order keys
    QList<QColor>               mSeriesColors;  // parallel to mSeriesOrder

    QWidget*                    mReadoutPanel{nullptr};
    QMap<QString, QLabel*>      mReadoutLabels;
    bool                        mShowReadout{false};

    double mWindowSecs{10.0};
    QStringList mSeriesNames;

    // autoscale state
    bool   mAutoScaleY{true};
    double mYMinSeen{std::numeric_limits<double>::max()};
    double mYMaxSeen{std::numeric_limits<double>::lowest()};
    double mYPadding{0.05};
};