#ifndef LIVEPLOT_H
#define LIVEPLOT_H
 
#include <QWidget>
#include <QLineSeries> 
#include <QChart> 
#include <QValueAxis>
#include <QString> 
#include <QMap> 

#include "abv_gui/TopicAdapter.hpp"

class LivePlot : public QWidget
{ 
Q_OBJECT

public:
    explicit LivePlot(const QString& aTitle, 
                      double aYmin, double aYmax,
                      QStringList aSeriesNames, 
                      double aWindowSecs = 30.0, 
                      QWidget* aParent = nullptr);
    ~LivePlot() = default; 

public slots: 
    void onNewData(double aTime, QVector<double> aValues); 

public: 
    QVector<double> latestValues() const;
    void connectTo(TopicAdapterBase* adapter);

private:
    QMap<QString, QLineSeries*> mSeries; 
    QStringList mSeriesNames; 
    QChart* mChart; 
    QValueAxis* mXAxis; 
    QValueAxis* mYAxis; 
    double mWindowSecs; 

    double mYMinSeen = std::numeric_limits<double>::max();
    double mYMaxSeen = std::numeric_limits<double>::lowest();
    bool mAutoScaleY = true;
    double mYPadding = 0.1; // 10% padding



};
#endif //LIVEPLOT_H