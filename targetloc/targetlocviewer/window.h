#ifndef WINDOW_H
#define WINDOW_H

#include <QBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QPushButton>

#include <opencv2/core/types.hpp>
#include <qcustomplot.h>

#include "targetloc.h"

// class definition 'Window'
class Window : public QWidget
{
    // must include the Q_OBJECT macro for the Qt signals/slots framework to work with this class
    Q_OBJECT

public:
    Window();
    ~Window()
    {
        targetLoc.stop();
    }

private:
    QVBoxLayout *vLayout;

    QLabel *image;
    QLabel *info;

    void updateGUI();

    void timerEvent(QTimerEvent *event);

    TargetLoc targetLoc;

    const cv::Size displayImageSize{1640/2, 1232/2};

    virtual void newTargetDetected(const float r, const float phi);
};

#endif // WINDOW_H
