#ifndef WINDOW_H
#define WINDOW_H

#include <QBoxLayout>
#include <qcustomplot.h>
#include <QSizePolicy>

class QLIDARPlot:public QCustomPlot
{
public:
    QLIDARPlot():QCustomPlot(){
	QSizePolicy p(sizePolicy());
        p.setHeightForWidth(true);
        setSizePolicy(p);
    };
    virtual int heightForWidth ( int w ) const override { return w;};
};


// class definition 'Window'
class Window : public QWidget
{
    // must include the Q_OBJECT macro for the Qt signals/slots framework to work with this class
    Q_OBJECT

public:
    Window();

    void timerEvent(QTimerEvent *);


private:
    QVBoxLayout  *vLayout;
    QLIDARPlot *lidarPlot;
    float t = 0;

};

#endif // WINDOW_H
