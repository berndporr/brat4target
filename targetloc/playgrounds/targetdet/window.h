#ifndef WINDOW_H
#define WINDOW_H

#include <QBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QPushButton>

#include "libcam2opencv.h"

#include "targetdet.h"

// class definition 'Window'
class Window : public QWidget
{
    // must include the Q_OBJECT macro for for the Qt signals/slots framework to work with this class
    Q_OBJECT

public:
    Window();
    void updateImageL(const cv::Mat &mat);
    void onQRdetected(std::vector<cv::Point2f> pts) {
        qrCoord = targetDet.calcCentre(pts);
    }

private:
    QVBoxLayout  *vLayout;
    QLabel       *imageL;
    QLabel       *targetInfo;

    static constexpr int displaywidth = 640;

    TargetDet targetDet;
    cv::Point2f qrCoord;
};

#endif // WINDOW_H
