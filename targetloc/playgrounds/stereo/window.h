#ifndef WINDOW_H
#define WINDOW_H

#include <QBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QPushButton>

#include "libcam2opencv.h"

#include "stereo.h"

// class definition 'Window'
class Window : public QWidget
{
    // must include the Q_OBJECT macro for the Qt signals/slots framework to work with this class
    Q_OBJECT

public:
    Window();
    void updateImageL(const cv::Mat &mat);
    void updateImageR(const cv::Mat &mat);

private:
    QHBoxLayout  *h1Layout;
    QHBoxLayout  *h2Layout;
    QVBoxLayout  *vLayout;
    QLabel       *imageL;
    QLabel       *imageR;
    QLabel       *imageCombined;
    QLabel       *imageDisparity;

    cv::Mat currentL;
    cv::Mat currentR;
    cv::Mat currentD;
    bool refreshDisparity = false;

    Stereo stereo;

    // blends the current L and R images and displays it
    void blendLRandDisplayD();

    const cv::Size imageSize{640,360};
};

#endif // WINDOW_H
