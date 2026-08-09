#include "window.h"
#include <c1lidarrpi.h>

Window::Window ()
{
    vLayout = new QVBoxLayout ();

    image = new QLabel;
    vLayout->addWidget (image);
    info = new QLabel;
    vLayout->addWidget (info);

    setLayout (vLayout);

    fprintf (stderr, "Starting screen update timer.\n");
    startTimer (std::chrono::milliseconds{ 100 });

    fprintf (stderr, "Starting Targetloc.\n");
    targetLoc.registerDetectionEvent (
        [&] (const float r, const float phi) { newTargetDetected (r, phi); });
    targetLoc.start ();
}

void Window::updateGUI ()
{
    cv::Mat resized;
    cv::Mat orig = targetLoc.getCurrentCameraImage ().clone ();

    if (targetLoc.getCurrentCameraImage ().empty ())
        return;

    if (!targetLoc.getQRcodeContour ().empty ())
        {
            std::vector<std::vector<cv::Point> > contours{
                targetLoc.getQRcodeContour ()
            };
            if (false)
                {
                    fprintf (stderr, "Contours(%ld): ", contours.size ());
                    for (auto &c : contours[0])
                        {
                            fprintf (stderr, "[%d,%d]", c.x, c.y);
                        }
                    fprintf (stderr, "\n");
                }
            cv::drawContours (orig, contours, -1, { 0, 255, 0 }, 3);
        }

    cv::resize (orig, resized, displayImageSize);

    const QImage frameD (resized.data, resized.cols, resized.rows,
                         resized.step, QImage::Format_BGR888);

    image->setPixmap (QPixmap::fromImage (frameD));
}

// properly done with a callback!
void Window::newTargetDetected (const float r, const float phi)
{
    fprintf (stderr, "Target at [r=%f,phi=%f].\n", r, phi);
    char tmp[256];
    sprintf (tmp, "r=%f,phi=%f", r, phi);
    info->setText (tmp);
}

// the rest just with a timer
void Window::timerEvent (QTimerEvent *) { updateGUI (); }
