#include "targetloc.h"
#include <cmath>
#include <libcamera/libcamera/camera_manager.h>
#include <opencv2/core/types.hpp>

void TargetLoc::start ()
{
    cm.start ();

    camera.registerCallback (
        [&] (const cv::Mat &left, const libcamera::ControlList &) {
            updateImage (left);
        });

    targetDet.registerDetCallback (
        [&] (const std::vector<cv::Point2f> &coords) {
            onTargetDetected (coords);
        });

    settings.width = 1640;
    settings.height = 1232;
    settings.cameraIndex = 0;
    camera.start (cm, settings);
}

void TargetLoc::stop ()
{
    camera.stop ();
    cm.stop ();
}

void TargetLoc::updateImage (const cv::Mat &l)
{
    image_mutex.lock ();
    current = l;
    image_mutex.unlock ();
    targetDet.detectAsync (l);
}

// here it's where it's getting interesting!
void TargetLoc::onTargetDetected (const std::vector<cv::Point2f> &contour)
{
    std::lock_guard<std::mutex> guard (contour_mutex);
    // We put the detection contours into a double ended queue of 3
    // and check if all the detection points are within an error margin
    // of maxContourPixelErrorBetweenDetectionContours.
    contoursRingbuffer.push_front (contour);
    if (contoursRingbuffer.size () < 3)
    {
        return;
    }
    contoursRingbuffer.pop_back ();
    for (int i = 1; i < 3; i++)
    {
        std::vector<cv::Point2f> contour1 = contoursRingbuffer[i - 1];
        std::vector<cv::Point2f> contour2 = contoursRingbuffer[i];
        for (unsigned long int j = 0;
             (j < contour1.size ()) && (j < contour2.size ()); j++)
        {
            if (point2point (contour1[j], contour2[j])
                > maxContourPixelErrorBetweenDetectionContours)
            {
                fprintf (stderr, "Contour discarded.\n");
                return;
            }
        }
    }

    int i = 0;
    float avgX = 0;
    float minX = 1;
    float maxX = 0;
    float minY = 1;
    float maxY = 0;
    for (auto &c : contour)
    {
        const float x = c.x / settings.width;
        const float y = c.x / settings.height;
        avgX = avgX + x;
        i++;
        if (x > maxX)
            maxX = x;
        if (x < minX)
            minX = x;
        if (y > maxY)
            maxY = y;
        if (y < minY)
            minY = y;
    }
    avgX = (avgX / (float)i) - 0.5;
    //const float phi = avgX * fieldOfView;
    const float phi = -atan (avgX * 2 * tan (fieldOfView / 2));
    float r = 1E38;
    if (maxY != minY)
    {
        r = QRcodeYsize2distance / (maxY - minY);
    }
    // printf ("avgX = %f, phi=%f, minX=%f, maxX=%f, minY=%f, maxY=%f, r = %f\n",
    //        avgX, phi, minX, maxX, minY, maxY, r);

    // callback!
    if (detectionEvent)
    {
        detectionEvent (r, phi);
    }
}
