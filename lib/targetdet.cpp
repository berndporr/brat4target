#include "targetdet.h"
#include "lib/debug.h"

TargetDet::~TargetDet ()
{
    if (detThread.joinable ())
        {
            detThread.join ();
        }
}

const std::vector<cv::Point2f> TargetDet::detectSync (const cv::Mat img)
{
    std::vector<cv::Point2f> points;
    if (img.empty ())
        {
            bratlogger.log("TargetDetSync: image empty.");
            return points;
        }
    bool r = false;
    r = qrDetector.detect (img, points);
    if (!r)
        {
            points.clear ();
            return points;
        }
    return points;
}

void TargetDet::detectAsync (const cv::Mat img)
{
    if (img.empty ())
        {
            bratlogger.log("TargetDetASync: image empty.");
            return;
        }
    if (isDetecting)
        return;
    if (detThread.joinable ())
        {
            detThread.join ();
        }
    detThread = std::thread (
        [&] (const cv::Mat imgThr) {
            isDetecting = true;
            auto pts = detectSync (imgThr);
            if ((pts.size () > 0) && (onDetected))
                {
                    onDetected (pts);
                }
            isDetecting = false;
        },
        img);
}
