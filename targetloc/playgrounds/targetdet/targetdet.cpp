#include "targetdet.h"

TargetDet::~TargetDet()
{
    if (detThread.joinable())
    {
        detThread.join();
    }
}

std::vector<cv::Point2f> TargetDet::detectSync(const cv::Mat img)
{
    std::vector<cv::Point2f> points;
    if (img.empty())
    {
        fprintf(stderr, "TargetDetSync: image empty\n");
        return points;
    }
    bool r = false;
    switch (detectorType) {
    case QR:
	r = qrDetector.detect(img, points);
	break;
    case Barcode:
	r = barcodeDetector.detect(img, points);
	break;
    }
    if (!r)
    {
        points.clear();
        return points;
    }
    return points;
}

void TargetDet::detectAsync(const cv::Mat img)
{
    if (img.empty())
    {
        fprintf(stderr, "TargetDetASync: image empty\n");
        return;
    }
    if (isDetecting)
        return;
    if (detThread.joinable())
    {
        detThread.join();
    }
    detThread = std::thread([&](const cv::Mat imgThr)
                            {
                            isDetecting=true;
                            auto pts = detectSync(imgThr);
                            if ((pts.size()>0) && (onDetected))
                                onDetected(pts);
                            isDetecting=false; }, img);
}

cv::Point2f TargetDet::calcCentre(std::vector<cv::Point2f> points) const
{

    cv::Point2f centre{0, 0};
    for (const auto &p : points)
    {
        centre += p;
    }
    centre *= (1.0f / points.size());
    return centre;
}
