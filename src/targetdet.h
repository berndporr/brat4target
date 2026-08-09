#pragma once
#include <atomic>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

class TargetDet
{
  public:
    ~TargetDet ();
    cv::QRCodeDetector qrDetector;

    enum DetectorType
    {
        QR
    };

    void setDetectorType (DetectorType dt) { detectorType = dt; }

    // callback for the coordinate
    using OnDetected = std::function<void (const std::vector<cv::Point2f> &)>;

    const std::vector<cv::Point2f> detectSync (const cv::Mat img);

    void detectAsync (const cv::Mat img);

    void registerDetCallback (OnDetected cb) { onDetected = cb; }

    cv::Point2f calcCentre (std::vector<cv::Point2f>) const;

    std::atomic<bool> isDetecting = false;

  private:
    DetectorType detectorType = QR;
    OnDetected onDetected;
    std::thread detThread;
};
