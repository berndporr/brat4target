#pragma once

#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <vector>

class TargetDet
{
public:
    ~TargetDet();
    cv::QRCodeDetector qrDetector;

    // callback for the coordinate
    using OnDetected = std::function<void(const std::vector<cv::Point2f>&)>;

    const std::vector<cv::Point2f> detectSync(const cv::Mat img);

    void detectAsync(const cv::Mat img);

    void registerDetCallback(OnDetected cb)
    {
        onDetected = cb;
    }

    cv::Point2f calcCentre(std::vector<cv::Point2f>) const;

    std::atomic<bool> isDetecting = false;

private:
    OnDetected onDetected;
    std::thread detThread;
};
