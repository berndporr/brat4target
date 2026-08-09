#pragma once

#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>

class TargetDet
{
public:
    ~TargetDet();
    cv::QRCodeDetector qrDetector;
    cv::barcode::BarcodeDetector barcodeDetector;

    enum DetectorType {QR, Barcode};

    void setDetectorType(DetectorType dt) {
	detectorType = dt;
    }

    // callback for the coordinate
    using OnDetected = std::function<void(std::vector<cv::Point2f>)>;

    std::vector<cv::Point2f> detectSync(const cv::Mat img);

    void detectAsync(const cv::Mat img);

    void registerDetCallback(OnDetected cb) {
        onDetected = cb;
    }

    cv::Point2f calcCentre(std::vector<cv::Point2f>) const;

    std::atomic<bool> isDetecting = false;

private:
    DetectorType detectorType = QR;
    OnDetected onDetected;
    std::thread detThread;
};
