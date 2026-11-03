#pragma once

#include <atomic>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

/**
 * Sync or async QR code detector.
 * Class allows async QR code detection so that if it takes too long it
 * can run in a different thread.
 */
class TargetDet
{
  public:
    ~TargetDet ();
    cv::QRCodeDetector qrDetector;

    /**
     * Callback for the contour surrounding the QR code
     */
    using OnDetected = std::function<void (const std::vector<cv::Point2f> &)>;

    /**
     * Synchronous detection of the QR code. Might take longer than the framerate.
     */
    const std::vector<cv::Point2f> detectSync (const cv::Mat img);

    /**
     * Async detection of the QR code. Calls then the callback with the contour.
     */
    void detectAsync (const cv::Mat img);

    /**
     * Registers callback which receives the contour of the QR code.
     */
    void registerDetCallback (OnDetected cb) { onDetected = cb; }

  private:
    std::atomic<bool> isDetecting = false;
    OnDetected onDetected;
    std::thread detThread;
};
