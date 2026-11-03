#pragma once

#include "libcam2opencv.h"
#include "targetdet.h"
#include <cmath>
#include <libcamera/libcamera/camera_manager.h>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <vector>

struct TargetLocSettings
{
    /** 
     * Camera width in pixels
     */
    int width = 1280;

    /**
     * Camera height in pixels
     */
    int height = 800;

    /**
     * Camera field of view. Here: 62.2 degrees.
     */
    float fieldOfView = 62.2 / 180.0 * M_PI;

    /**
     * Factor mapping from QR code size to distance.
     * Distance is inferred from the height of the QR code.
     * QR code height must be 8cm.
     */
    float QRcodeYsize2distance = 0.108;
};

/**
 * Detects a QR code and calls a callback with its angle and distance.
 */
class TargetLoc
{
  public:
    TargetLoc () = default;

    /**
     * Callback interface which reports a new target location.
     */
    using DetectionEvent
        = std::function<void (const float r, const float phi)>;
    void registerDetectionEvent (DetectionEvent de) { detectionEvent = de; }

    /**
     * Starts the camera
     */
    void start (TargetLocSettings settings = TargetLocSettings ());

    /**
     * Stops the camera
     */
    void stop ();

    /**
     * For debugging purposes we can get the current left camera
     * image.
     */
    const cv::Mat getCurrentCameraImage ()
    {
        std::lock_guard<std::mutex> guard (image_mutex);
        return current;
    }

    /**
     * Gets the countour around the detected target.
     */
    const std::vector<cv::Point> getQRcodeContour ()
    {
        std::lock_guard<std::mutex> guard (contour_mutex);
        if (contoursRingbuffer.size () > 0)
        {
            std::vector<cv::Point> p;
            for (auto const &v : contoursRingbuffer[0])
            {
                p.push_back ({ (int)v.x, (int)v.y });
            }
            return p;
        }
        return std::vector<cv::Point> ();
    }

  private:
    // current camera image
    cv::Mat current;

    // Callback when a target has been detected
    void onTargetDetected (const std::vector<cv::Point2f> &contour);

    // Helper which maps libcamera to openCV
    Libcam2OpenCV camera;

    // Cameramanager for both cameras
    libcamera::CameraManager cm;

    // callback from libcamera with an image
    void updateImage (const cv::Mat &l);

    // keeping image data stable while processing
    std::mutex image_mutex;
    std::mutex contour_mutex;

    // ringbuffer of countours to check if they are consistently det
    std::deque<std::vector<cv::Point2f> > contoursRingbuffer;

    // Detection Callback
    DetectionEvent detectionEvent;

    // Cartesian distance between two points
    float point2point (cv::Point2f a, cv::Point2f b)
    {
        const float dx = a.x - b.x;
        const float dy = a.y - b.y;
        return sqrt (dx * dx + dy * dy);
    }

    // Max error in pixels between consecutive contours
    const float maxContourPixelErrorBetweenDetectionContours = 10;

    TargetDet targetDet;

    TargetLocSettings targetLocSettings;
};
