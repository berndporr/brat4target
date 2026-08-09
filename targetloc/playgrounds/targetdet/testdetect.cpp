#include "targetdet.h"

#include <opencv2/opencv.hpp>
#include <functional>
#include <future>

using namespace std::chrono_literals;

// We define a "promise" here which has initially no
// value but will be set by the callback.
// In the main program we can then wait for its
// value with a timeout.
std::promise<std::vector<cv::Point2f>> promisedQRcoord;

// callback which eventually sets promisedQRcoord
void onDetect(std::vector<cv::Point2f> points)
{
    fprintf(stderr,"Callback!\n");
    promisedQRcoord.set_value(points);
}

int main(int argc, char *argv[])
{
    cv::Mat img = cv::imread("QRCode.png");

    if (img.empty())
    {
        std::cerr << "Image load failed\n";
        throw;
    }

    const std::vector<cv::Point2f> expectedPoints = { {23, 34}, {690, 29}, {704, 704}, {30, 692} };

    // test the sync
    TargetDet targetDet;
    auto pos1 = targetDet.detectSync(img);
    std::cerr << "Pos: " << pos1 << std::endl;
    if (pos1 != expectedPoints) {
        fprintf(stderr,"Detected coordinates wrong.\n");
        throw;
    }

    // test the async
    targetDet.registerDetCallback([&](std::vector<cv::Point2f> points){onDetect(points);});
    // we start the background thread
    targetDet.detectAsync(img);

    // We wait now for the callback to happen by getting a "future" from the "promise"
    // which will then contain the result from the callback.
    auto ourFuture = promisedQRcoord.get_future();
    // We wait for max 1sec for the future to receive a value from the callback!
    if (std::future_status::timeout == ourFuture.wait_for(1s)) {
        fprintf(stderr,"Callback never called. Timeout!\n");
        throw;
    }
    // Getting the value of the future.
    auto pos2 = ourFuture.get();
    std::cerr << "Pos: " << pos2 << std::endl;
    if (pos2 != expectedPoints) {
        fprintf(stderr,"Detected coordinates wrong.\n");
        throw;
    }

    if (pos1 != pos2) {
        fprintf(stderr,"QR code bounding boxes different between async and sync.\n");
        throw;
    }

    auto pos3 = targetDet.calcCentre(pos2);
    std::cerr << "Centre pos: " << pos3 << std::endl;

    return 0;
}