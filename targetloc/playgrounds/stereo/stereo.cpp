#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "stereo.h"

cv::Mat Stereo::rectifyLeft(const cv::Mat &left)
{
    return left;
}

cv::Mat Stereo::rectifyRight(const cv::Mat &right)
{
    return right;
}

cv::Mat Stereo::calcDepthMapSync(const cv::Mat &left, const cv::Mat &right)
{
    cv::Mat disparity;
    stereoMatcher->setP1(8 * left.channels() * 5 * 5);
    stereoMatcher->setP2(32 * left.channels() * 5 * 5);
    stereoMatcher->compute(left, right, disparity);
        //    fprintf(stderr,"Disp calc finished.\n");
    return disparity;
}

void Stereo::calcDepthMapAsync(const cv::Mat &left, const cv::Mat &right)
{
    if (isCalculatingDisparity)
        return;
    if (disparityCalcThread.joinable())
    {
        disparityCalcThread.join();
    }
    disparityCalcThread = std::thread([&](const cv::Mat &leftThr, const cv::Mat &rightThr)
                                      {     
                                        isCalculatingDisparity = true;
                                        const cv::Mat d = calcDepthMapSync(leftThr, rightThr);
					if (onDisparity) {
					    onDisparity(d);
					}
                                        isCalculatingDisparity = false; }, left, right);
}

Stereo::~Stereo()
{
    if (disparityCalcThread.joinable())
    {
        disparityCalcThread.join();
    }
}
