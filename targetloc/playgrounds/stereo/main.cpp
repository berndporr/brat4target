#include "window.h"
#include <QApplication>
#include <opencv2/opencv.hpp>
#include <libcamera/libcamera/camera_manager.h>

// Main program
int main(int argc, char *argv[])
{
	QApplication app(argc, argv);

	libcamera::CameraManager cm;
	cm.start();

	// create the window
	Window window;
	window.show();

	Libcam2OpenCV cameraL;
	cameraL.registerCallback([&](const cv::Mat &mat, const libcamera::ControlList &)
							 { 
								window.updateImageL(mat); });

	Libcam2OpenCV cameraR;
	cameraR.registerCallback([&](const cv::Mat &mat, const libcamera::ControlList &)
							 { 
								window.updateImageR(mat); });

	Libcam2OpenCVSettings settings;
	settings.width=1920;
	settings.height=1080;
	settings.cameraIndex = 0;
	cameraL.start(cm, settings);
	settings.cameraIndex = 1;
	cameraR.start(cm, settings);

	// execute the application
	const int r = app.exec();

	cameraL.stop();
	cameraR.stop();
	cm.stop();
	return r;
}
