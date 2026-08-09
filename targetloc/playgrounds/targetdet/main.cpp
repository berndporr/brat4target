#include "window.h"
#include <QApplication>
#include <opencv2/opencv.hpp>

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

	Libcam2OpenCVSettings settings;
	settings.cameraIndex = 0;
	settings.width = 1920;
	settings.height = 1080;
	cameraL.start(cm, settings);

	// execute the application
	const int r = app.exec();

	cameraL.stop();
	cm.stop();
	return r;
}
