#include "window.h"

Window::Window()
{
	vLayout = new QVBoxLayout();

	h1Layout = new QHBoxLayout();
	imageL = new QLabel;
	imageR = new QLabel;
	h1Layout->addWidget(imageL);
	h1Layout->addWidget(imageR);
	vLayout->addLayout(h1Layout);

	h2Layout = new QHBoxLayout();
	imageCombined = new QLabel;
	h2Layout->addWidget(imageCombined);
	imageDisparity = new QLabel;
	h2Layout->addWidget(imageDisparity);
	vLayout->addLayout(h2Layout);

	setLayout(vLayout);

	stereo.registerCallback([&](cv::Mat d){currentD = d; refreshDisparity = true;});
}

void Window::updateImageL(const cv::Mat &leftInput)
{
	cv::resize(leftInput, currentL, imageSize);
	const QImage frame(currentL.data, currentL.cols, currentL.rows, currentL.step,
					   QImage::Format_BGR888);
	imageL->setPixmap(QPixmap::fromImage(frame));
	blendLRandDisplayD();
}

void Window::updateImageR(const cv::Mat &rightInput)
{
	cv::resize(rightInput, currentR, imageSize);
	const QImage frame(currentR.data, currentR.cols, currentR.rows, currentR.step,
					   QImage::Format_BGR888);
	imageR->setPixmap(QPixmap::fromImage(frame));
	blendLRandDisplayD();
}

void Window::blendLRandDisplayD()
{
	if (currentL.empty())
		return;
	if (currentR.empty())
		return;
	if (currentL.size != currentR.size)
		return;
	cv::Mat blended;
	cv::addWeighted(currentL, 0.5, currentR, 0.5, 0.0, blended);
	const QImage frame(blended.data, blended.cols, blended.rows, blended.step,
					   QImage::Format_BGR888);
	imageCombined->setPixmap(QPixmap::fromImage(frame));

	// trigger async calc
	stereo.calcDepthMapAsync(currentL,currentR);

	// however we can put that here in the callback as QT won't like it so we basically
	// "poll" it.
	if (!refreshDisparity) return;
	cv::Mat disp8;
	cv::normalize(currentD, disp8, 0, 255, cv::NORM_MINMAX, CV_8U);
	const QImage dispImage(disp8.data, disp8.cols, disp8.rows, disp8.step,
					   QImage::Format_Grayscale8);
	imageDisparity->setPixmap(QPixmap::fromImage(dispImage));
	refreshDisparity = false;
}

