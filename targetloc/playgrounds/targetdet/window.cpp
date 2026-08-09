#include "window.h"

Window::Window()
{
	vLayout = new QVBoxLayout();
	imageL = new QLabel;
	vLayout->addWidget(imageL);
	targetInfo = new QLabel;
	vLayout->addWidget(targetInfo);
	setLayout(vLayout);
	targetDet.registerDetCallback([&](std::vector<cv::Point2f> pts){onQRdetected(pts);});
}

void Window::updateImageL(const cv::Mat &leftInput)
{
    cv::Mat greyImage;
    cv::cvtColor(leftInput, greyImage, cv::COLOR_BGR2GRAY);
    cv::Mat displayImage;
    cv::cvtColor(greyImage, displayImage, cv::COLOR_GRAY2BGR);
    cv::rectangle(displayImage,{(int)qrCoord.x-5,(int)qrCoord.y-5},{(int)qrCoord.x+5,(int)qrCoord.y+5},{0,255,0},3);
	const QImage frame(displayImage.data, displayImage.cols, displayImage.rows, displayImage.step,
					   QImage::Format_RGB888);
	imageL->setPixmap(QPixmap::fromImage(frame.scaledToWidth(displaywidth)));
	update();
	targetDet.detectAsync(greyImage);
	std::string text = "Coord: ";
	text = text + std::to_string(qrCoord.x) + "," + std::to_string(qrCoord.y);
	targetInfo->setText(text.c_str());
}
