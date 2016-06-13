#include "cv_window.h"

cv::Window::Window(String name)
{
	this->name = name;
}

cv::Window::~Window() {}

void cv::Window::setImage(Mat img)
{
	imshow(name, img);
}

void cv::Window::setMouseCallback(MouseCallback callback)
{
	cv::setMouseCallback(name, callback, &param);
}
