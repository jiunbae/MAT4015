#include <iostream>
#include "opencv2\opencv.hpp"

using namespace std;
using namespace cv;

static string WINDOW_NAME = "TEST";

int main()
{
	cout << "OpenCV Version : " << CV_VERSION << endl;
	Mat img = imread("test.jpg", CV_LOAD_IMAGE_COLOR);

	namedWindow(WINDOW_NAME, CV_WINDOW_AUTOSIZE);

	if (img.empty())
	{
		cout << "[!] You can NOT see the cat!" << endl;
		return -1;
	}

	imshow(WINDOW_NAME, img);
	waitKey(0);
	destroyWindow(WINDOW_NAME);
	return 0;
}
