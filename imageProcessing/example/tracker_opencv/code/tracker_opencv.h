///////////////////////////////////////////////////////////////////////
// OpenCV tracking example.
// Written by darkpgmr (http://darkpgmr.tistory.com), 2013

#pragma once

// include opencv
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/video.hpp"

using namespace cv;

enum {CM_GRAY, CM_HUE, CM_RGB, CM_HSV};	// color model

enum {MEANSHIFT, CAMSHIFT}; // method

struct tracker_opencv_param
{
	int hist_bins;
	int max_itrs;
	int color_model;
	int method;

	tracker_opencv_param()
	{
		hist_bins = 16;
		max_itrs = 10;
		color_model = CM_HSV;
		method = CAMSHIFT;
	}
};

class tracker_opencv
{
public:
	tracker_opencv(void);
	~tracker_opencv(void);

	void init(Mat img, Rect rc);
	bool run(Mat img, Rect& rc);

	void configure();
	Mat get_bp_image();

protected:
	Mat m_model;
	MatND m_model3d;
	Mat m_backproj;
	Rect m_rc;
	tracker_opencv_param m_param;
};
