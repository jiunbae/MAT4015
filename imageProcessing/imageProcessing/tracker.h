#pragma once

#include "opencv2\core\core.hpp"
#include "opencv2\highgui\highgui.hpp"
#include "opencv2\imgproc\imgproc.hpp"
#include "opencv2\video\video.hpp"

using namespace cv;

#ifndef TRACKER
#define TRACKER

enum COLOR_MODEL { CM_HSV, CM_RGB, CM_HUE, CM_GRAY };

struct TrackerParam
{
	int hist_bins;
	int max_itrs;
	COLOR_MODEL color_model;

	TrackerParam()
	{
		hist_bins = 16;
		max_itrs = 16;
		color_model = CM_HSV;
	}
};

class Tracker {
public:
	Tracker(void);
	~Tracker(void);

	void initilize(Mat, Rect, COLOR_MODEL);
	bool run(Mat, Rect&);

	Mat get_image();

protected:
	TrackerParam param;
	MatND model3d;
	Mat model, backproj;
	Rect rect;
};

#endif