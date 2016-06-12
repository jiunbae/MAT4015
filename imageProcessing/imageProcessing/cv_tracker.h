#pragma once

#include "opencv2\core\core.hpp"
#include "opencv2\highgui\highgui.hpp"
#include "opencv2\imgproc\imgproc.hpp"
#include "opencv2\video\video.hpp"

using namespace cv;

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

class cvTracker {
public:
	cvTracker(void);
	~cvTracker(void);

	void initilize(Mat, Rect, COLOR_MODEL);
	bool run(Mat, Rect&);

	Mat get_bp_image();

protected:
	TrackerParam param;
	MatND m_model3d;
	Mat m_model, m_backproj;
	Rect rect;
};
