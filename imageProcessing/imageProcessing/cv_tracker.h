#pragma once

#ifndef __CV__TRACKER__
#define __CV__TRACKER__

#include "opencv2\core\core.hpp"
#include "opencv2\highgui\highgui.hpp"
#include "opencv2\imgproc\imgproc.hpp"
#include "opencv2\video\video.hpp"

using namespace cv;

#define APPLIED_NAME "applied"
#define APPLIED_TARGET_NAME "applied target"

namespace cv {
	enum COLOR_MODEL { CM_HSV, CM_RGB, CM_HUE, CM_GRAY };

	struct TrackerParam
	{
		int hist_bins;
		int max_itrs;
		COLOR_MODEL cModel;

		TrackerParam()
		{
			hist_bins = 32;
			max_itrs = 32;
			cModel = CM_HSV;
		}
	};

	class Tracker {
	public:
		Tracker();
		~Tracker();

		void initilize(Mat, Rect, COLOR_MODEL);
		bool run(Mat);

		Mat get_image();

	protected:
		TrackerParam param;
		MatND model3d;
		Mat model, backproj, maskproj;
		Rect rect;
	};
}
#endif