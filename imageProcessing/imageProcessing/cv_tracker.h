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
	// color model enum
	enum COLOR_MODEL { CM_HSV, CM_RGB, CM_HUE, CM_GRAY };

	// tracker parameter
	struct TrackerParam
	{
		COLOR_MODEL cModel;
		double channel_ratio[3] = { 0.8, 0.1, 0.1 };
		int hist_bins, max_itrs;
		int search_range, sampling;
		int vector_size;

		TrackerParam()
		{
			hist_bins = 16;
			max_itrs = 32;
			search_range = 32;
			sampling = 4;
			vector_size = 8;
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
		double * objectHists;
		char HistRatio = 8;
		bool my = true;

		void matrixSet(Mat&, int, int, double[]);
		int matrixAt(const Mat&, int x, int y);

	private:
		void myShowHistogram(double *);
		double * myHistogram(const Mat&, const Rect&);
		double myHistogramValue(const Mat&, int x, int y, double*);
		double mySimilarity(const Mat&, int x, int y, double*);
	};
}
#endif