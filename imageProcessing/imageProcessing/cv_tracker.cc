#include "cv_tracker.h"
#include <iostream>

using namespace std;

Tracker::Tracker(void) {}
Tracker::~Tracker(void) {}

// initilize (img = first frame, rc = select tracking object, cModel = color model)
void Tracker::initilize(Mat img, Rect rc, COLOR_MODEL cModel)
{
	// this is mask for calc histogram
	Mat mask = Mat();
	param.cModel = cModel;

	if (img.channels() <= 1)
	{
		float vrange[] = { 0, 256 };
		const float* phranges = vrange;
		Mat roi(img, rc);
		calcHist(&roi, 1, 0, mask, model, 1, &param.hist_bins, &phranges);
	}
	// switch with color model
	// each switch run calcHist with colormodel
	else
	{
		switch (param.cModel)
		{
			case CM_GRAY:
				{
					Mat gray;
					cvtColor(img, gray, CV_BGR2GRAY);

					float vrange[] = { 0,256 };
					const float* phranges = vrange;
					Mat target(gray, rc);
					calcHist(&target, 1, 0, mask, model, 1, &param.hist_bins, &phranges);
					imshow(APPLIED_NAME, gray);
					imshow(APPLIED_TARGET_NAME, target);
					break;
				}
			case CM_HUE:
				{
					Mat hsv;
					cvtColor(img, hsv, CV_BGR2HSV);

					float hrange[] = { 0,180 };
					const float* phranges = hrange;
					int channels[] = { 0 };
					Mat target(hsv, rc);
					calcHist(&target, 1, channels, mask, model, 1, &param.hist_bins, &phranges);
					imshow(APPLIED_NAME, hsv);
					imshow(APPLIED_TARGET_NAME, target);
					break;
				}
			case CM_RGB:
				{
					float vrange[] = { 0,255 };
					const float* ranges[] = { vrange, vrange, vrange };
					int channels[] = { 0, 1, 2 };
					int hist_sizes[] = { param.hist_bins, param.hist_bins, param.hist_bins };
					Mat target(img, rc);
					calcHist(&target, 1, channels, mask, model3d, 3, hist_sizes, ranges);
					imshow(APPLIED_NAME, img);
					imshow(APPLIED_TARGET_NAME, target);
					break;
				}
			case CM_HSV:
				{
					Mat hsv;
					cvtColor(img, hsv, CV_BGR2HSV);

					float hrange[] = { 0,180 };
					float vrange[] = { 0,255 };
					const float* ranges[] = { hrange, vrange, vrange };
					int channels[] = { 0, 1, 2 };
					int hist_sizes[] = { param.hist_bins, param.hist_bins, param.hist_bins };
					Mat target(hsv, rc);
					calcHist(&target, 1, channels, mask, model3d, 3, hist_sizes, ranges);
					imshow(APPLIED_NAME, hsv);
					imshow(APPLIED_TARGET_NAME, target);
					break;
				}
		}
	}

	maskproj = mask;
	rect = rc;
}

// object tracking
bool Tracker::run(Mat img)
{
	// histogram backprojection
	if (img.channels() <= 1)
	{
		float vrange[] = { 0,256 };
		const float* phranges = vrange;
		calcBackProject(&img, 1, 0, model, backproj, &phranges);
	}
	// swith with color model, run backprojection
	else
	{
		switch (param.cModel)
		{
		case CM_GRAY:
			{
				Mat gray;
				cvtColor(img, gray, CV_BGR2GRAY);

				float vrange[] = { 0,256 };
				const float* phranges = vrange;
				calcBackProject(&gray, 1, 0, model, backproj, &phranges);
				break;
			}
		case CM_HUE:
			{
				Mat hsv;
				cvtColor(img, hsv, CV_BGR2HSV);

				float hrange[] = { 0,180 };
				const float* phranges = hrange;
				int channels[] = { 0 };
				calcBackProject(&hsv, 1, channels, model, backproj, &phranges);
				break;
			}
		case CM_RGB:
			{
				float vrange[] = { 0,255 };
				const float* ranges[] = { vrange, vrange, vrange };	// B,G,R
				int channels[] = { 0, 1, 2 };
				int hist_sizes[] = { param.hist_bins, param.hist_bins, param.hist_bins };
				calcBackProject(&img, 1, channels, model3d, backproj, ranges);
				break;
			}
		case CM_HSV:
			{
				Mat hsv;
				cvtColor(img, hsv, CV_BGR2HSV);

				float hrange[] = { 0,180 };
				float vrange[] = { 0,255 };
				const float* ranges[] = { hrange, vrange, vrange };	// hue, saturation, brightness
				int channels[] = { 0, 1, 2 };
				calcBackProject(&hsv, 1, channels, model3d, backproj, ranges);
				break;
			}
		}
	}

	//mean shift
	if (0)
	{
		int itrs = meanShift(backproj, rect, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, param.max_itrs, 1));
		rectangle(img, rect, Scalar(0, 255, 0), 3, CV_AA);
	}
	//cam shift
	else
	{
		if (rect.width > 0 && rect.height > 0)
		{
			RotatedRect trackBox = CamShift(backproj, rect, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, param.max_itrs, 1));
			ellipse(img, trackBox, Scalar(0, 0, 255), 3, CV_AA);
		}

		if (rect.width <= 1 || rect.height <= 1)
		{
			int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5) / 6;
			rect = Rect(rect.x - r, rect.y - r, rect.width + 2 * r, rect.height + 2 * r) & Rect(0, 0, cols, rows);
		}
	}

	return true;
}

Mat Tracker::get_image()
{
	Mat ret;
	normalize(backproj, ret, 0, 255, CV_MINMAX);
	return ret;
}