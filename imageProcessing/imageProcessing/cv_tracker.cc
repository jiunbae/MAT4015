#include "cv_tracker.h"
#include <iostream>

using namespace std;

cvTracker::cvTracker(void) {}
cvTracker::~cvTracker(void) {}

void cvTracker::initilize(Mat img, Rect rc, COLOR_MODEL model)
{
	Mat mask = Mat::zeros(rc.height, rc.width, CV_8U);
	ellipse(mask, Point(rc.width / 2, rc.height / 2), Size(rc.width / 2, rc.height / 2), 0, 0, 360, 255, CV_FILLED);
	param.color_model = model;

	if (img.channels() <= 1)
	{
		float vrange[] = { 0,256 };
		const float* phranges = vrange;
		Mat roi(img, rc);
		calcHist(&roi, 1, 0, mask, m_model, 1, &param.hist_bins, &phranges);
	}
	else
	{
		switch (param.color_model)
		{
		case CM_GRAY:
			{
				Mat gray;
				cvtColor(img, gray, CV_BGR2GRAY);

				float vrange[] = { 0,256 };
				const float* phranges = vrange;
				Mat roi(gray, rc);
				calcHist(&roi, 1, 0, mask, m_model, 1, &param.hist_bins, &phranges);
				break;
			}
		case CM_HUE:
			{
				Mat hsv;
				cvtColor(img, hsv, CV_BGR2HSV);

				float hrange[] = { 0,180 };
				const float* phranges = hrange;
				int channels[] = { 0 };
				Mat roi(hsv, rc);
				calcHist(&roi, 1, channels, mask, m_model, 1, &param.hist_bins, &phranges);
				break;
			}
		case CM_RGB:
			{
				float vrange[] = { 0,255 };
				const float* ranges[] = { vrange, vrange, vrange };
				int channels[] = { 0, 1, 2 };
				int hist_sizes[] = { param.hist_bins, param.hist_bins, param.hist_bins };
				Mat roi(img, rc);
				calcHist(&roi, 1, channels, mask, m_model3d, 3, hist_sizes, ranges);
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
				Mat roi(hsv, rc);
				calcHist(&roi, 1, channels, mask, m_model3d, 3, hist_sizes, ranges);
				break;
			}
		}
	}

	rect = rc;
}

bool cvTracker::run(Mat img, Rect& rc)
{
	Mat mask = Mat::zeros(rc.height, rc.width, CV_8U);
	ellipse(mask, Point(rc.width / 2, rc.height / 2), Size(rc.width / 2, rc.height / 2), 0, 0, 360, 255, CV_FILLED);

	// histogram backprojection
	if (img.channels() <= 1)
	{
		float vrange[] = { 0,256 };
		const float* phranges = vrange;
		calcBackProject(&img, 1, 0, m_model, m_backproj, &phranges);
	}
	else
	{
		switch (param.color_model)
		{
		case CM_GRAY:
			{
				Mat gray;
				cvtColor(img, gray, CV_BGR2GRAY);

				float vrange[] = { 0,256 };
				const float* phranges = vrange;
				calcBackProject(&gray, 1, 0, m_model, m_backproj, &phranges);
				break;
			}
		case CM_HUE:
			{
				Mat hsv;
				cvtColor(img, hsv, CV_BGR2HSV);

				float hrange[] = { 0,180 };
				const float* phranges = hrange;
				int channels[] = { 0 };
				calcBackProject(&hsv, 1, channels, m_model, m_backproj, &phranges);
				break;
			}
		case CM_RGB:
			{
				float vrange[] = { 0,255 };
				const float* ranges[] = { vrange, vrange, vrange };	// B,G,R
				int channels[] = { 0, 1, 2 };
				int hist_sizes[] = { param.hist_bins, param.hist_bins, param.hist_bins };
				calcBackProject(&img, 1, channels, m_model3d, m_backproj, ranges);
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
				calcBackProject(&hsv, 1, channels, m_model3d, m_backproj, ranges);
				break;
			}
		}
	}
	m_backproj &= mask;
	RotatedRect trackBox = CamShift(m_backproj, rect, TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 10, 1));

	if (rect.area() <= 1)
	{
		int cols = m_backproj.cols, rows = m_backproj.rows, r = (MIN(cols, rows) + 5) / 6;
		rect = Rect(rect.x - r, rect.y - r, rect.x + r, rect.y + r) & Rect(0, 0, cols, rows);
	}

	ellipse(img, trackBox, Scalar(0, 0, 255), 3, LINE_AA);

	// tracking - mean shift
	//int itrs = meanShift(m_backproj, rect, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, param.max_itrs, 1));
	//rectangle(img, rect, Scalar(0, 255, 0), 3, CV_AA);

	// tracking - cam shift
	/*if (rect.width>0 && rect.height>0)
	{
		RotatedRect trackBox = CamShift(m_backproj, rect, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, param.max_itrs, 1));
		ellipse(img, trackBox, Scalar(0, 0, 255), 3, CV_AA);
	}

	if (rect.width <= 1 || rect.height <= 1)
	{
		int cols = m_backproj.cols, rows = m_backproj.rows, r = (MIN(cols, rows) + 5) / 6;
		rect = Rect(rect.x - r, rect.y - r, rect.width + 2 * r, rect.height + 2 * r) & Rect(0, 0, cols, rows);
	}*/

	rc = rect;
	return true;
}

Mat cvTracker::get_bp_image()
{
	normalize(m_backproj, m_backproj, 0, 255, CV_MINMAX);
	return m_backproj;
}