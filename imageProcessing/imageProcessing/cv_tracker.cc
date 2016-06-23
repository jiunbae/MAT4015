#include "cv_tracker.h"
#include <iostream>

using namespace std;


Tracker::Tracker(void) {}
Tracker::~Tracker(void) {}

// initilize (img = first frame, rc = select tracking object, cModel = color model)
void Tracker::initilize(Mat img, Rect rc, COLOR_MODEL color_model)
{
	param.color_model = color_model;

	// my method
	{
		Mat hsv;
		cvtColor(img, hsv, CV_BGR2HSV);

		// show screen of color histogram each channels
		double * hists = myHistogram(hsv, rc);
		myShowHistogram(hists);
		this->objectHists = hists;
	}

	// openCV method
	{
		cvHistogram(img, rect = cvRect = myRect = rc);
	}
}

// object tracking
bool Tracker::run(Mat img)
{
	// my method
	{
		// mean shift
		Mat hsv;
		cvtColor(img, hsv, CV_BGR2HSV);
		Rect nRect = myRect, temp, bRect = myRect;
		double nX = 0, nY = 0, tW = 0, w;
		do {
			tW = 0, nX = 0; nY = 0; myRect = nRect;

			// set searching area
			temp = Rect(max(nRect.x - (int)param.search_range, 0), max(nRect.y - (int)param.search_range, 0),
				min(nRect.width + (int)param.search_range * 2, hsv.rows), min(nRect.height + (int)param.search_range * 2, hsv.cols));
			double sRatioWidth = temp.width / param.sampling, sRatioHeight = temp.height / param.sampling;
			for (int i = 0; i <= temp.width / 2; i+=sRatioWidth)
				for (int j = 0; j <= temp.height / 2; j+=sRatioHeight)
				{
					w = mySimilarity(hsv, Rect(temp.x + (temp.width / 2) + i, temp.y + (temp.height / 2) + j, nRect.width, nRect.height), this->objectHists);
					tW += w;
					nX += w * (temp.x + (temp.width / 2) + i);
					nY += w * (temp.y + (temp.height / 2) + j);
					if (i != 0 || j != 0)
					{
						w = mySimilarity(hsv, Rect(temp.x + (temp.width / 2) - i, temp.y + (temp.height / 2) - j, nRect.width, nRect.height), this->objectHists);
						tW += w;
						nX += w * (temp.x + (temp.width / 2) - i);
						nY += w * (temp.y + (temp.height / 2) - j);
					}
				}
			nX /= tW;
			nY /= tW;

			// get mean of w, if w > 0.25 make range narrow, or not make extend;
			double twRatio = tW / ((temp.width / sRatioWidth) * (temp.height / sRatioHeight));
			param.search_range *= (twRatio > (EXTEND_LIMIT) ? 1 - twRatio : (1 - EXTEND_LIMIT) +twRatio);

			nRect = Rect(max((int)nX - nRect.width / 2, 0), max((int)nY - nRect.height / 2, 0), myRect.width, myRect.height);
		} while (sqrt(pow(myRect.x - nRect.x, 2) + pow(myRect.y - nRect.y, 2)) > param.search_range);

		// resize object box.
		tW = 0;
		for (int i = nRect.width - (RECT_EXTEND_MAX / 2); i <= nRect.width + (RECT_EXTEND_MAX / 2); i += RECT_STEP)
			for (int j = nRect.height - (RECT_EXTEND_MAX / 2); j <= nRect.height + (RECT_EXTEND_MAX / 2); j += RECT_STEP)
			{
				w = mySimilarity(hsv, temp = Rect(nX, nY, i, j), this->objectHists);
				if (w > tW && i * j > RECT_SIZE_MIN * (rect.width * rect.height) && i * j < RECT_SIZE_MAX * (rect.width * rect.height))
				{
					tW = w;
					nRect = temp;
				}
			}

		// check how moved, extend or narrow
		// if moved long, range extend range, or not narrow range
		double moved = (sqrt(pow(bRect.x - nRect.x, 2) + pow(bRect.y - nRect.y, 2))/ param.search_range);
		param.search_range *= sqrt(moved);
		param.search_range = min(max(param.search_range, SEARCH_MIN), SEARCH_MAX);

		// update rect (object)
		bRect = myRect = nRect;

		// if search range is narrow enough, update model histogram,
		if (param.search_range == SEARCH_MIN)
		{
			double * nHists = myHistogram(hsv, nRect);
			for (int i = 0; i < param.hist_bins; ++i)
				this->objectHists[i] += (nHists[i] - objectHists[i]) * 0.05;
		}

		// back projection - show color histogram [ white is similar ]
		Mat imx = hsv.clone();
		for (int i = 0; i < hsv.cols; ++i)
			for (int j = 0; j < hsv.rows; ++j)
			{
				w = myHistogramValue(imx, i, j, this->objectHists);
				double pixel[] = { 255 * w, 255 * w, 255 * w };
				matrixSet(imx, i, j, pixel);
			}
		imshow(MY_HISTOGRAM_NAME, imx);

		// show tracking object rectangle of (0,255,0)
		rectangle(img, myRect, Scalar(0, 255, 0), 3, CV_AA);
	}

	// openCV method
	{
		// histogram backprojection
		// swith with color model, run backprojection
		Mat backproj;
		cvBackProject(img, backproj);

		if (cvMeanshift)
			//mean shift
			cvMeanShift(img, backproj, cvRect);
		else
			//cam shift
			cvCamShift(img, backproj, cvRect);
	}

	imshow(CV_TRACKER_NAME, img);
	return true;
}

double cv::Tracker::mySimilarity(const Mat& img, Rect rc, double * hists)
{
	double similarity = 0;

	double * nHists = myHistogram(img, rc);

	for (int i = 0; i < param.hist_bins; ++i)
		similarity += hists[i] * nHists[i];

	return similarity;
}

double * Tracker::myHistogram(const Mat& img, const Rect& rc)
{
	double * hists = (double*)calloc(param.hist_bins, sizeof(double)), hist_size = 256 / param.hist_bins;
	int eWidth = min(rc.x + rc.width, img.size().width),
		eHeight = min(rc.y + rc.height, img.size().height);
	for (int i = rc.x; i < eWidth; ++i)
		for (int j = rc.y; j < eHeight; ++j)
			hists[matrixAt(img, i, j) / (int)hist_size]++;

	double total = 0;
	for (int i = 0; i < param.hist_bins; ++i)
		total += hists[i] * hists[i];
	total = sqrt(total);
	for (int i = 0; i < param.hist_bins; ++i)
		hists[i] /= total;

	return hists;
}

void Tracker::myShowHistogram(double * hists)
{
	Mat histImg(param.hist_bins * HistRatio, param.hist_bins * HistRatio, CV_8U, Scalar(255));
	for (int i = 0; i < param.hist_bins; ++i)
	{
		double intensity = hists[i] * param.hist_bins / 1, j = HistRatio;
		while (j--)
			line(histImg, Point(i * HistRatio + j, param.hist_bins * HistRatio),
				Point(i * HistRatio + j, (param.hist_bins - intensity * HistRatio) * HistRatio), Scalar::all(0));
	}
	imshow("histogram", histImg);
}

void Tracker::matrixSet(Mat& img, int x, int y, double pixel[])
{
	uchar * data = img.data + y * img.step + x * img.elemSize();
	for (int k = 0; k < img.channels(); ++k)
		data[k] = pixel[k];
}

int Tracker::matrixAt(const Mat& img, int x, int y)
{
	if (x < 0 || y < 0 || x > img.size().width || y > img.size().height) return 0;
	int sum = 0;
	uchar * data = img.data + y*img.step + x * img.elemSize();
	for (int k = 0; k < img.channels(); ++k)
		sum += data[k] * this->param.channel_ratio[k];
	return sum;
}

double Tracker::myHistogramValue(const Mat& img, int x, int y, double * hists)
{
	return hists[matrixAt(img, x, y) / param.hist_bins];
}

void cv::Tracker::cvHistogram(const Mat & img, const Rect rc)
{
	Mat mask = Mat();
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
		switch (param.color_model)
		{
		case CM_GRAY:
		{
			Mat gray;
			cvtColor(img, gray, CV_BGR2GRAY);

			float vrange[] = { 0,256 };
			const float* phranges = vrange;
			Mat target(gray, rc);
			calcHist(&target, 1, 0, mask, model, 1, &param.hist_bins, &phranges);
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
			break;
		}
		}
	}
}

void Tracker::cvBackProject(const Mat& img, Mat& backproj)
{
	if (img.channels() <= 1)
	{
		float vrange[] = { 0,256 };
		const float* phranges = vrange;
		calcBackProject(&img, 1, 0, model, backproj, &phranges);
		return;
	}

	switch (param.color_model)
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

void Tracker::cvMeanShift(Mat& img, const Mat& backproj, Rect rc)
{
	meanShift(backproj, rc, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, param.max_itrs, 1));
	rectangle(img, rc, Scalar(0, 0, 255), 3, CV_AA);
}

void cv::Tracker::cvCamShift(Mat& img, const Mat& backproj, Rect rc)
{
	if (rc.width > 0 && rc.height > 0)
	{
		RotatedRect trackBox = CamShift(backproj, rc, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, param.max_itrs, 1));
		ellipse(img, trackBox, Scalar(0, 0, 255), 3, CV_AA);
	}

	if (rc.width <= 1 || rc.height <= 1)
	{
		int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5) / 6;
		rc = Rect(rc.x - r, rc.y - r, rc.width + 2 * r, rc.height + 2 * r) & Rect(0, 0, cols, rows);
	}
}
