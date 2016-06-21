#include "cv_tracker.h"
#include <iostream>

using namespace std;

Tracker::Tracker(void) {}
Tracker::~Tracker(void) {}

// initilize (img = first frame, rc = select tracking object, cModel = color model)
void Tracker::initilize(Mat img, Rect rc, COLOR_MODEL cModel)
{
	if (my)
	{
		Mat roi(img, rc), hsv, histImg(param.hist_bins * HistRatio, param.hist_bins * HistRatio, CV_8U, Scalar(255));
		cvtColor(img, hsv, CV_BGR2HSV);

		// show screen of color histogram each channels
		double * hists = myHistogram(hsv, rc);
		for (int i = 0; i < param.hist_bins; ++i)
		{
			int intensity = static_cast<int>(hists[i] * param.hist_bins / 1), j = HistRatio;
			while (j--)
				line(histImg, Point(i * HistRatio + j, param.hist_bins * HistRatio),
					Point(i * HistRatio + j, (param.hist_bins - intensity * HistRatio) * HistRatio), Scalar::all(0));
		}
		imshow("histogram", histImg);
		rect = rc;
		return;
	}
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
	if (my)
	{
		// mean shift
		Rect nRect = rect, temp;
		int nX = 0, nY = 0, tW = 0, w;
		do {
			nX = 0; nY = 0; rect = nRect;

			// set searching area
			Rect temp = Rect(max(nRect.x - param.search_range, 0), max(nRect.y - param.search_range, 0),
				min(nRect.width + param.search_range * 2, img.rows), min(nRect.height + param.search_range * 2, img.cols));
			for (int i = temp.x; i < temp.x + temp.width; ++i)
				for (int j = temp.y; j < temp.y + temp.height; ++j)
				{
					double * nHists = myHistogram(img, temp);
					w = mySimilarity(img, i, j, nHists);
					tW += w;
					nX += w * i;
					nY += w * j;
				}
			nX /= tW;
			nX /= tW;

			nRect = Rect(max(nX - nRect.width / 2, 0), max(nY - nRect.height / 2, 0), rect.width, rect.height);
		} while (sqrt(pow(abs(rect.x - nRect.x), 2) + pow(abs(rect.y - nRect.y), 2)) > param.vector_size);

		// back projection
		rectangle(img, rect, Scalar(0, 255, 0), 3, CV_AA);


		return true;
	}
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
	if (1)
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

double cv::Tracker::mySimilarity(const Mat& img, int x, int y, double * hists)
{
	double similarity = 0;

	int left = max(x - rect.width / 2, 0), right = min(x + rect.width / 2, img.size().width);
	int down = max(y - rect.width / 2, 0), up = min(y + rect.width / 2, img.size().height);
	Rect rc(left, down, left + right, down + up);

	double * nHists = myHistogram(img, rc);
	for (int i = 0; i < param.hist_bins; ++i)
		similarity += sqrt(hists[i] * nHists[i]);

	return similarity;
}

double * Tracker::myHistogram(const Mat& img, const Rect& rc)
{
	double * hists = (double*)calloc(param.hist_bins, sizeof(double)), hist_size = 256 / param.hist_bins;

	for (int i = rc.x; i < rc.x + rc.width; ++i)
		for (int j = rc.y; j < rc.y + rc.height; ++j)
			hists[myHistogramAt(img, i, j) / (int)hist_size]++;

	int total = 0;
	for (int i = 0; i < param.hist_bins; ++i)
		total += hists[i] * hists[i];
	total = sqrt(total);
	for (int i = 0; i < param.hist_bins; ++i)
		hists[i] /= total;

	return hists;
}

int Tracker::myHistogramAt(const Mat& img, int x, int y)
{
	int sum = 0;
	uchar * value = img.data + y * img.step + x * img.elemSize();
	for (int k = 0; k < img.channels(); ++k)
		sum += value[k] * this->param.channel_ratio[k];
	return sum;
}

double Tracker::myHistogramValue(const Mat& img, int x, int y, double * hists)
{
	return hists[myHistogramAt(img, x, y) / param.hist_bins];
}
