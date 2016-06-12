#include "tracker.h"

Tracker::Tracker(void) {}

Tracker::~Tracker(void) {}

void Tracker::initilize(Mat img, Rect rc, COLOR_MODEL model)
{
	int vmin = 10, vmax = 256, smin = 30;
	Mat hsv, mask, hue, hist, histimg;
	cvtColor(img, hsv, COLOR_BGR2HSV);

	inRange(hsv, Scalar(0, smin, MIN(vmin, vmax)), Scalar(180, 256, MAX(vmin, vmax)), mask);

	int ch[] = { 0, 0 };
	//hue.create(hsv.size(), hsv.depth);
	mixChannels(&hsv, 1, &hue, 1, ch, 1);

	Mat roi(hue, rc), maskroi(mask, rc);
	float hranges[] = { 0, 180 };
	const float* phranges = hranges;
	calcHist(&roi, 1, 0, maskroi, hist, 1, &param.hist_bins, &phranges);
	normalize(hist, hist, 0, 255, NORM_MINMAX);

}
