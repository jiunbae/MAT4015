#include "tracker.h"

Tracker::Tracker(void) {}
Tracker::~Tracker(void) {}

void Tracker::initilize(Mat img, Rect rc, COLOR_MODEL cModel)
{
	Mat mask = Mat::zeros(rc.height, rc.width, CV_8U);
	ellipse(mask, Point(rc.width / 2, rc.height / 2), Size(rc.width / 2, rc.height / 2), 0, 0, 360, 255, CV_FILLED);
	param.color_model = cModel;


	if (img.channels() <= 1)
	{
		float vrange[] = { 0,256 };
		const float* phranges = vrange;
		Mat roi(img, rc);
		calcHist(&roi, 1, 0, mask, model, 1, &param.hist_bins, &phranges);
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
				Mat target(gray, rc);
				calcHist(&target, 1, 0, mask, model, 1, &param.hist_bins, &phranges);
				imshow("applied image", gray);
				imshow("applied image_target", target);
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
				imshow("applied image", hsv);
				imshow("applied image_target", target);
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
				imshow("applied image", img);
				imshow("applied image_target", target);
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
				imshow("applied image", hsv);
				imshow("applied image_target", target);
				break;
			}
		}
	}

	rect = rc;
}

void Tracker::run(Mat img, Rect& rc)
{
	if (img.channels() <= 1)
	{
		float vrange[] = { 0, 256 };
		const float * phranges = vrange;
		calcBackProject(&img, 1, 0, model, backproj, &phranges);
	}
	else
	{
		switch (param.color_model)
		{
			case CM_GRAY:
				{
					break;
				}
			case CM_HUE:
				{
					break;
				}
			case CM_RGB:
				{
					break;
				}
			case CM_HSV:
				{
					break;
				}
		}
	}
}
Mat Tracker::get_image()
{
	normalize(backproj, backproj, 0, 255, CV_MINMAX);
	return backproj;
}