#include "tracker.h"
#include <iostream>

using namespace std;

tracker_opencv::tracker_opencv(void)
{
}

tracker_opencv::~tracker_opencv(void)
{
}

void tracker_opencv::init(Mat img, Rect rc)
{
	Mat mask = Mat::zeros(rc.height, rc.width, CV_8U);
	ellipse(mask, Point(rc.width / 2, rc.height / 2), Size(rc.width / 2, rc.height / 2), 0, 0, 360, 255, CV_FILLED);

	if (img.channels() <= 1)
	{
		float vrange[] = { 0,256 };
		const float* phranges = vrange;
		Mat roi(img, rc);
		calcHist(&roi, 1, 0, mask, m_model, 1, &m_param.hist_bins, &phranges);
	}
	else if (m_param.color_model == CM_GRAY)
	{
		Mat gray;
		cvtColor(img, gray, CV_BGR2GRAY);

		float vrange[] = { 0,256 };
		const float* phranges = vrange;
		Mat roi(gray, rc);
		calcHist(&roi, 1, 0, mask, m_model, 1, &m_param.hist_bins, &phranges);
	}
	else if (m_param.color_model == CM_HUE)
	{
		Mat hsv;
		cvtColor(img, hsv, CV_BGR2HSV);

		float hrange[] = { 0,180 };
		const float* phranges = hrange;
		int channels[] = { 0 };
		Mat roi(hsv, rc);
		calcHist(&roi, 1, channels, mask, m_model, 1, &m_param.hist_bins, &phranges);
	}
	else if (m_param.color_model == CM_RGB)
	{
		float vrange[] = { 0,255 };
		const float* ranges[] = { vrange, vrange, vrange };	// B,G,R
		int channels[] = { 0, 1, 2 };
		int hist_sizes[] = { m_param.hist_bins, m_param.hist_bins, m_param.hist_bins };
		Mat roi(img, rc);
		calcHist(&roi, 1, channels, mask, m_model3d, 3, hist_sizes, ranges);
	}
	else if (m_param.color_model == CM_HSV)
	{
		Mat hsv;
		cvtColor(img, hsv, CV_BGR2HSV);

		float hrange[] = { 0,180 };
		float vrange[] = { 0,255 };
		const float* ranges[] = { hrange, vrange, vrange };	// hue, saturation, brightness
		int channels[] = { 0, 1, 2 };
		int hist_sizes[] = { m_param.hist_bins, m_param.hist_bins, m_param.hist_bins };
		Mat roi(hsv, rc);
		calcHist(&roi, 1, channels, mask, m_model3d, 3, hist_sizes, ranges);
	}

	m_rc = rc;
}

bool tracker_opencv::run(Mat img, Rect& rc)
{
	// elliptic mask
	Mat mask = Mat::zeros(rc.height, rc.width, CV_8U);
	ellipse(mask, Point(rc.width / 2, rc.height / 2), Size(rc.width / 2, rc.height / 2), 0, 0, 360, 255, CV_FILLED);

	// histogram backprojection
	if (img.channels() <= 1)
	{
		float vrange[] = { 0,256 };
		const float* phranges = vrange;
		calcBackProject(&img, 1, 0, m_model, m_backproj, &phranges);
	}
	else if (m_param.color_model == CM_GRAY)
	{
		Mat gray;
		cvtColor(img, gray, CV_BGR2GRAY);

		float vrange[] = { 0,256 };
		const float* phranges = vrange;
		calcBackProject(&gray, 1, 0, m_model, m_backproj, &phranges);
	}
	else if (m_param.color_model == CM_HUE)
	{
		Mat hsv;
		cvtColor(img, hsv, CV_BGR2HSV);

		float hrange[] = { 0,180 };
		const float* phranges = hrange;
		int channels[] = { 0 };
		calcBackProject(&hsv, 1, channels, m_model, m_backproj, &phranges);
	}
	else if (m_param.color_model == CM_RGB)
	{
		float vrange[] = { 0,255 };
		const float* ranges[] = { vrange, vrange, vrange };	// B,G,R
		int channels[] = { 0, 1, 2 };
		int hist_sizes[] = { m_param.hist_bins, m_param.hist_bins, m_param.hist_bins };
		calcBackProject(&img, 1, channels, m_model3d, m_backproj, ranges);
	}
	else if (m_param.color_model == CM_HSV)
	{
		Mat hsv;
		cvtColor(img, hsv, CV_BGR2HSV);

		float hrange[] = { 0,180 };
		float vrange[] = { 0,255 };
		const float* ranges[] = { hrange, vrange, vrange };	// hue, saturation, brightness
		int channels[] = { 0, 1, 2 };
		calcBackProject(&hsv, 1, channels, m_model3d, m_backproj, ranges);
	}

	// tracking
	if (m_param.method == MEANSHIFT)
	{
		int itrs = meanShift(m_backproj, m_rc, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, m_param.max_itrs, 1));
		rectangle(img, m_rc, Scalar(0, 0, 255), 3, CV_AA);
	}
	else if (m_param.method == CAMSHIFT)
	{
		if (m_rc.width>0 && m_rc.height>0)
		{
			RotatedRect trackBox = CamShift(m_backproj, m_rc, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, m_param.max_itrs, 1));
			ellipse(img, trackBox, Scalar(0, 0, 255), 3, CV_AA);
		}

		if (m_rc.width <= 1 || m_rc.height <= 1)
		{
			int cols = m_backproj.cols, rows = m_backproj.rows, r = (MIN(cols, rows) + 5) / 6;
			m_rc = Rect(m_rc.x - r, m_rc.y - r, m_rc.width + 2 * r, m_rc.height + 2 * r) & Rect(0, 0, cols, rows);
		}
	}

	rc = m_rc;

	return true;
}

void tracker_opencv::configure()
{
	char sel = -1;
	cout << "  1. camshift\n"
		<< "  2. meanshift\n";
	cout << "select tracking method[1-2]: ";
	cin >> sel;
	cout << endl;

	if (sel == '1')
		m_param.method = CAMSHIFT;
	else if (sel == '2')
		m_param.method = MEANSHIFT;

	cout << "  1. HSV\n"
		<< "  2. RGB\n"
		<< "  3. hue\n"
		<< "  4. gray\n";
	cout << "select color model[1-4]: ";
	cin >> sel;
	cout << endl;

	if (sel == '1')
		m_param.color_model = CM_HSV;
	else if (sel == '2')
		m_param.color_model = CM_RGB;
	else if (sel == '3')
		m_param.color_model = CM_HUE;
	else if (sel == '4')
		m_param.color_model = CM_GRAY;
}

Mat tracker_opencv::get_bp_image()
{
	normalize(m_backproj, m_backproj, 0, 255, CV_MINMAX);
	return m_backproj;
}