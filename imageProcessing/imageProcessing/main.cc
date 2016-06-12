#include <iostream>
#include <Windows.h>
#include "cv_tracker.h"

using namespace std;

static string NAME = "MAT4015";

void process(VideoCapture *, COLOR_MODEL);
string get_filename();
void onMouse(int, int, int, int, void*);

struct MouseParam
{
	Mat frame;
	Point pt1, pt2;
	Rect rect;
	bool drag, updated;
};

int main(int argc, char * argv[])
{
	// print information
	cout << "MAT4015 - project image tracking" << endl;
	cout << "Developed by MaybeS(https://github.com/MaybeS), 2016" << endl;
	cout << "\tOpenCV Version: " << CV_VERSION << endl << endl;
	
	// get filename
	cout << "Select file for image tracking" << endl;
	string name = get_filename();
	if (name == "")
	{
		cout << "\t@select file for image tarcking!" << endl;
		return 0;
	}
	cout << "\t" << name << " selected" << endl << endl;

	// new VideoCapture
	VideoCapture * vc = new VideoCapture(name);
	if (!vc->isOpened())
	{
		cout << "\t@select openable video file!" << endl;
		return 0;
	}

	// set Color Model
	cout << "Select color model" << endl;
	int model;
	cout << "\t1. HSV" << endl << "\t2. RGB" << endl << "\t3. hue" << endl << "\t4. gray" << endl << "selected model: ";
	cin >> model;

	if (vc)
		process(vc, static_cast<COLOR_MODEL>(model - 1));
	//if (vc)
	//	delete vc;

	waitKey(0);
	destroyAllWindows();
	return 0;
}
void onMouse(int event, int x, int y, int flags, void* param)
{
	MouseParam * p = (MouseParam *)param;
	switch (event)
	{
		case CV_EVENT_LBUTTONDOWN:
			p->pt1.x = x;
			p->pt1.y = y;
			p->pt2 = p->pt1;
			p->drag = true;
			break;

		case CV_EVENT_LBUTTONUP:
			{
				int w = x - p->pt1.x;
				int h = y - p->pt1.y;

				p->rect.x = p->pt1.x;
				p->rect.y = p->pt1.y;
				p->rect.width = w;
				p->rect.height = h;
				p->drag = false;

				if (w >= 10 && h >= 10)
					p->updated = true;
				break;
			}

		case CV_EVENT_MOUSEMOVE:
			if (p->drag && (p->pt2.x != x || p->pt2.y != y) && !p->updated)
			{
				Mat img = p->frame.clone();
				p->pt2.x = x;
				p->pt2.y = y;
				rectangle(img, p->pt1, p->pt2, Scalar(0, 255, 0), 1);
				imshow("image", img);
				break;
			}
	}
}

void process(VideoCapture * vc, COLOR_MODEL model)
{
	cvTracker tracker;

	Mat frame;
	*vc >> frame;
	imshow("image", frame);

	MouseParam param;
	param.frame = frame;
	param.drag = false;
	param.updated = false;
	setMouseCallback("image", onMouse, &param);

	bool tracking = false;
	while (1)
	{
		if (tracking)
		{
			*vc >> frame;
			if (frame.empty())
				break;;
			Rect nRect;
			tracker.run(frame, nRect);
		}

		if (param.updated)
		{
			tracker.initilize(frame, param.rect, model);
			param.updated = false;
			tracking = true;
		}

		imshow("image", frame);

		if (waitKey(10) == 27)
			break;
	}
}

string get_filename()
{
	OPENFILENAME file;
	char szFile[MAX_PATH] = "";
	ZeroMemory(&file, sizeof(OPENFILENAME));
	file.lStructSize = sizeof(OPENFILENAME);
	file.hwndOwner = NULL;
	file.lpstrFile = szFile;
	file.nMaxFile = sizeof(szFile);
	file.lpstrFilter = "Avi Files(*.avi)\0*.avi\0All Files (*.*)\0*.*\0";
	file.nFilterIndex = 1;
	file.lpstrFileTitle = NULL;
	file.nMaxFileTitle = 0;
	file.lpstrInitialDir = NULL;
	file.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;
	if (::GetOpenFileName(&file) == false)
		return "";
	return szFile;
}