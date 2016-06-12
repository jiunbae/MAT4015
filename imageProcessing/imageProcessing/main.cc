#include <iostream>
#include <Windows.h>
#include "tracker.h"

using namespace std;

static string WINDOW_NAME = "TEST";

void process(VideoCapture *);
string get_filename();
void onMouse(int, int, int, int, void*);

struct MouseParam
{
	Mat frame;
	Point pt1, pt2;
	Rect rect;
	bool drag;
	bool updated;
};

int main(int argc, char * argv[])
{
	cout << "MAT4015 - project image tracking" << endl;
	cout << "Developed by MaybeS(https://github.com/MaybeS), 2016" << endl;
	cout << "\tOpenCV Version: " << CV_VERSION << endl;
	
	string name = get_filename();
	if (name == "")
	{
		cout << "select file" << endl;
		return 0;
	}

	VideoCapture * vc = new VideoCapture(name);
	if (!vc->isOpened())
	{
		cout << "can't open file" << endl;
		return 0;
	}

	if (vc)
		process(vc);
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
			}
			break;
	}
}

void process(VideoCapture * vc)
{
	tracker_opencv tracker;
	tracker.configure();

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
		if (param.updated)
		{
			Rect rc = param.rect;
			tracker.init(frame, rc);
			param.updated = false;
			tracking = true;
		}

		if (tracking)
		{
			*vc >> frame;
			if (frame.empty())
				break;
			Rect rc;
			bool ok = tracker.run(frame, rc);
		}

		imshow("image", frame);

		if (waitKey(10) == 27)
			break;
	}
}

string get_filename()
{
	cout << "select file for image tracking" << endl;
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
	cout << szFile << "selected" << endl;
	return szFile;
}