/////////////////////////////////////////////////
// MaybeS: https://github.com/MaybeS/MAT4015.git
// mean shift - object tracking.
// + dynamic search range
// + tracking object autosizing
// + model histogram update
//
// default@opencv
//@import 
//- .\include
//- .\lib\x86\vc12\bin
//@debug
//- opencv_world300d.lib
//- opencv_ts300d.lib
//@lib
//- .\lib\x86\vc12\lib


#include <iostream>
#include <Windows.h>
#include "cv_tracker.h"

using namespace std;

#define PROGRAM_NAME "MAT4015"
#define WINDOW_NAME "tracking"

string get_filename();
void onMouse(int, int, int, int, void*);
void process(VideoCapture *, COLOR_MODEL);

int main(int argc, char * argv[])
{
	// print information
	cout << "MAT4015 - project image tracking" << endl;
	cout << "Developed by MaybeS(https://github.com/MaybeS), 2016" << endl;
	cout << "\tOpenCV Version: " << CV_VERSION << endl << endl;
	
	// get filename (select file dialog)
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

	// set Color Model (select color model)
	cout << "Select color model" << endl;
	int model;
	cout << "\t1. HSV" << endl << "\t2. RGB" << endl << "\t3. hue" << endl << "\t4. gray" << endl << "selected model: ";
	cin >> model;

	// process object tracking
	if (vc)
		process(vc, static_cast<COLOR_MODEL>(model - 1));
	//if (vc)
	//	delete vc;

	waitKey(0);
	destroyAllWindows();
	return 0;
}
struct MouseParam {
	Mat frame;
	Point sPoint, ePoint;
	Rect rect;
	bool drag, updated;
};

// Mouse Event callback function
// using for select tracking object
void onMouse(int event, int x, int y, int flags, void* param)
{
	MouseParam * p = (MouseParam *)param;
	switch (event)
	{
		case CV_EVENT_LBUTTONDOWN:
			p->ePoint = p->sPoint = { x, y };
			p->drag = true;
			break;

		case CV_EVENT_LBUTTONUP:
			{
				p->rect = { p->sPoint.x, p->sPoint.y, x - p->sPoint.x, y - p->sPoint.y };
				p->drag = false;
				
				if (p->rect.area() > 100)
					p->updated = true;
				break;
			}

		case CV_EVENT_MOUSEMOVE:
			if (p->drag && (p->ePoint.x - x || p->ePoint.y - y) && !p->updated)
			{
				Mat img = p->frame.clone();
				p->ePoint = { x, y };
				rectangle(img, p->sPoint, p->ePoint, Scalar(0, 0, 255), 2);
				imshow(WINDOW_NAME, img);
				break;
			}
	}
}

// object tracking process with Tracker - cv_tracker
void process(VideoCapture * vc, COLOR_MODEL model)
{
	Tracker tracker;

	Mat frame;
	*vc >> frame;

	// open window for image capture
	imshow(WINDOW_NAME, frame);

	MouseParam param;
	param.frame = frame;
	param.drag = false;
	param.updated = false;
	setMouseCallback(WINDOW_NAME, onMouse, &param);

	bool tracking = false;
	while (1)
	{
		// tracking
		if (tracking)
		{
			// next frame
			*vc >> frame;
			if (frame.empty())
				break;
			
			tracker.run(frame);
		}

		// if tracking object selected
		if (param.updated)
		{
			// initilize with param.rect(selected object)
			tracker.initilize(frame, param.rect, model);
			param.updated = false;
			tracking = true;

			// close window
			cvDestroyWindow(WINDOW_NAME);
		}

		if (waitKey(10) == 27)
			break;
	}
}

// get filename dialog
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