#pragma once

#ifndef __CV__WINDOW__
#define __CV__WINDOW__

#include <functional>

#include "opencv2\core\core.hpp"
#include "opencv2\highgui\highgui.hpp"
#include "opencv2\imgproc\imgproc.hpp"
#include "opencv2\video\video.hpp"

using namespace std;
using namespace cv;

namespace cv {
	struct MouseParam 
	{
		Mat frame;
		Point sPoint, ePoint;
		Rect rect;
		bool drag, updated;
	};

	class Window {
	public:
		Window(String name);
		~Window();

		void setImage(Mat);
		void setMouseCallback(MouseCallback callback);
	protected:

		String name;
		MouseParam param;
	};
}

#endif