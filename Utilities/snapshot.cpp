#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>

using namespace cv;
using namespace std;

int _____main(int argc, char** argv)
{
	namedWindow("Video");
	namedWindow("Picture");

	bool is_picture_ready = false;
	int count = 0;
	Mat frame, picture;
	VideoCapture cap(0);

	while (cap.isOpened())
	{
		cap >> frame;
		imshow("Video", frame);

		char key = char(waitKey(1));

		if (key == (char)27)
		{
			cap.release();
		}

		if (key == 'c')
		{
			frame.copyTo(picture);
			is_picture_ready = true;
		}

		if (is_picture_ready)
		{
			imshow("Picture", picture);

			if (key == 'p')
			{
				imwrite(std::string("test").append(to_string(count++)).append(".png").c_str(), picture);
			}
		}
	}

	return 0;
}