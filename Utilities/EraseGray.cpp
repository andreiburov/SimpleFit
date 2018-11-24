#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <future>

#include <Windows.h>

#include "Utilities.h"

using namespace cv;
using namespace std;

const double FPS = 10;
const int MAX_VIDEOS_COUNT = 10;

int EraseGray(int argc, char** argv)
{
	namedWindow("Input");
	namedWindow("Output");
	Mat input, image_hsv, output;

	input = imread("Video/Frames1/338.png", IMREAD_UNCHANGED);
	imshow("Input", input);

	input.copyTo(output);

	cvtColor(input, image_hsv, CV_BGR2HSV);

	for (int x = 0; x < input.rows; x++)
	{
		for (int y = 0; y < input.cols; y++)
		{
			Vec3b intensity = image_hsv.at<Vec3b>(x, y);
			uchar hue = intensity.val[0]; // 0..255
			uchar sat = intensity.val[1]; // 0..180
			uchar lum = intensity.val[2]; // 0..255

			if (sat < 40u) // low chroma
			{
				if (lum > 55u) // but not dark
				{
					Vec3b& pixel = output.at<Vec3b>(x, y);
					pixel[0] = 255u;
					pixel[1] = 255u;
					pixel[2] = 255u;
				}
			}
		}
	}

	imshow("Output", output);
	waitKey(0);

	return 0;
}