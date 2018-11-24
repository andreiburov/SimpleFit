#include "Demo.h"

#include <openpose/headers.hpp>

int Video(int argc, char** argv)
{
	cv::Mat frame;
	cv::VideoCapture cap{ 0 };

	cv::namedWindow("Video");

	while (cap.isOpened())
	{
		cap >> frame;

		cv::imshow("Video", frame);
		cv::waitKey(0);
	}
	
	cap.release();
	cv::destroyAllWindows();

	return 0;
}