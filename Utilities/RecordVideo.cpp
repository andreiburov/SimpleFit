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

int RecordVideo(int argc, char** argv)
{
	namedWindow("Video");

	bool is_recording = false;
	unsigned video_count = 0, frame_count = 0;

	std::vector<std::future<void> > futures;

	VideoCapture cap(0);

	if (!cap.isOpened())
	{
		MessageBoxA(NULL, "Can not open video captue!", "Error", MB_OK);
		return -1;
	}

	cv::Size2i size(cap.get(CV_CAP_PROP_FRAME_WIDTH), cap.get(CV_CAP_PROP_FRAME_HEIGHT));
	cv::Rect border(cv::Point(0, 0), size);
	cv::Scalar color(0, 255, 0);
	int thickness = 3;

	VideoWriter* video_writer = nullptr;

	while (cap.isOpened())
	{
		Mat frame, frame_copy;
		cap >> frame;
		
		char key = char(waitKey(1));

		if (key == (char)27)
		{
			cap.release();
			break;
		}

		if (is_recording)
		{
			if (video_writer != nullptr) video_writer->write(frame);

			frame_count++;
			frame.copyTo(frame_copy);
			futures.push_back(std::async([=]()
			{
				imwrite(std::string("Video/Frames").append(to_string(video_count)).append("/")
					.append(std::to_string(frame_count)).append(".png").c_str(), frame_copy);
			}));

			cv::rectangle(frame, border, color, thickness);

			if (key == 'p')
			{
				is_recording = false;
				video_count++;
				frame_count = 0;

				if (video_count >= MAX_VIDEOS_COUNT)
				{
					break;
				}

				futures.push_back(std::async([=]()
				{
					video_writer->release();
					delete video_writer;
				}));

				video_writer = nullptr;
			}
		}

		if (key == 'c')
		{
			if (video_count < MAX_VIDEOS_COUNT && !is_recording)
			{
				video_writer = new VideoWriter(std::string("Video/video").append(std::to_string(video_count))
					.append(".avi"), CV_FOURCC('M', 'J', 'P', 'G'), FPS, size, true);
			}
			is_recording = true;
		}

		putText(frame, "Press 'c' to start recording", cvPoint(30, 30),
			FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 10), 1, CV_AA);
		putText(frame, "Press 'p' to stop recording", cvPoint(30, 45),
			FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 10), 1, CV_AA);
		imshow("Video", frame);
	}

	for (auto& f : futures)
	{
		f.get();
	}

	return 0;
}