#include "Demo.h"

#include <openpose/core/headers.hpp>
#include <openpose/filestream/headers.hpp>
#include <openpose/gui/headers.hpp>
#include <openpose/pose/headers.hpp>
#include <openpose/utilities/headers.hpp>
#include <iostream>
#include <fstream>
#include <future>
#include <vector>
#include <algorithm>
#include <memory>

//#define FROM_VIDEO
#define TUNE_HYPERPARAMETERS
//#define WRITE_DOWN
#define RENDER_POSE
const int BODY_25_JOINT_COUNT = 25;
const bool kUseBody25 = true;
const int kImageWidth = 640;
const int kImageHeight = 480;

int kHLS[3] = { 255, 30, 40 }; // HLSType
int kClipRect[4] = { 35, 35, 30, 10 }; // left, top, right, bottom

enum HLSType
{
	hue, luminance, saturation
};

struct HLSUserData
{
	uchar border[3];
};

struct HLSUserDataWrapper
{
	HLSUserDataWrapper(const std::shared_ptr<HLSUserData>& user_data, const HLSType& type) :
		user_data(user_data), type(type) {}

	std::shared_ptr<HLSUserData> user_data;
	HLSType type;
};

void OnHLSChange(int current_value, void* ptr)
{
	HLSUserDataWrapper* wrapper = reinterpret_cast<HLSUserDataWrapper*>(ptr);
	std::shared_ptr<HLSUserData>& user_data = wrapper->user_data;

	switch (wrapper->type)
	{
	case hue:
		break;
	case luminance:
		user_data->border[1] = static_cast<uchar>(current_value);
		break;
	case saturation:
		user_data->border[2] = static_cast<uchar>(current_value);
		break;
	}
}

struct PauseUserData
{
	bool pause;
	cv::Rect button;
};

struct PauseUserDataWrapper
{
	PauseUserDataWrapper(const std::shared_ptr<PauseUserData>& user_data) :
		user_data(user_data) {}

	std::shared_ptr<PauseUserData> user_data;
};

void OnMouseClick(int event, int x, int y, int flags, void* ptr)
{
	PauseUserDataWrapper* wrapper = reinterpret_cast<PauseUserDataWrapper*>(ptr);
	std::shared_ptr<PauseUserData>& user_data = wrapper->user_data;

	if (event == CV_EVENT_LBUTTONDOWN)
	{
		if (user_data->button.contains(cv::Point(x, y)))
		{
			user_data->pause = !user_data->pause;
		}
	}
}

enum ClipRectType
{
	left, right, top, bottom
};

struct ClipRectUserData
{
	int clip_rect[4];
	ClipRectType type;
};

struct ClipRectUserDataWrapper
{
	ClipRectUserDataWrapper(const std::shared_ptr<ClipRectUserData>& user_data, const ClipRectType& type) :
		user_data(user_data), type(type) {}

	std::shared_ptr<ClipRectUserData> user_data;
	ClipRectType type;
};

void RenderButton(cv::Mat& image, const std::shared_ptr<PauseUserData>& user_data)
{
	cv::rectangle(image, user_data->button, cv::Scalar(0, 255, 0), cv::FILLED);
	cv::putText(image, cv::String("Pause"),
		cv::Point(user_data->button.width*0.2 + user_data->button.x, 
			user_data->button.height*0.5 + user_data->button.y),
		cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
}

void OnClipRectChange(int current_value, void* ptr)
{
	ClipRectUserDataWrapper* wrapper = reinterpret_cast<ClipRectUserDataWrapper*>(ptr);
	std::shared_ptr<ClipRectUserData>& user_data = wrapper->user_data;

	switch (wrapper->type)
	{
	case left:
		user_data->clip_rect[0] = current_value;
		break;
	case top:
		user_data->clip_rect[1] = current_value;
		break;
	case right:
		user_data->clip_rect[2] = current_value;
		break;
	case bottom:
		user_data->clip_rect[3] = current_value;
		break;
	}
}

void FilterBasedOnHLS(const std::shared_ptr<HLSUserData>& user_data, 
	const cv::Mat& input_image, cv::Mat& output_image)
{
	cv::Mat hls;
	//cv::cvtColor(input_image, hls, CV_BGR2HSV);
	cv::cvtColor(input_image, hls, CV_BGR2HLS);

	for (int x = 0; x < kImageWidth; x++)
	{
		for (int y = 0; y < kImageHeight; y++)
		{
			cv::Vec3b intensity = hls.at<cv::Vec3b>(y, x);
			uchar hue = intensity.val[0];
			uchar lum = intensity.val[1];
			uchar sat = intensity.val[2];

			if (sat < user_data->border[1]) // low chroma
			{
				if (lum > user_data->border[2]) // but not dark
				{
					cv::Vec3b& pixel = output_image.at<cv::Vec3b>(y, x);
					pixel[0] = 255u; pixel[1] = 255u; pixel[2] = 255u;
				}
			}
		}
	}
}

void FilterBasedOnRectangle(const std::shared_ptr<ClipRectUserData>& user_data,
	const op::Array<float>& pose_keypoints, cv::Mat& output_image)
{
	int pose_x[BODY_25_JOINT_COUNT];
	int pose_y[BODY_25_JOINT_COUNT];
	int min_x = kImageWidth, max_x = 0, min_y = kImageHeight, max_y = 0;
	int thd = 35, thd_bottom = 10;

	size_t volume = pose_keypoints.getVolume();
	if (volume / 3 != BODY_25_JOINT_COUNT) return;

	for (int i = 0; i < BODY_25_JOINT_COUNT; i++)
	{
		pose_x[i] = static_cast<int>(pose_keypoints[i * 3]);
		pose_y[i] = static_cast<int>(pose_keypoints[i * 3 + 1]);

		if (min_x > pose_x[i] && pose_x[i] > 0) min_x = pose_x[i];
		if (min_y > pose_y[i] && pose_y[i] > 0) min_y = pose_y[i];
		if (max_x < pose_x[i]) max_x = pose_x[i];
		if (max_y < pose_y[i]) max_y = pose_y[i];
	}

	/*cv::Rect border(cv::Point(min_x - thd, min_y - thd),
		cv::Point(max_x + thd, max_y + thd_bottom));
	cv::Scalar color(0, 255, 0);
	int thickness = 3;
	cv::rectangle(output_image, border, color, thickness);*/

	for (int x = 0; x < kImageWidth; x++)
	{
		for (int y = 0; y < kImageHeight; y++)
		{
			cv::Vec3b& pixel = output_image.at<cv::Vec3b>(y, x);

			if (x > min_x - user_data->clip_rect[0] && x < max_x + user_data->clip_rect[2] &&
				y > min_y - user_data->clip_rect[1] && y < max_y + user_data->clip_rect[3])
			{
			}
			else {
				pixel[0] = 255u; pixel[1] = 255u; pixel[2] = 255u;
			}
		}
	}
}

void InvertSilhouette(cv::Mat& output_image)
{
	for (int x = 0; x < kImageWidth; x++)
	{
		for (int y = 0; y < kImageHeight; y++)
		{
			cv::Vec3b& pixel = output_image.at<cv::Vec3b>(y, x);
			if (pixel[0] == 255u && pixel[1] == 255u && pixel[2] == 255u)
			{
				pixel[0] = 0u; pixel[1] = 0u; pixel[2] = 0u;
			}
			else
			{
				pixel[0] = 255u; pixel[1] = 255u; pixel[2] = 255u;
			}
		}
	}
}

int ExtractSilhouette(int argc, char** argv)
{
	// use -1x-1 to force the program to use the input image resolution
	const auto kOutputSize = op::Point<int>(-1, -1);
	// using -1 in any of the dimensions, OP will choose the optimal aspect ratio
	// depending on the user's input value
	const auto kNetInputSize = op::Point<int>(-1, 368);
	const auto kPoseModel = (kUseBody25) ? op::PoseModel::BODY_25 : op::PoseModel::COCO_18;
	const std::string kModelFolder("../Resources/models/");
	// GPU device start number
	const int kGpuStartIndex = 0;
	// blending factor (range 0-1) for the body part rendering
	const double kAlphaPose = 0.6;
	// blending factor (range 0-1) between heatmap and original frame. 1 will only show the heatmap
	const double kAlphaHeatmap = 0.7;
	// no effect unless kScaleNumber > 1
	const double kScaleGap = 0.3;
	// number of scales to average
	const int kScaleNumber = 1;
	// only estimated keypoints whose score confidences are higher than this threshold will be rendered.
	// generally, a high threshold (> 0.5) will only render very clear body parts.
	const double kRenderThreshold = 0.05;
	// if enabled, it will render the results (keypoint skeletons or heatmaps) on a black.
	const bool kDisableBlending = false;
	// prediction channel to visualize (default: 0). 0 for all the body parts, 1-18 for each body
	// part heat map, 19 for the background heat map, 20 for all the body part heat maps
	// together, 21 for all the PAFs, 22-40 for each body part pair PAF.
	const int kPartToShow = 0;

	// CURRENT EXAMPLE
	op::ConfigureLog::setPriorityThreshold((op::Priority)3);
	const op::ScaleAndSizeExtractor scale_and_size_extractor{ kNetInputSize, kOutputSize, kScaleNumber, kScaleGap };
	auto pose_extractor_ptr = std::make_shared<op::PoseExtractorCaffe>(kPoseModel, kModelFolder, kGpuStartIndex);
	op::PoseGpuRenderer pose_gpu_renderer
		{ kPoseModel, pose_extractor_ptr, (float)kRenderThreshold, !kDisableBlending, 
		(float)kAlphaPose, (float)kAlphaHeatmap };
	pose_gpu_renderer.setElementToRender(kPartToShow);
	pose_extractor_ptr->initializationOnThread();
	pose_gpu_renderer.initializationOnThread();

#ifdef RENDER_POSE
	op::OpOutputToCvMat op_output_to_cv_mat;
#endif
	op::CvMatToOpInput cv_mat_to_op_input{ kPoseModel };
	op::CvMatToOpOutput cv_mat_to_op_output;
	cv::Mat input_image, output_image;

	const std::string kInputFolder{ "../Resources/Easy/" };
	size_t count = 0;
	const size_t kMaxCount = 1842;
#ifdef WRITE_DOWN
	std::vector<std::future<void>> futures;
#endif
	
	// SCALES
	const op::Point<int> kImageSize{ kImageWidth, kImageHeight };
	std::vector<double> scale_input_to_net_inputs;
	std::vector<op::Point<int>> net_input_sizes;
	double scale_input_to_output;
	op::Point<int> output_resolution;
	std::tie(scale_input_to_net_inputs, net_input_sizes, scale_input_to_output, output_resolution)
		= scale_and_size_extractor.extract(kImageSize);

	// OTHER
	std::shared_ptr<HLSUserData> hls_user_data = std::make_shared<HLSUserData>();
	hls_user_data->border[1] = static_cast<uchar>(kHLS[1]);
	hls_user_data->border[2] = static_cast<uchar>(kHLS[2]);
	std::vector<std::unique_ptr<HLSUserDataWrapper>> hls_wrappers;
	hls_wrappers.emplace_back(std::make_unique<HLSUserDataWrapper>(hls_user_data, luminance));
	hls_wrappers.emplace_back(std::make_unique<HLSUserDataWrapper>(hls_user_data, saturation));

	std::shared_ptr<PauseUserData> pause_user_data = std::make_shared<PauseUserData>();
	pause_user_data->button = cv::Rect(0, 0, 100, 50);
	pause_user_data->pause = false;
	std::unique_ptr<PauseUserDataWrapper> pause_wrapper = std::make_unique<PauseUserDataWrapper>(pause_user_data);

	std::shared_ptr<ClipRectUserData> clip_rect_user_data = std::make_shared<ClipRectUserData>();
	memcpy(clip_rect_user_data->clip_rect, kClipRect, sizeof(clip_rect_user_data->clip_rect));
	std::vector<std::unique_ptr<ClipRectUserDataWrapper>> clip_rect_wrappers;
	clip_rect_wrappers.emplace_back(std::make_unique<ClipRectUserDataWrapper>(clip_rect_user_data, left));
	clip_rect_wrappers.emplace_back(std::make_unique<ClipRectUserDataWrapper>(clip_rect_user_data, top));
	clip_rect_wrappers.emplace_back(std::make_unique<ClipRectUserDataWrapper>(clip_rect_user_data, right));
	clip_rect_wrappers.emplace_back(std::make_unique<ClipRectUserDataWrapper>(clip_rect_user_data, bottom));

	cv::namedWindow("Extract Silhouette");
	cv::setMouseCallback("Extract Silhouette", OnMouseClick, pause_wrapper.get());
#ifdef RENDER_POSE
	cv::namedWindow("Extract Pose");
#endif
#ifdef TUNE_HYPERPARAMETERS
	cv::createTrackbar("Luminance", "Extract Silhouette", &kHLS[1], 255, OnHLSChange, hls_wrappers[0].get());
	cv::createTrackbar("Saturation", "Extract Silhouette", &kHLS[2], 255, OnHLSChange, hls_wrappers[1].get());
	cv::createTrackbar("Left", "Extract Silhouette", &kClipRect[0], kImageWidth, OnClipRectChange, clip_rect_wrappers[0].get());
	cv::createTrackbar("Top", "Extract Silhouette", &kClipRect[1], kImageHeight, OnClipRectChange, clip_rect_wrappers[1].get());
	cv::createTrackbar("Right", "Extract Silhouette", &kClipRect[2], kImageWidth, OnClipRectChange, clip_rect_wrappers[2].get());
	cv::createTrackbar("Bottom", "Extract Silhouette", &kClipRect[3], kImageHeight, OnClipRectChange, clip_rect_wrappers[3].get());
#endif

	// POSE EXTRACTION AND RENDERING
#ifdef FROM_VIDEO
	cv::VideoCapture cap(0);
#endif
	while (true)
	{
		int64 start = cv::getTickCount();

#ifdef FROM_VIDEO
		if (!pause_user_data->pause) {
			cap >> input_image;
		}
#else
		if (!pause_user_data->pause) {
			input_image = op::loadImage(kInputFolder + std::to_string(count) + ".png", CV_LOAD_IMAGE_COLOR);
		}
#endif

		const auto net_input_array = cv_mat_to_op_input.createArray(input_image, scale_input_to_net_inputs, net_input_sizes);
		auto output_array = cv_mat_to_op_output.createArray(input_image, scale_input_to_output, output_resolution);
		pose_extractor_ptr->forwardPass(net_input_array, kImageSize, scale_input_to_net_inputs);
		const auto pose_keypoints = pose_extractor_ptr->getPoseKeypoints();
		
#ifdef RENDER_POSE
		const auto scale_net_to_output = pose_extractor_ptr->getScaleNetToOutput();
		pose_gpu_renderer.renderPose(output_array, pose_keypoints, scale_input_to_output, scale_net_to_output);
		auto pose_image = op_output_to_cv_mat.formatToCvMat(output_array);
		cv::imshow("Extract Pose", pose_image);
#endif
		input_image.copyTo(output_image);

		FilterBasedOnRectangle(clip_rect_user_data, pose_keypoints, output_image);
		FilterBasedOnHLS(hls_user_data, input_image, output_image);
		InvertSilhouette(output_image);

#ifdef WRITE_DOWN
		cv::Mat lambda_image;
		output_image.copyTo(lambda_image);
		futures.push_back(std::async([=]()
		{
			cv::imwrite(std::string("../Resources/EasySilhouettes/").append(std::to_string(count)).append(".png").c_str(), lambda_image);
			std::ofstream file(std::string("../Resources/EasyBody25/").append(std::to_string(count)).append(".txt"));
			std::vector<int> pose_keypoints_size = pose_keypoints.getSize();
			size_t volume = pose_keypoints.getVolume();
			if (volume / 3 != BODY_25_JOINT_COUNT) return;

			for (int i = 0; i < BODY_25_JOINT_COUNT; i++)
			{
				file << pose_keypoints[3*i] << " ";
				file << pose_keypoints[3*i + 1] << " ";
			}
		}));
#endif

		RenderButton(output_image, pause_user_data);
	
		cv::imshow("Extract Silhouette", output_image);
		cv::waitKey(1);

		if (!pause_user_data->pause) count++;
		if (count > kMaxCount) break;
		double fps = cv::getTickFrequency() / (cv::getTickCount() - start);
		std::cout << "FPS : " << fps << std::endl;
	}

#ifdef WRITE_DOWN
	for (auto& f : futures)
	{
		f.get();
	}
#endif

	return 0;
}