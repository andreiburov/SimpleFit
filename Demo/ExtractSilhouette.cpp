#include "Demo.h"
#include <openpose/core/headers.hpp>
#include <openpose/filestream/headers.hpp>
#include <openpose/gui/headers.hpp>
#include <openpose/pose/headers.hpp>
#include <openpose/utilities/headers.hpp>

#define FROM_VIDEO

int ExtractSilhouette(int argc, char** argv)
{
	op::ConfigureLog::setPriorityThreshold((op::Priority)3);

	// OPENPOSE CONFIG
	// use -1x-1 to force the program to use the input image resolution
	const auto outputSize = op::Point<int>(-1, -1);
	// using -1 in any of the dimensions, OP will choose the optimal aspect ratio
	// depending on the user's input value
	const auto netInputSize = op::Point<int>(-1, 368);
	const auto poseModel = op::PoseModel::BODY_25;
	const std::string model_folder("../Resources/models/");
	// GPU device start number
	const int num_gpu_start = 0;
	// blending factor (range 0-1) for the body part rendering
	const double alpha_pose = 0.6;
	// blending factor (range 0-1) between heatmap and original frame. 1 will only show the heatmap
	const double alpha_heatmap = 0.7;
	// no effect unless scale_number > 1
	const double scale_gap = 0.3;
	// number of scales to average
	const int scale_number = 1;
	// prediction channel to visualize (default: 0). 0 for all the body parts, 1-18 for each body
	// part heat map, 19 for the background heat map, 20 for all the body part heat maps
	// together, 21 for all the PAFs, 22-40 for each body part pair PAF.
	int part_to_show = 0;
	// only estimated keypoints whose score confidences are higher than this threshold will be rendered.
	// generally, a high threshold (> 0.5) will only render very clear body parts.
	double render_threshold = 0.05;
	// if enabled, it will render the results (keypoint skeletons or heatmaps) on a black.
	bool disable_blending = false;

	// CURRENT EXAMPLE
	const std::string image_path("../Resources/Frames1/338.png");

	op::ScaleAndSizeExtractor scaleAndSizeExtractor(netInputSize, outputSize, scale_number, scale_gap);
	op::CvMatToOpInput cvMatToOpInput{poseModel};
	op::CvMatToOpOutput cvMatToOpOutput;
	auto poseExtractorPtr = std::make_shared<op::PoseExtractorCaffe>
		(poseModel, model_folder, num_gpu_start);

	op::PoseGpuRenderer poseGpuRenderer
		{poseModel, poseExtractorPtr, (float)render_threshold, !disable_blending, 
		(float)alpha_pose, (float)alpha_heatmap};

	poseGpuRenderer.setElementToRender(part_to_show);
	op::OpOutputToCvMat opOutputToCvMat;
	//op::FrameDisplayer frameDisplayer{"Extract Silhouette", outputSize};
	cv::namedWindow("Extract Silhouette");

	// single thread initialization
	poseExtractorPtr->initializationOnThread();
	poseGpuRenderer.initializationOnThread();

	cv::Mat inputImage;
	inputImage = op::loadImage(image_path, CV_LOAD_IMAGE_COLOR);

	const op::Point<int> imageSize{ inputImage.cols, inputImage.rows };
	// get desired scale sizes
	std::vector<double> scaleInputToNetInputs;
	std::vector<op::Point<int>> netInputSizes;
	double scaleInputToOutput;
	op::Point<int> outputResolution;
	std::tie(scaleInputToNetInputs, netInputSizes, scaleInputToOutput, outputResolution)
		= scaleAndSizeExtractor.extract(imageSize);
	// format input image to OpenPose input and output formats
	const auto netInputArray = cvMatToOpInput.createArray(inputImage, scaleInputToNetInputs, netInputSizes);
	auto outputArray = cvMatToOpOutput.createArray(inputImage, scaleInputToOutput, outputResolution);

	// POSE EXTRACTION AND RENDERING
#ifdef FROM_VIDEO
	cv::VideoCapture cap(0);
#endif
	while (cap.isOpened())
	{
#ifdef FROM_VIDEO
		cap >> inputImage;
#endif
#ifndef FROM_VIDEO
		inputImage = op::loadImage(image_path, CV_LOAD_IMAGE_COLOR);
#endif

		// estimate poseKeypoints
		poseExtractorPtr->forwardPass(netInputArray, imageSize, scaleInputToNetInputs);
		const auto poseKeypoints = poseExtractorPtr->getPoseKeypoints();
		const auto scaleNetToOutput = poseExtractorPtr->getScaleNetToOutput();
		// render pose
		poseGpuRenderer.renderPose(outputArray, poseKeypoints, scaleInputToOutput, scaleNetToOutput);
		// OpenPose output format to cv::Mat
		auto outputImage = opOutputToCvMat.formatToCvMat(outputArray);

		//frameDisplayer.displayFrame(outputImage, 0); // Alternative: cv::imshow(outputImage) + cv::waitKey(0)
		cv::imshow("Extract Silhouette", outputImage);
		cv::waitKey(0);
	}

	return 0;
}