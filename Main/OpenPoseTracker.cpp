#include "OpenPoseTracker.h"
#include <iostream>
#include <string>

OpenPoseTracker::OpenPoseConfiguration::OpenPoseConfiguration(int argc, char** argv)
{
	op::ConfigureLog::setPriorityThreshold((op::Priority)3);
	op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);

	outputSize = op::Point<int>(-1, -1);
	netInputSize = op::Point<int>(-1, 368);
	poseModel = op::PoseModel::COCO_18;
	
	op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
}

OpenPoseTracker::OpenPoseTracker(int argc, char** argv) :
	configuration_(argc, argv), scaleAndSizeExtractor_(configuration_.netInputSize, configuration_.outputSize, 1, 0.3),
	cvMatToOpInput_(configuration_.poseModel), cvMatToOpOutput_(), poseExtractorCaffe_(configuration_.poseModel, "../../../models/", 0)
{
	poseExtractorCaffe_.initializationOnThread();
}

std::vector<float> OpenPoseTracker::operator()(const std::string& image)
{
	// ------------------------- POSE ESTIMATION AND RENDERING -------------------------
	// Step 1 - Read and load image, error if empty (possibly wrong path)
	// Alternative: cv::imread(FLAGS_image_path, CV_LOAD_IMAGE_COLOR);
	cv::Mat inputImage = op::loadImage(image, CV_LOAD_IMAGE_COLOR);
	if (inputImage.empty())
		op::error("Could not open or find the image: " + image, __LINE__, __FUNCTION__, __FILE__);
	const op::Point<int> imageSize{ inputImage.cols, inputImage.rows };
	// Step 2 - Get desired scale sizes
	std::vector<double> scaleInputToNetInputs;
	std::vector<op::Point<int>> netInputSizes;
	double scaleInputToOutput;
	op::Point<int> outputResolution;
	std::tie(scaleInputToNetInputs, netInputSizes, scaleInputToOutput, outputResolution)
		= scaleAndSizeExtractor_.extract(imageSize);
	// Step 3 - Format input image to OpenPose input and output formats
	const auto netInputArray = cvMatToOpInput_.createArray(inputImage, scaleInputToNetInputs, netInputSizes);
	auto outputArray = cvMatToOpOutput_.createArray(inputImage, scaleInputToOutput, outputResolution);
	// Step 4 - Estimate poseKeypoints
	poseExtractorCaffe_.forwardPass(netInputArray, imageSize, scaleInputToNetInputs);
	const auto poseKeypoints = poseExtractorCaffe_.getPoseKeypoints();

	std::vector<float> tracked_joints;
	// 18 joints in COCO dataset
	for (uint i = 0; i < 18; i++)
	{
		tracked_joints.push_back(poseKeypoints[3 * i]);
		tracked_joints.push_back(poseKeypoints[3 * i + 1]);
	}

	return tracked_joints;
}