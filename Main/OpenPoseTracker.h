#pragma once

// OpenPose dependencies
#include <openpose/core/headers.hpp>
#include <openpose/filestream/headers.hpp>
#include <openpose/gui/headers.hpp>
#include <openpose/pose/headers.hpp>
#include <openpose/utilities/headers.hpp>

class OpenPoseTracker
{
public:

	struct OpenPoseConfiguration
	{
		OpenPoseConfiguration(int argc, char** argv);

		op::Point<int> outputSize;
		op::Point<int> netInputSize;
		op::PoseModel poseModel;
	};

	OpenPoseTracker(int argc, char** argv);
		
	std::vector<float> operator()(const std::string& image);

private:
	OpenPoseConfiguration configuration_;
	op::ScaleAndSizeExtractor  scaleAndSizeExtractor_;
	op::CvMatToOpInput cvMatToOpInput_;
	op::CvMatToOpOutput cvMatToOpOutput_;
	op::PoseExtractorCaffe poseExtractorCaffe_;
};