#include "Definitions.h"

namespace smpl
{
	const char* JOINT_FROM_INDEX[JOINT_COUNT] = {
		"HIP_CENTER",
		"HIP_RIGHT",
		"HIP_LEFT",
		"STOMACH",
		"KNEE_RIGHT",
		"KNEE_LEFT",
		"BACKBONE",
		"ANKLE_RIGHT",
		"ANKLE_LEFT",
		"CHEST",
		"FOOT_RIGHT",
		"FOOT_LEFT",
		"SHOULDER_CENTER",
		"PECK_RIGHT",
		"PECK_LEFT",
		"CHIN",
		"SHOULDER_RIGHT",
		"SHOULDER_LEFT",
		"ELBOW_RIGHT",
		"ELBOW_LEFT",
		"WRIST_RIGHT",
		"WRIST_LEFT",
		"HAND_RIGHT",
		"HAND_LEFT",
	};

	std::map<std::string, int> INDEX_FROM_JOINT = {
		{"HIP_CENTER", 0},
		{"HIP_RIGHT", 1},
		{"HIP_LEFT", 2},
		{"STOMACH", 3},
		{"KNEE_RIGHT", 4},
		{"KNEE_LEFT", 5},
		{"BACKBONE", 6},
		{"ANKLE_RIGHT", 7},
		{"ANKLE_LEFT", 8},
		{"CHEST", 9},
		{"FOOT_RIGHT", 10},
		{"FOOT_LEFT", 11},
		{"SHOULDER_CENTER", 12},
		{"PECK_RIGHT", 13},
		{"PECK_LEFT", 14},
		{"CHIN", 15},
		{"SHOULDER_RIGHT", 16},
		{"SHOULDER_LEFT", 17},
		{"ELBOW_RIGHT", 18},
		{"ELBOW_LEFT", 19},
		{"WRIST_RIGHT", 20},
		{"WRIST_LEFT", 21},
		{"HAND_RIGHT", 22},
		{"HAND_LEFT", 23}
	};

	int GetIndexFromJoint(std::string joint)
	{
		try {
			return INDEX_FROM_JOINT.at(joint);
		}
		catch (...)
		{
			std::cerr << "Joint name not recognized: " << joint << std::endl;
		}

		return -1;
	}

	std::vector<float3> Joints2Vector(const RegressedJoints& joints)
	{
		std::vector<float3> r;

		for (uint i = 0; i < joints.cols(); i++)
		{
			auto j = joints.col(i);
			r.push_back(float3(j));
		}

		return r;
	}
}