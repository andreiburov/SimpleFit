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

	std::vector<float3> Joints2Vector(const Joints& joints)
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