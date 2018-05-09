#pragma once

#include "Utils.h"
#include "JointRegressor.h"

namespace smpl
{
	struct OptimizerConfiguration
	{
		OptimizerConfiguration(const std::string& configuration_path)
		{
			ReadSparseMatrixFile(configuration_path  + std::string("/coco_regressor.txt"), joint_regressor);
		}

		SparseMatrix joint_regressor;
	};

	class Optimizer
	{
	public:
		Optimizer(const D3D& d3d, OptimizerConfiguration configuration, const std::vector<float2>& tracked_joints) :
			tracked_joints_(tracked_joints), joint_regressor_(d3d, configuration.joint_regressor)
		{
		}

		void operator()(const Body& body, ShapeCoefficients& betas, PoseCoefficients& thetas)
		{
			Joints joints = joint_regressor_(body.vertices);

			std::ofstream file("coco_joints.obj");
			for (uint i = 0; i < COCO_JOINT_COUNT; i++)
			{
				file << "v " << joints.col(i)(0) << " " << joints.col(i)(1) << " " << joints.col(i)(2) << "\n";
			}
		}

	private:
		const std::vector<float2> tracked_joints_;
		const JointRegressor joint_regressor_;
	};
}