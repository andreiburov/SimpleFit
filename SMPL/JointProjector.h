#pragma once

#include "Utils.h"
#include "Definitions.h"
#include "JointRegressor.h"

namespace smpl
{
	class JointProjector
	{
	public:

		struct Configuration
		{
			Configuration(const std::string& configuration_path)
			{
				ReadSparseMatrixFile(configuration_path + std::string("/coco_regressor.txt"), joint_regressor);
			}

			SparseMatrix joint_regressor;
			// taken from calibration (Utilities project)
			float intrinsics[9] = { 
				8.2860288147440258e+02f, 0.f, 3.1508821421435442e+02f,
				0.f, 8.0065721532070404e+02f, 2.4244337648523023e+02f,
				0.f, 0.f, 1.f };
		};

		JointProjector(const D3D& d3d, Configuration configuration) :
			d3d_(d3d), joint_regressor_(d3d, configuration.joint_regressor),
			intrinsics_(Eigen::Map<Eigen::Matrix3f, Eigen::RowMajor>(configuration.intrinsics, 3, 3).transpose())
		{
		}

		std::vector<float> operator()(Body body, float3 scaling, float3 translation) const
		{
			std::vector<float> joints2d;
			std::vector<float3>& vertices = body.vertices;

			#pragma omp parallel for
			for (uint i = 0; i < VERTEX_COUNT; i++)
			{
				vertices[i] = float3(Eigen::Scaling(scaling.ToEigen()) * vertices[i].ToEigen()  + translation.ToEigen());
			}

			Joints joints = joint_regressor_(vertices);
			Joints projected_joints = intrinsics_ * joints;

			for (uint i = 0; i < COCO_JOINT_COUNT; i++)
			{
				Eigen::Vector3f v = projected_joints.col(i);
				// homogeneous division
				joints2d.push_back(v(0) / v(2));
				joints2d.push_back(v(1) / v(2));
			}
			
			return joints2d;
		}

	private:
		const D3D& d3d_;
		const JointRegressor joint_regressor_;
		const Eigen::Matrix3f intrinsics_;

	};
}