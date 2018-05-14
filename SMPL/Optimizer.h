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
		// taken from calibration (Utilities project)
		float intrinsics[9] = { 8.2860288147440258e+02f, 0.f, 3.1508821421435442e+02f, 0.f,
			8.0065721532070404e+02f, 2.4244337648523023e+02f, 0.f, 0.f, 1.f };
	};

	class Projector
	{
	public:
		Projector(const D3D& d3d, float* intrinsics) :
			intrinsics_(Eigen::Map<Eigen::Matrix3f, Eigen::RowMajor>(intrinsics, 3, 3).transpose())
		{
		}

		std::vector<float2> operator()(const Joints& joints) const
		{
			// project with intrinsics
			Joints projected_joints = intrinsics_ * joints;

			std::vector<float2> joints2d;
			
			for (uint i = 0; i < COCO_JOINT_COUNT; i++)
			{
				Eigen::Vector3f v = projected_joints.col(i);
				// homogeneous division
				v /= v(2);
				joints2d.push_back(float2(v(0),v(1)));
			}

			return joints2d;
		}

	private:
		Eigen::Matrix3f intrinsics_;
	};

	class Optimizer
	{
	public:
		Optimizer(const D3D& d3d, OptimizerConfiguration configuration, const std::vector<float>& tracked_joints) :
			joint_regressor_(d3d, configuration.joint_regressor),
			projector_(d3d, configuration.intrinsics)
		{
			for (uint i = 0; i < COCO_JOINT_COUNT; i++)
			{
				tracked_joints_.push_back(float2(tracked_joints[2 * i], tracked_joints[2 * i + 1]));
			}
			vertices_.resize(VERTEX_COUNT);
		}

		void operator()(const Body& body, ShapeCoefficients& betas, PoseCoefficients& thetas, float3& translation, float3& scale)
		{
			#pragma omp parallel for
			for (uint i = 0; i < VERTEX_COUNT; i++)
			{
				Eigen::Vector3f v = Eigen::Scaling(scale.ToEigen()).toDenseMatrix() * (body.vertices[i].ToEigen() + translation.ToEigen());
				vertices_[i] = float3(v);
			}
			Joints joints = joint_regressor_(vertices_);
			std::vector<float2> estimated_joints = projector_(joints);
			estimated_joints[0];
		}

	private:
		std::vector<float3> vertices_;
		std::vector<float2> tracked_joints_;
		const JointRegressor joint_regressor_;
		const Projector projector_;
	};
}