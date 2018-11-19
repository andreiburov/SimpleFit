#include "JointsRegressor.h"

namespace smpl
{
	RegressedJoints JointsRegressor::operator()(const std::vector<float3>& vertices) const
	{
		RegressedJoints joints(joint_count_, 3);

		assert(VERTEX_COUNT == vertices.size());

		// triplets are read line by line, thus row major
		Eigen::MatrixXf vertex_matrix = 
			Eigen::Map<Eigen::MatrixXf, Eigen::RowMajor>
			((float*)(vertices.data()), 3, VERTEX_COUNT).transpose();
		joints = eigen_matrix_ * vertex_matrix;

		// joints have to be manipulated as vectors (columnwise)
		return joints.transpose();
	}

	std::vector<float> JointsRegressor::Linearized(const std::vector<float3>& vertices) const
	{
		std::vector<float> result;
		auto joints = operator()(vertices);
		result.reserve(joints.rows() * joints.cols());

		for (uint j = 0; j < joints.cols(); j++)
		{
			for (uint i = 0; i < joints.rows(); i++)
			{
				result.push_back(joints(i, j));
			}
		}

		return result;
	}
}