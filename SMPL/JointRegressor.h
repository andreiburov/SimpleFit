#pragma once

#include "Utils.h"

namespace smpl
{
	class JointRegressor
	{
	public:
		JointRegressor(const D3D& d3d, SparseMatrix& matrix) :
			d3d_(d3d), matrix_(std::move(matrix)), eigen_matrix_(matrix_.ToEigen())
		{
		}

		Joints operator()(const std::vector<float3>& vertices) const
		{
			Joints joints(JOINT_COUNT, 3);
			// triplets are read line by line, thus row major
			Eigen::MatrixXf vertex_matrix = Eigen::Map<Eigen::MatrixXf, Eigen::RowMajor>((float*)(vertices.data()), 3, VERTEX_COUNT).transpose();
			joints = eigen_matrix_ * vertex_matrix;

			// joints have to be manipulated as vectors (columnwise)
			return joints.transpose();
		}

	private:
		const D3D& d3d_;
		SparseMatrix matrix_;
		Eigen::SparseMatrix<float> eigen_matrix_;
	};
}