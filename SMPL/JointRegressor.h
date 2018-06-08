#pragma once

#include "Utils.h"

namespace smpl
{
	class JointRegressor
	{
	public:
		JointRegressor(const SparseMatrix& matrix, const uint& joint_count) :
			joint_count_(joint_count), matrix_(std::move(matrix)), eigen_matrix_(matrix_.ToEigen(joint_count))
		{
		}

		Joints operator()(const std::vector<float3>& vertices) const
		{
			Joints joints(joint_count_, 3);
			// triplets are read line by line, thus row major
			Eigen::MatrixXf vertex_matrix = Eigen::Map<Eigen::MatrixXf, Eigen::RowMajor>((float*)(vertices.data()), 3, VERTEX_COUNT).transpose();
			joints = eigen_matrix_ * vertex_matrix;

			// joints have to be manipulated as vectors (columnwise)
			return joints.transpose();
		}

		Eigen::VectorXf GetRow(uint i) const
		{
			return Eigen::VectorXf(eigen_matrix_.row(i));
		}

	private:
		const uint joint_count_;
		const SparseMatrix matrix_;
		const Eigen::SparseMatrix<float> eigen_matrix_;
	};
}