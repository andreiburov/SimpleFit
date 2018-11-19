#pragma once

#include "Utils.h"

namespace smpl
{
	class JointsRegressor
	{
	public:

		enum JointType
		{
			SMPL,
			COCO,
			BODY_25
		};

		struct Configuration
		{
			SparseMatrix regressor;
			uint joint_count;

			Configuration(const std::string& model_path, const JointType& joint_type)
			{
				switch (joint_type)
				{
				case SMPL:
					ReadSparseMatrixFile(model_path + std::string("smpl_regressor.txt"), regressor);
					joint_count = JOINT_COUNT;
					break;
				case COCO:
					ReadSparseMatrixFile(model_path + std::string("coco_regressor.txt"), regressor);
					joint_count = COCO_JOINT_COUNT;
					break;
				case BODY_25:
					MessageBoxA(nullptr, "BODY_25 regressor not implemented!", "Error", MB_OK);
					break;
				}	
			}
		};

		JointsRegressor(Configuration&& configuration) :
			JointsRegressor(configuration.regressor, configuration.joint_count)
		{
		}

		JointsRegressor(const SparseMatrix& matrix, const uint& joint_count) :
			joint_count_(joint_count), matrix_(std::move(matrix)),
			eigen_matrix_(matrix_.ToEigen(joint_count))
		{
		}

		Eigen::VectorXf JointRegressor(uint i) const
		{
			return Eigen::VectorXf(eigen_matrix_.row(i));
		}

		RegressedJoints operator()(const std::vector<float3>& vertices) const;

		std::vector<float> Linearized(const std::vector<float3>& vertices) const;

	private:
		const uint joint_count_;
		const SparseMatrix matrix_;
		const Eigen::SparseMatrix<float> eigen_matrix_;
	};
}