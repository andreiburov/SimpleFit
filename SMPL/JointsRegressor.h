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
			JointType joint_type;
			int joint_count;

			Configuration(const std::string& model_path, const std::string& joint_type)
			{
				if (joint_type.compare("coco") == 0)
					Configuration(model_path, COCO);
				else if (joint_type.compare("smpl") == 0)
					Configuration(model_path, SMPL);
				else if (joint_type.compare("body25") == 0)
					Configuration(model_path, BODY_25);
				else
				{
					MessageBoxA(nullptr, "Invalid regressor in reconstruction configuration", "Error", MB_OK);
					throw std::exception("Invalid regressor");
				}
			}

			Configuration(const std::string& model_path, const JointType& joint_type) : joint_type(joint_type)
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
                    ReadSparseMatrixFile(model_path + std::string("body25_regressor.txt"), regressor);
                    joint_count = BODY_25_JOINT_COUNT;
					break;
				}	
			}
		};

		JointsRegressor(Configuration&& configuration) :
			JointsRegressor(
				configuration.regressor, 
				configuration.joint_type, 
				configuration.joint_count)
		{
		}

		JointsRegressor(const SparseMatrix& matrix, 
			const JointType joint_type, const int& joint_count) :
			matrix_(std::move(matrix)),
			joint_type_(joint_type), 
			joint_count_(joint_count), 
			eigen_matrix_(matrix_.ToEigen(joint_count))
		{
		}

		Eigen::VectorXf JointRegressor(int i) const
		{
			return Eigen::VectorXf(eigen_matrix_.row(i));
		}

		RegressedJoints operator()(const std::vector<float3>& vertices) const;

		std::vector<float> Linearized(const std::vector<float3>& vertices) const;

		const JointType& GetJointType() const { return joint_type_; }

	private:
		const SparseMatrix matrix_;
		const JointType joint_type_;
		const int joint_count_;
		const Eigen::SparseMatrix<float> eigen_matrix_;
	};
}