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
			BODY25
		};

		struct Configuration
		{
			SparseMatrix regressor;
			JointType joint_type;
			int joint_count;

            static JointType String2JointType(const std::string& string)
            {
                if (string.compare("coco") == 0)
                    return COCO;
                else if (string.compare("smpl") == 0)
                    return SMPL;
                else if (string.compare("body25") == 0)
                    return BODY25;
                else
                {
                    MessageBoxA(nullptr, "Invalid regressor in reconstruction configuration", "Error", MB_OK);
                    throw std::exception("Invalid regressor");
                }
            }

			Configuration(const std::string& model_path, const std::string& joint_type) :
                Configuration(model_path, String2JointType(joint_type))
			{
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
				case BODY25:
                    ReadSparseMatrixFile(model_path + std::string("body25_regressor.txt"), regressor);
                    joint_count = BODY25_JOINT_COUNT;
					break;
				}	
			}
		};

		JointsRegressor(const Configuration& configuration) :
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