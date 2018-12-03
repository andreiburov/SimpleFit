#include "JointsEnergy.h"

namespace smpl
{
	JointsEnergy::JointsEnergy(const Generator& generator, 
		const Projector& projector,
		const JointsRegressor& regressor) :
		generator_(generator), projector_(projector), regressor_(regressor)
	{
	}

	void JointsEnergy::InitializeCameraPosition(
		const JointsRegressor::JointType& joint_type,
		const std::vector<float>& input_joints,
		const float focal_length_y, Eigen::Vector3f& translation) const
	{
		switch (joint_type)
		{
		case JointsRegressor::COCO:
		{
			float model_avg_shoulders_to_hips = 1.04932f;
			float _x = (input_joints[2 * COCO_SHOULDER_LEFT] - input_joints[2 * COCO_HIP_LEFT]) +
				(input_joints[2 * COCO_SHOULDER_RIGHT] - input_joints[2 * COCO_HIP_RIGHT]);
			float _y = (input_joints[2 * COCO_SHOULDER_LEFT + 1] - input_joints[2 * COCO_HIP_LEFT + 1]) +
				(input_joints[2 * COCO_SHOULDER_RIGHT + 1] - input_joints[2 * COCO_HIP_RIGHT + 1]);
			float image_avg_shoulders_to_hips = sqrtf(_x * _x + _y * _y);

			float z = model_avg_shoulders_to_hips / image_avg_shoulders_to_hips * focal_length_y;
			translation = Eigen::Vector3f(0.f, 0.f, -z);
			break;
		}
        case JointsRegressor::BODY25:
        {
            float model_avg_shoulders_to_hips = 1.04932f;
            float _x = (input_joints[2 * BODY25_SHOULDER_LEFT] - input_joints[2 * BODY25_HIP_LEFT]) +
                (input_joints[2 * BODY25_SHOULDER_RIGHT] - input_joints[2 * BODY25_HIP_RIGHT]);
            float _y = (input_joints[2 * BODY25_SHOULDER_LEFT + 1] - input_joints[2 * BODY25_HIP_LEFT + 1]) +
                (input_joints[2 * BODY25_SHOULDER_RIGHT + 1] - input_joints[2 * BODY25_HIP_RIGHT + 1]);
            float image_avg_shoulders_to_hips = sqrtf(_x * _x + _y * _y);

            float z = model_avg_shoulders_to_hips / image_avg_shoulders_to_hips * focal_length_y;
            translation = Eigen::Vector3f(0.f, 0.f, -z);
            break;
        }
		default:
			MessageBoxA(nullptr, "Guess Camera Position", "Error", MB_OK);
			throw std::exception("Not implemented");
		}
	}

	void JointsEnergy::ComputeError(const std::vector<float>& input_joints, 
		const RegressedJoints& model_joints, const Eigen::Vector3f& translation,
		const int residuals, const float weight, Eigen::VectorXf& error) const
	{
		assert(error.size() == residuals);

#pragma omp parallel for
		for (int m = 0; m < residuals; m += 2)
		{
			int joint_index = m / 2;
			Eigen::Vector2f joint = Image::Coordinate(
				projector_(ToView(model_joints.col(joint_index), translation)));
			error(m) = joint(0) - input_joints[m];
			error(m + 1) = joint(1) - input_joints[m + 1];
		}

		error *= weight;
	}

	void JointsEnergy::ComputeJacobianFromShape(
		const Body& body, const RegressedJoints& model_joints,
		const std::vector<float3>& dshape, const Eigen::Vector3f& translation,
		const int residuals, const float weight, Eigen::MatrixXf& jacobian) const
	{
		assert(jacobian.rows() == residuals);
		assert(jacobian.cols() == BETA_COUNT);

		for (int m = 0; m < residuals; m += 2)
		{
			int joint_index = m / 2;
			Eigen::Vector3f v = ToView(model_joints.col(joint_index), translation);
			Eigen::Vector3f dv[BETA_COUNT];
			ZeroMemory(dv, sizeof(dv));

			auto joint_regressor = regressor_.JointRegressor(joint_index);

#pragma omp parallel for
			for (int i = 0; i < VERTEX_COUNT; i++)
			{
				for (int j = 0; j < BETA_COUNT; j++)
				{
					dv[j] += ToView(joint_regressor(i) * dshape[i*BETA_COUNT + j].ToEigen());
				}
			}

			for (int j = 0; j < BETA_COUNT; j++)
			{
				Eigen::Vector2f djoint = Image::Jacobian(projector_.Jacobian(v, dv[j]));
				jacobian(m, j) = djoint.x();
				jacobian(m + 1, j) = djoint.y();
			}
		}

		jacobian *= weight;
	}

	void JointsEnergy::ComputeJacobianFromPose(
		const Body& body, const RegressedJoints& model_joints,
		const std::vector<float3>& dpose, const Eigen::Vector3f& translation,
		const int residuals, const float weight, Eigen::MatrixXf& jacobian) const
	{
		assert(jacobian.rows() == residuals);
		assert(jacobian.cols() == THETA_COMPONENT_COUNT);

		for (int m = 0; m < residuals; m += 2)
		{
			int joint_index = m / 2;
			Eigen::Vector3f v = ToView(model_joints.col(joint_index), translation);
			Eigen::Vector3f dv[THETA_COMPONENT_COUNT];
			ZeroMemory(dv, sizeof(dv));

			auto joint_regressor = regressor_.JointRegressor(joint_index);

#pragma omp parallel for
			for (int i = 0; i < VERTEX_COUNT; i++)
			{
				for (int k = 0; k < THETA_COMPONENT_COUNT; k++)
				{
					dv[k] += ToView(joint_regressor(i) * dpose[i*THETA_COMPONENT_COUNT + k].ToEigen());
				}
			}

			for (int k = 0; k < THETA_COMPONENT_COUNT; k++)
			{
				Eigen::Vector2f djoint = Image::Jacobian(projector_.Jacobian(v, dv[k]));
				jacobian(m, k) = djoint.x();
				jacobian(m + 1, k) = djoint.y();
			}
		}

		jacobian *= weight;
	}

	void JointsEnergy::ComputeJacobianFromTranslation(
		const Body& body, const RegressedJoints& model_joints, const Eigen::Vector3f& translation,
		const int residuals, const float weight, Eigen::MatrixXf& jacobian) const
	{
		assert(jacobian.rows() == residuals);
		assert(jacobian.cols() == 3);

		for (int m = 0; m < residuals; m += 2)
		{
			int joint_index = m / 2;
			Eigen::Vector3f v = ToView(model_joints.col(joint_index), translation);
			
			Eigen::Matrix<float, 3, 2> p = projector_.Jacobian(v);
			
			// reverse y derivative
			for (int i = 0; i < 3; i++) p(i, 1) *= -1.f;

			jacobian.block<2, 3>(m, 0) = p.transpose();
		}

		jacobian *= weight;
	}

	Eigen::Vector3f JointsEnergy::ToView(const Eigen::Vector3f& vertex, const Eigen::Vector3f& translation) const
	{
		return vertex + translation;
	}

	Eigen::Vector3f JointsEnergy::ToView(const Eigen::Vector3f& vertex) const
	{
		return vertex;
	}
}