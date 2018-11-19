#include "JointsEnergy.h"

namespace smpl
{
	JointsEnergy::JointsEnergy(const Generator& generator, 
		const Projector& projector,
		const JointsRegressor& regressor) :
		generator_(generator), projector_(projector), regressor_(regressor)
	{
	}

	void JointsEnergy::ComputeError(const std::vector<float>& input_joints, 
		const RegressedJoints& model_joints, const Eigen::Vector3f translation,
		const int residuals, const float weight, Eigen::VectorXf& error) const
	{
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
		const Body& body, const std::vector<float3>& dshape, const Eigen::Vector3f& translation,
		const int residuals, const float weight, Eigen::MatrixXf& jacobian) const
	{
		RegressedJoints joints = regressor_(body.vertices);		

		for (int m = 0; m < residuals; m += 2)
		{
			int joint_index = m / 2;
			Eigen::Vector3f v = ToView(joints.col(joint_index), translation);
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
		const Body& body, const std::vector<float3>& dpose, const Eigen::Vector3f& translation,
		const int residuals, const float weight, Eigen::MatrixXf& jacobian) const
	{
		RegressedJoints joints = regressor_(body.vertices);

		for (int m = 0; m < residuals; m += 2)
		{
			int joint_index = m / 2;
			Eigen::Vector3f v = ToView(joints.col(joint_index), translation);
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
				Eigen::Vector2f dprojection = projector_.Jacobian(v, dv[k]);
				jacobian(m, k) = dprojection.x();
				jacobian(m + 1, k) = dprojection.y();
			}
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

	Eigen::Matrix4f JointsEnergy::CalculateView(Eigen::Vector3f translation) const
	{
		Eigen::Matrix4f view(Eigen::Matrix4f::Identity());

		// put mesh at negative distance
		view(0, 3) = translation.x();
		view(1, 3) = translation.y();
		view(2, 3) = translation.z();

		return view;
	}
}