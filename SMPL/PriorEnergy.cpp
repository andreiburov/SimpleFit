#include "PriorEnergy.h"

namespace smpl
{
	PriorEnergy::PriorEnergy() :
		pose_weight_per_theta_({
		10, 10, 10, // 0
		1, 10, 2, //1 
		1, 10, 2, //2
		2, 4, 4, //3
		1, 100, 100, //4
		1, 100, 100, //5
		2, 4, 4, //6
		100, 100, 100, //7
		100, 100, 100, //8
		10, 4, 4, //9
		100, 100, 100, //10
		100, 100, 100, //11
		1, 1, 2, //12
		10, 10, 10, //13
		10, 10, 10, //14
		4, 2, 4, //15
		10, 1, 1, //16
		10, 1, 1, //17
		100, 2, 1, //18
		100, 2, 1, //19
		100, 100, 100, //20
		100, 100, 100, //21
		100, 100, 100, //22
		100, 100, 100  //23
		}),
		bend_mask_per_theta_({
		0, 0, 0, //0 
		1, 0, 0, //1
		1, 0, 0, //2
		0, 0, 0, //3
		1, 0, 0, //4
		1, 0, 0, //5
		0, 0, 0, //6
		0, 0, 0, //7
		0, 0, 0, //8
		0, 0, 0, //9
		0, 0, 0, //10
		0, 0, 0, //11
		0, 0, 0, //12
		0, 0, 1, //13
		0, 0, 1, //14
		0, 0, 0, //15
		0, 1, 1, //16
		0, 1, 1, //17
		0, 0, 1, //18
		0, 0, 1, //19
		0, 0, 0, //20
		0, 0, 0, //21
		0, 0, 1, //22
		0, 0, 1  //23
		}),
		bend_bias_per_theta_({
		0, 0, 0, //0 
		0.75f, 0, 0, //1
		0.75f, 0, 0, //2
		0, 0, 0, //3
		2.5f, 0, 0, //4
		2.5f, 0, 0, //5
		0, 0, 0, //6
		0, 0, 0, //7
		0, 0, 0, //8
		0, 0, 0, //9
		0, 0, 0, //10
		0, 0, 0, //11
		0, 0, 0, //12
		0, 0, 0.75f, //13
		0, 0, 0.37f, //14
		0, 0, 0, //15
		0, 0.37f, 1.f, //16
		0, 2.5f, 1.5f, //17
		0, 0, 2.5f, //18
		0, 0, 0.f, //19
		0, 0, 0, //20
		0, 0, 0, //21
		0, 0, 0.37f, //22
		0, 0, 1.5f  //23
		})
	{
	}

	void PriorEnergy::ComputePosePriorError(
		const PoseEulerCoefficients& thetas, const float weight, Eigen::VectorXf& error) const
	{
		assert(error.size() == THETA_COMPONENT_COUNT);
#pragma omp parallel for
		for (int k = 0; k < THETA_COMPONENT_COUNT; k++)
		{
			error(k) = weight * pose_weight_per_theta_[k] * thetas(k);
		}
	}

	void PriorEnergy::ComputeBendPriorError(
		const PoseEulerCoefficients& thetas, const float weight, Eigen::VectorXf& error) const
	{
		assert(error.size() == THETA_COMPONENT_COUNT);
#pragma omp parallel for
		for (int k = 0; k < THETA_COMPONENT_COUNT; k++)
		{
			error(k) = weight * bend_mask_per_theta_[k] * expf(thetas(k) - bend_bias_per_theta_[k]);
		}
	}

	void PriorEnergy::ComputeShapePriorError(const ShapeCoefficients& betas, const float weight, Eigen::VectorXf& error) const
	{
		assert(error.size() == BETA_COUNT);

		for (int j = 0; j < BETA_COUNT; j++)
		{
			error(j) = weight * betas[j];
		}
	}

	void PriorEnergy::ComputePosePriorJacobian(const float weight, Eigen::MatrixXf& jacobian) const
	{
		assert(jacobian.rows() == THETA_COMPONENT_COUNT);
		assert(jacobian.cols() == THETA_COMPONENT_COUNT);

#pragma omp parallel for
		for (int k = 0; k < THETA_COMPONENT_COUNT; k++)
		{
			jacobian(k, k) = weight * pose_weight_per_theta_[k];
		}
	}

	void PriorEnergy::ComputeBendPriorJacobian(const PoseEulerCoefficients& thetas, const float weight, Eigen::MatrixXf& jacobian) const
	{
		assert(jacobian.rows() == THETA_COMPONENT_COUNT);
		assert(jacobian.cols() == THETA_COMPONENT_COUNT);

#pragma omp parallel for
		for (int k = 0; k < THETA_COMPONENT_COUNT; k++)
		{
			jacobian(k, k) = weight * bend_mask_per_theta_[k] * expf(thetas(k) - bend_bias_per_theta_[k]);
		}
	}

	void PriorEnergy::ComputeShapePriorJacobian(const float weight, Eigen::MatrixXf& jacobian) const
	{
		assert(jacobian.rows() == BETA_COUNT);
		assert(jacobian.cols() == BETA_COUNT);

		for (int j = 0; j < BETA_COUNT; j++)
		{
			jacobian(j, j) = weight;
		}
	}
}