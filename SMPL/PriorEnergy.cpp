#include "PriorEnergy.h"

namespace smpl
{
	PriorEnergy::PriorEnergy() :
		pose_prior_per_theta_({
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
		2, 1, 1, //16
		2, 1, 1, //17
		100, 2, 1, //18
		100, 2, 1, //19
		100, 100, 100, //20
		100, 100, 100, //21
		100, 100, 100, //22
		100, 100, 100  //23
		})
	{
	}

	void PriorEnergy::ComputePosePriorError(const PoseEulerCoefficients& thetas, Eigen::VectorXf error) const
	{
#pragma omp parallel for
		for (int k = 0; k < THETA_COMPONENT_COUNT; k++)
		{
			error(k) = pose_prior_weight_ * pose_prior_per_theta_[k] * thetas(k);
		}
	}

	void PriorEnergy::ComputePosePriorJacobian(Eigen::MatrixXf& jacobian) const
	{
#pragma omp parallel for
		for (int k = 0; k < THETA_COMPONENT_COUNT; k++)
		{
			jacobian(k, k) = pose_prior_weight_ * pose_prior_per_theta_[k];
		}
	}
}