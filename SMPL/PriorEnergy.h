#pragma once

#include "Definitions.h"

namespace smpl
{
	class PriorEnergy
	{
	public:
		PriorEnergy();

		void ComputePosePriorError(const PoseEulerCoefficients& thetas, Eigen::VectorXf error) const;

		void ComputePosePriorJacobian(Eigen::MatrixXf& jacobian) const;

	private:

		const float pose_prior_weight_ = 50.f;

		// priors per theta components
		const std::vector<float> pose_prior_per_theta_;
	};
}