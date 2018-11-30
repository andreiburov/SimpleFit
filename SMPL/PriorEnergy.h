#pragma once

#include "Definitions.h"

namespace smpl
{
	class PriorEnergy
	{
	public:
		PriorEnergy();

		void ComputePosePriorError(const PoseEulerCoefficients& thetas, const float weight, Eigen::VectorXf& error) const;

		void ComputeBendPriorError(const PoseEulerCoefficients& thetas, const float weight, Eigen::VectorXf& error) const;

		void ComputeShapePriorError(const ShapeCoefficients& betas, const float weight, Eigen::VectorXf& error) const;

		void ComputePosePriorJacobian(const float weight, Eigen::MatrixXf& jacobian) const;

		void ComputeBendPriorJacobian(const PoseEulerCoefficients& thetas, const float weight, Eigen::MatrixXf& jacobian) const;

		void ComputeShapePriorJacobian(const float weight, Eigen::MatrixXf& jacobian) const;

	private:

		// bigger weight, worse the error
		const std::vector<float> pose_weight_per_theta_;
		// which thetas have the bend constraint
		const std::vector<float> bend_mask_per_theta_;
		// centering the exponent
		const std::vector<float> bend_bias_per_theta_;
		// mirroring the exponent
		const std::vector<float> bend_sign_per_theta_;
	};
}