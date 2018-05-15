#pragma once

#include "Utils.h"
#include "Definitions.h"
#include "JointProjector.h"
#include "Generator.h"

namespace smpl
{
	class Optimizer
	{
	public:
		Optimizer(const Generator& generate, const JointProjector& project, const std::vector<float>& tracked_joints) :
			generate_(generate), project_(project), tracked_joints_(tracked_joints)
		{
		}

		void operator()(ShapeCoefficients& betas, PoseCoefficients& thetas, float3& scaling, float3& translation)
		{
			Body body = generate_(betas, thetas);
			std::vector<float> joints = project_(body, scaling, translation);

			// f(x) = f(a) - f'(a) * (x-a)
			// we will try to do least squares on tracked and estimated joints
			// for this one has to calculate derivative of projected smpl w.r.t. betas and thetas
		}

	private:
		const Generator generate_;
		const JointProjector project_;
		const std::vector<float> tracked_joints_;
	};
}