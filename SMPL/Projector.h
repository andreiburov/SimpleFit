#pragma once

#include "Utils.h"
#include "Definitions.h"

namespace smpl
{
	class Projector
	{
	public:
		Projector(float* intrinsics);

		Eigen::Vector2f operator()(const Eigen::Vector3f& vertex) const;

		Eigen::Vector2f operator()(const Eigen::Vector3f& vertex, const Eigen::Vector3f& translation) const;

		Eigen::Vector2f derivative(const Eigen::Vector3f& t, const Eigen::Vector3f& dt) const;

		Eigen::Matrix<float, 3, 2> derivative(const Eigen::Vector3f& t) const;

		const Eigen::Matrix3f& GetIntrinsics() const { return intrinsics_; }

	private:
		const Eigen::Matrix3f intrinsics_;
	};
}