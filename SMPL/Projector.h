#pragma once

#include "Utils.h"
#include "Definitions.h"

namespace smpl
{
	class Projector
	{
	public:
		Projector(float* intrinsics) :
			intrinsics_(Eigen::Map<Eigen::Matrix3f, Eigen::RowMajor>(intrinsics, 3, 3).transpose())
		{
		}

		Eigen::Vector2f operator()(const Eigen::Vector3f& vertex) const
		{
			Eigen::Vector3f t = intrinsics_ * vertex;
			return Eigen::Vector2f(t(0) / t(2), t(1) / t(2));
		}

		Eigen::Vector2f operator()(const Eigen::Vector3f& vertex,
			const Eigen::Vector3f& scaling, const Eigen::Vector3f& translation) const
		{
			Eigen::Vector3f t = intrinsics_ * (Eigen::Scaling(scaling) * vertex + translation);
			return Eigen::Vector2f(t(0) / t(2), t(1) / t(2));
		}

		Eigen::Vector2f derivative(const Eigen::Vector3f& t, const Eigen::Vector3f& dt) const
		{
			float z2 = t(2)*t(2); // z squared
			return Eigen::Vector2f(intrinsics_(0, 0) * (dt(0) / t(2) - t(0)*dt(2) / z2), intrinsics_(1, 1) * (dt(1) / t(2) - t(1)*dt(2) / z2));
		}

		Eigen::Matrix<float, 3, 2> derivative(const Eigen::Vector3f& t) const
		{
			float z2 = t(2) * t(2);
			float r[6] = { intrinsics_(0,0) / t(2), 0, -intrinsics_(0,0) * t(0) / z2, 0, intrinsics_(1,1) / t(2), -intrinsics_(1,1) * t(1) / z2 };
			return Eigen::Map<Eigen::MatrixXf, Eigen::RowMajor>(r, 3, 2);
		}

		const Eigen::Matrix3f& GetIntrinsics() const { return intrinsics_; }

	private:
		const Eigen::Matrix3f intrinsics_;
	};
}