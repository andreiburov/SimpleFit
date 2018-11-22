#include "Projector.h"

namespace smpl
{
	Projector::Projector(float* intrinsics) :
		intrinsics_(Eigen::Map<Eigen::Matrix3f, Eigen::RowMajor>(intrinsics, 3, 3).transpose())
	{
	}

	Projector::Projector(Configuration&& configuration) :
		intrinsics_(Eigen::Map<Eigen::Matrix3f, Eigen::RowMajor>(configuration.intrinsics, 3, 3).transpose()), 
		near_(configuration.near_), far_(configuration.far_), is_rhs_(configuration.is_rhs_)
	{
	}

	Eigen::Vector2f Projector::operator()(const Eigen::Vector3f& vertex) const
	{
		// since the camera looks at -Z
		// the focal lengths are also negative
		if (is_rhs_)
		{
			float x = -intrinsics_(0, 0) * vertex(0) / vertex(2) + intrinsics_(0, 2);
			float y = -intrinsics_(1, 1) * vertex(1) / vertex(2) + intrinsics_(1, 2);
			return Eigen::Vector2f(x, y);
		}

		// lhs
		Eigen::Vector3f t = intrinsics_ * vertex;
		return Eigen::Vector2f(t(0) / t(2), t(1) / t(2));
	}

	Eigen::Vector2f Projector::operator()
		(const Eigen::Vector3f& vertex, const Eigen::Vector3f& translation) const
	{
		if (is_rhs_)
		{
			return operator()(vertex + translation);
		}

		// lhs
		Eigen::Vector3f turn(-vertex.x(), vertex.y(), -vertex.z());
		return operator()(turn + translation);
	}

	std::vector<float> Projector::FromRegressed
	(const RegressedJoints& regressed_joints, const Eigen::Vector3f& translation) const
	{
		std::vector<float> result;
		result.reserve(regressed_joints.cols() * 2);

		for (uint j = 0; j < regressed_joints.cols(); j++)
		{
			Eigen::Vector2f projected = operator()(regressed_joints.col(j), translation);
			result.push_back(projected(0));
			result.push_back(projected(1));
		}

		return result;
	}

	Eigen::Vector2f Projector::Jacobian(const Eigen::Vector3f& t, const Eigen::Vector3f& dt) const
	{
		float z2 = t(2)*t(2);

		if (is_rhs_)
		{
			return Eigen::Vector2f(intrinsics_(0, 0) * (t(0)*dt(2) / z2 - dt(0) / t(2)),
				intrinsics_(1, 1) * (t(1)*dt(2) / z2 - dt(1) / t(2)));
		}
		
		return Eigen::Vector2f(intrinsics_(0, 0) * (dt(0) / t(2) - t(0)*dt(2) / z2), 
			intrinsics_(1, 1) * (dt(1) / t(2) - t(1)*dt(2) / z2));
	}

	Eigen::Matrix<float, 3, 2> Projector::Jacobian(const Eigen::Vector3f& t) const
	{
		float z2 = t(2) * t(2);
		
		if (is_rhs_)
		{
			float r2[6] = { -intrinsics_(0,0) / t(2), 0, intrinsics_(0,0) * t(0) / z2,
				0, -intrinsics_(1,1) / t(2), intrinsics_(1,1) * t(1) / z2 };
			return Eigen::Map<Eigen::MatrixXf, Eigen::RowMajor>(r2, 3, 2);
		}
		
		float r[6] = { intrinsics_(0,0) / t(2), 0, -intrinsics_(0,0) * t(0) / z2,
			0, intrinsics_(1,1) / t(2), -intrinsics_(1,1) * t(1) / z2 };
		return Eigen::Map<Eigen::MatrixXf, Eigen::RowMajor>(r, 3, 2);
	}

	Eigen::Matrix4f Projector::CalculateView(const Eigen::Vector3f& translation) const
	{
		Eigen::Matrix4f view(Eigen::Matrix4f::Identity());

		if (is_rhs_)
		{
			view(0, 3) = translation.x();
			view(1, 3) = translation.y();
			view(2, 3) = translation.z();
		}
		else
		{
			// mesh is facing from us in LHS, rotate 180 around y
			view(0, 0) = -1;
			view(1, 1) = 1;
			view(2, 2) = -1;

			view(0, 3) = translation.x();
			view(1, 3) = translation.y();
			view(2, 3) = translation.z();
		}

		return view;
	}

	Eigen::Matrix4f Projector::DirectXProjection(int width, int height) const
	{
		Eigen::Matrix4f intrinsics(Eigen::Matrix4f::Identity());
		intrinsics.block<3, 3>(0, 0) = intrinsics_;

		if (is_rhs_)
		{
			intrinsics(0, 0) *= -1.f;
			intrinsics(1, 1) *= -1.f;
		}

		Eigen::Matrix4f projection =
			CalculateNDC(static_cast<float>(width), static_cast<float>(height)) * intrinsics;

		return projection;
	}

	Eigen::Matrix4f Projector::CalculateNDC(float width, float height) const
	{
		// map a clipping space cube to directx ndc space
		Eigen::Matrix4f ndc(Eigen::Matrix4f::Zero());

		// 0 x 0 ; width x height -> -1 x -1 ; 1 x 1
		ndc(0, 0) = 2.f / width;
		ndc(0, 2) = -1.f;
		ndc(1, 1) = 2.f / height;
		ndc(1, 2) = -1.f;

		// RHS
		if (is_rhs_)
		{
			float range = far_ / (far_ - near_);
			ndc(2, 2) = range;
			ndc(2, 3) = near_ * range;
			ndc(3, 2) = 1.f;
			ndc *= -1.f; // changing the sign for clip test
		}
		else
		{
			float range = far_ / (far_ - near_);
			ndc(2, 2) = range;
			ndc(2, 3) = -near_ * range;
			ndc(3, 2) = 1.f;
		}

		return ndc;
	}
}