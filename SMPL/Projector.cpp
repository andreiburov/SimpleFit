#include "Projector.h"

namespace smpl
{
	Projector::Projector(float* intrinsics) :
		intrinsics_(Eigen::Map<Eigen::Matrix3f, Eigen::RowMajor>(intrinsics, 3, 3).transpose())
	{
	}

	Projector::Projector(Configuration configuration) :
		intrinsics_(Eigen::Map<Eigen::Matrix3f, Eigen::RowMajor>(configuration.intrinsics, 3, 3).transpose())
	{
	}

	Eigen::Vector2f Projector::operator()(const Eigen::Vector3f& vertex) const
	{
		Eigen::Vector3f t = intrinsics_ * vertex;
		return Eigen::Vector2f(t(0) / t(2), t(1) / t(2));
	}

	Eigen::Vector2f Projector::operator()(const Eigen::Vector3f& vertex, const Eigen::Vector3f& translation) const
	{
		Eigen::Vector3f t = intrinsics_ * (vertex + translation);
		return Eigen::Vector2f(t(0) / t(2), t(1) / t(2));
	}

	Eigen::Vector2f Projector::Jacobian(const Eigen::Vector3f& t, const Eigen::Vector3f& dt) const
	{
		float z2 = t(2)*t(2); // z squared
		return Eigen::Vector2f(intrinsics_(0, 0) * (dt(0) / t(2) - t(0)*dt(2) / z2), intrinsics_(1, 1) * (dt(1) / t(2) - t(1)*dt(2) / z2));
	}

	Eigen::Matrix<float, 3, 2> Projector::Jacobian(const Eigen::Vector3f& t) const
	{
		float z2 = t(2) * t(2);
		float r[6] = { intrinsics_(0,0) / t(2), 0, -intrinsics_(0,0) * t(0) / z2, 0, intrinsics_(1,1) / t(2), -intrinsics_(1,1) * t(1) / z2 };
		return Eigen::Map<Eigen::MatrixXf, Eigen::RowMajor>(r, 3, 2);
	}

	Eigen::Matrix4f Projector::GetDirectXProjection(float image_width, float image_height, float _near, float _far) const
	{
		// Pure intrinsics matrix flattens the z component
		// thus losing the depth information.
		// This is okay for us since the back_face culling is performed.

		Eigen::Matrix4f intrinsics4x4(Eigen::Matrix4f::Zero());
		intrinsics4x4.block<3, 3>(0, 0) = intrinsics_;
		intrinsics4x4(3, 2) = 1.f;

		return CalculateNDC(0.f, image_width, 0.f, image_height, _near, _far)*intrinsics4x4;
	}

	Eigen::Matrix4f Projector::GetDirectXProjection(float image_width, float image_height) const
	{
		return GetDirectXProjection(image_width, image_height, near_, far_);
	}

	Eigen::Matrix4f Projector::CalculateNDC(float left, float right, float bottom, float top, float _near, float _far) const
	{
		// map a clipping space cube to directx ndc space
		float reciprocal_width = 1.0f / (right - left);
		float reciprocal_height = 1.0f / (top - bottom);
		float reciprocal_range = 1.0f / (_far - _near);

		Eigen::Matrix4f NDC(Eigen::Matrix4f::Zero());
		NDC(0, 0) = reciprocal_width + reciprocal_width;
		NDC(1, 1) = reciprocal_height + reciprocal_height;
		NDC(2, 2) = reciprocal_range;
		NDC(0, 3) = -(left + right) * reciprocal_width;
		NDC(1, 3) = -(top + bottom) * reciprocal_height;
		NDC(2, 3) = -reciprocal_range * _near;
		NDC(3, 3) = 1.f;
		return NDC;
	}
}