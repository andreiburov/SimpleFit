#include "Camera.h"

Camera::Camera() :
position_(Eigen::Vector3f(0.0, 0.0, 2.0)),
focus_(Eigen::Vector3f(0.0, 0.0, 0.0)),
up_(Eigen::Vector3f(0.0, 1.0, 0.0))
{
	LookAt(position_, focus_, up_);
}

void Camera::LookAt(const Eigen::Vector3f& position, const Eigen::Vector3f& target, const Eigen::Vector3f& up)
{
	position_ = position;

	Eigen::Matrix3f R;
	R.col(2) = (position - target).normalized();
	R.col(0) = up.cross(R.col(2)).normalized();
	R.col(1) = R.col(2).cross(R.col(0));
	view_matrix_.topLeftCorner<3, 3>() = R.transpose();
	view_matrix_.topRightCorner<3, 1>() = -R.transpose() * position;
	view_matrix_(3, 3) = 1.0f;

	view_matrix_it_ = view_matrix_.inverse().transpose();
}

void Camera::SetPerspective(float fovY, float aspect, float _near, float _far)
{
	float theta = fovY * 0.5f;
	float range = _far - _near;
	float invtan = 1.f / tan(theta);

	projection_matrix_(0, 0) = invtan / aspect;
	projection_matrix_(1, 1) = invtan;
	projection_matrix_(2, 2) = -(_near + _far) / range;
	projection_matrix_(3, 2) = -1.f;
	projection_matrix_(2, 3) = -2.f * _near * _far / range;
	projection_matrix_(3, 3) = 0.f;
}

void Camera::Move(POINT& focus, RECT& rect)
{
	if (!active_camera_)
		return;

	float yaw = float(anchor_.x - focus.x) / rect.right * M_PI * 2.0f;
	float pitch = float(anchor_.y - focus.y) / rect.bottom * M_PI * 2.0f;

	Eigen::Vector3f focus2camera = position_ - focus_;
	focus2camera = Eigen::AngleAxisf(yaw, up_) * focus2camera;
	Eigen::Vector3f left = up_.cross(focus2camera.normalized()).normalized();
	focus2camera = Eigen::AngleAxisf(pitch, left) * focus2camera;
	up_ = focus2camera.normalized().cross(left).normalized();

	position_ = focus_ + focus2camera;
	anchor_ = focus;

	LookAt(position_, focus_, up_);
}

void Camera::Zoom(int& delta)
{
	Eigen::Vector3f camera2focus = focus_ - position_;
	float speed = 0.1f;

	if (delta > 0)
	{
		position_ += speed * camera2focus;
	}
	else
	{
		position_ -= speed * camera2focus;
	}

	LookAt(position_, focus_, up_);
}