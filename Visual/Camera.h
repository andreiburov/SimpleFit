#pragma once
#include <Eigen/Eigen>
#include <windows.h>

#define M_PI 3.14159265358979323846f

class Camera
{
public:

	Camera();

	void LookAt(const Eigen::Vector3f& position, const Eigen::Vector3f& target, const Eigen::Vector3f& up);


	void SetPerspective(float aspect)
	{
		SetPerspective(90.0f, aspect, 0.001f, 10.0f);
	}

	void SetPerspective(float fovY, float aspect, float _near, float _far);

	void Move(POINT& focus, RECT& rect);

	void Zoom(int& delta);

	void* GetDataPointer()
	{
		return &view_matrix_;
	}

	unsigned int GetDataSize()
	{
		return sizeof(view_matrix_) + sizeof(view_matrix_it_) + sizeof(projection_matrix_);
	}

	void Activate(POINT& anchor)
	{
		active_camera_ = true;
		anchor_ = anchor;
	}

	void Deactivate()
	{
		active_camera_ = false;
	}
private:
	
	Eigen::Matrix4f view_matrix_;
	Eigen::Matrix4f view_matrix_it_;
	Eigen::Matrix4f projection_matrix_;
	Eigen::Vector3f position_;
	Eigen::Vector3f focus_;
	Eigen::Vector3f up_;

	POINT			anchor_;
	bool			active_camera_;
};