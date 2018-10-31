#pragma once

#include "Utils.h"
#include "Definitions.h"

namespace smpl
{
	class Projector
	{
	public:
		struct Configuration
		{
			Configuration(const std::string& configuration_path)
			{
				std::ifstream intrinsics_file(configuration_path + std::string("/intrinsics.txt"));
				if (intrinsics_file.fail()) MessageBoxA(NULL, "File not found: intrinsics.txt", "Error", MB_OK);

				for (uint i = 0; i < 9; i++)
				{
					intrinsics_file >> intrinsics[i];
				}

			}

			float intrinsics[9];
		};

		Projector(float* intrinsics);

		Projector(Configuration configuration);

		Eigen::Vector2f operator()(const Eigen::Vector3f& vertex) const;

		Eigen::Vector2f operator()(const Eigen::Vector3f& vertex, const Eigen::Vector3f& translation) const;
		
		Eigen::Vector2f Jacobian(const Eigen::Vector3f& t, const Eigen::Vector3f& dt) const;

		Eigen::Matrix<float, 3, 2> Jacobian(const Eigen::Vector3f& t) const;

		const Eigen::Matrix3f& GetIntrinsics() const { return intrinsics_; }

		Eigen::Matrix4f GetDirectXProjection(float image_width, float image_height, float _near, float _far) const;

		Eigen::Matrix4f GetDirectXProjection(float image_width, float image_height) const;

	private:
		Eigen::Matrix4f CalculateNDC(float left, float right, float bottom, float top, float _near, float _far) const;

	private:
		const float near_ = 0.001f;
		const float far_ = 10.f;
		const Eigen::Matrix3f intrinsics_;
	};
}