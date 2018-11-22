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
			Configuration(const std::string& model_path)
			{
				std::ifstream intrinsics_file(model_path + std::string("intrinsics.txt"));
				if (intrinsics_file.fail()) MessageBoxA(NULL, "File not found: intrinsics.txt", "Error", MB_OK);

				for (uint i = 0; i < 9; i++)
				{
					intrinsics_file >> intrinsics[i];
				}

				std::ifstream projector_config(model_path + std::string("projector_configuration.txt"));
				if (intrinsics_file.fail()) MessageBoxA(NULL, "File not found: projector_configuration.txt", "Error", MB_OK);

				projector_config >> near_ >> far_ >> is_rhs_;
			}

			float intrinsics[9];

			float near_;
			float far_;
			int is_rhs_;
		};

		Projector(float* intrinsics);

		Projector(Configuration&& configuration);

		Eigen::Vector2f operator()(const Eigen::Vector3f& vertex) const;

		Eigen::Vector2f operator()
			(const Eigen::Vector3f& vertex, const Eigen::Vector3f& translation) const;

		std::vector<float> FromRegressed
			(const RegressedJoints& regressed_joints, const Eigen::Vector3f& translation) const;
		
		Eigen::Vector2f Jacobian(const Eigen::Vector3f& vertex, const Eigen::Vector3f& d_vertex) const;

		Eigen::Matrix<float, 3, 2> Jacobian(const Eigen::Vector3f& vertex) const;

		Eigen::Matrix4f CalculateView(const Eigen::Vector3f& translation) const;

		Eigen::Matrix4f DirectXProjection(int width, int height) const;

		const Eigen::Matrix3f& GetIntrinsics() const { return intrinsics_; }

		const int IsRhs() const { return is_rhs_; }

	private:
		Eigen::Matrix4f CalculateNDC(float width, float height) const;

	private:
		const Eigen::Matrix3f intrinsics_;
		const float near_ = 0.1f;
		const float far_ = 10.f;
		const int is_rhs_ = 1; // works as boolean
	};
}