#pragma once

#include "Definitions.h"
#include "Image.h"
#include "Generator.h"
#include "Projector.h"
#include "SilhouetteRenderer.h"

#include <cereal/cereal.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/vector.hpp>
#include <fstream>

namespace smpl
{
	class Reconstruction
	{
	public:

		struct Configuration
		{
			bool logging_on_;
			float joints_weight_;
			int iterations_;
			std::string model_path_;
			std::string output_path_;
			std::vector<float> translation_vector_;
			
			Eigen::Vector3f translation_;

			Configuration() {}

			Configuration(const std::string& configuration_path)
			{
				std::ifstream in(configuration_path);
				if (!in) MessageBoxA(nullptr, std::string("File not found: ").append(configuration_path).c_str(), "Error", MB_OK);
				cereal::JSONInputArchive archive(in);

				Serialize(archive);

				translation_(0) = translation_vector_[0];
				translation_(1) = translation_vector_[1];
				translation_(2) = translation_vector_[2];
			}

			template<class Archive>
			void Serialize(Archive& archive)
			{
				archive(
					CEREAL_NVP(logging_on_),
					CEREAL_NVP(joints_weight_),
					CEREAL_NVP(iterations_),
					CEREAL_NVP(model_path_),
					CEREAL_NVP(output_path_),
					CEREAL_NVP(translation_vector_)
				);
			}

			void Dump(const std::string& path)
			{
				std::ofstream out(path);
				cereal::JSONOutputArchive archive(out);

				Serialize(archive);
			}
		};

		Reconstruction(const Configuration& configuration);

		void BodyFromSilhouette(const Image& silhouette, ShapeCoefficients& betas, PoseEulerCoefficients& thetas) const;

		void BodyFromJoints(
			const std::vector<float>& joints, Eigen::Vector3f& translation,
			ShapeCoefficients& betas, PoseEulerCoefficients& thetas) const;

		void ShapeFromJoints(const std::vector<float>& joints, ShapeCoefficients& betas) const;

	private:

		const Generator generator_;
		const Projector projector_;
		const JointsRegressor regressor_;
		const SilhouetteRenderer silhouette_renderer_;

		// configuration

		const bool logging_on_;
		const float joints_weight_;
		const int iterations_;
		const std::string model_path_;
		const std::string output_path_;
		const Eigen::Vector3f translation_;
	};
}