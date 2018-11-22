#pragma once

#include "Definitions.h"
#include "Image.h"
#include "Generator.h"
#include "Projector.h"
#include "SilhouetteRenderer.h"

#include <cereal/cereal.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/experimental/eigen.hpp>
#include <fstream>

namespace smpl
{
	class Reconstruction
	{
	public:

		struct Configuration
		{
			float joints_weight;
			float silhouette_weight;
			int iterations;
			int ray_dist;
			std::string model_path;

			Configuration() {}

			Configuration(
				const std::string& model_path,
				const std::string& config_path) :
				model_path(model_path)
			{
				std::string config(model_path + config_path);
				std::ifstream in(config);
				if (!in) MessageBoxA(nullptr, 
					std::string("File not found: ").append(config).c_str(),
					"Error", MB_OK);
				cereal::JSONInputArchive archive(in);

				Serialize(archive);
			}

			template<class Archive>
			void Serialize(Archive& archive)
			{
				archive(
					CEREAL_NVP(joints_weight),
					CEREAL_NVP(silhouette_weight),
					CEREAL_NVP(iterations),
					CEREAL_NVP(ray_dist)
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

		void ShapeFromSilhouette(
			const std::string& output_path,
			const Image& input_silhouette,
			const Eigen::Vector3f& translation,
			ShapeCoefficients& betas) const;

		void PoseFromSilhouette(
			const std::string& output_path,
			const Image& input_silhouette,
			const Eigen::Vector3f& translation,
			PoseEulerCoefficients& thetas) const;

		void ShapeFromJoints(
			const std::string& output_path, 
			const std::vector<float>& input_joints, 
			const Eigen::Vector3f& translation,
			ShapeCoefficients& betas) const;

		void PoseFromJoints(
			const std::string& output_path,
			const std::vector<float>& input_joints,
			const Eigen::Vector3f& translation,
			PoseEulerCoefficients& thetas) const;

		const Generator& GetGenerator() const { return generator_; }
		const Projector& GetProjector() const { return projector_; }
		const JointsRegressor& GetJointsRegressor() const { return regressor_; }
		const SilhouetteRenderer& GetSilhouetteRenderer() const { return silhouette_renderer_; }

	private:

		const Generator generator_;
		const Projector projector_;
		const JointsRegressor regressor_;
		const SilhouetteRenderer silhouette_renderer_;

		// configuration
		const float joints_weight_;
		const float silhouette_weight_;
		const int iterations_;
		const int ray_dist_;
	};
}