#pragma once

#include "Definitions.h"
#include "Image.h"
#include "Generator.h"
#include "Projector.h"
#include "SilhouetteCorrespondences.h"
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
			float pose_prior_weight;
			float bend_prior_weight;
			float shape_prior_weight;

			int iterations;
			int ray_dist;
			int pruning_derivative_half_dx;
            std::string regressor;
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
                    CEREAL_NVP(pose_prior_weight),
                    CEREAL_NVP(bend_prior_weight),
                    CEREAL_NVP(shape_prior_weight),
                    CEREAL_NVP(iterations),
                    CEREAL_NVP(regressor),
					CEREAL_NVP(ray_dist),
					CEREAL_NVP(pruning_derivative_half_dx)
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

		void BodyFromSilhouette(const std::string& output_path,
			const Image& input_silhouette,
			const Eigen::Vector3f& translation,
			ShapeCoefficients& betas, PoseEulerCoefficients& thetas) const;

        void BodyFromJointsAndSilhouetteRegularized(
            const std::string& output_path,
            const std::vector<float>& input_joints, const Image& input_silhouette,
            Eigen::Vector3f& translation,
            ShapeCoefficients& betas, PoseEulerCoefficients& thetas) const;

		void BodyFromJointsAndSilhouette(
			const std::string& output_path,
			const std::vector<float>& input_joints, const Image& input_silhouette, 
			Eigen::Vector3f& translation, ShapeCoefficients& betas, PoseEulerCoefficients& thetas) const;

		void BodyFromJointsRegularized(
			const std::string& output_path,
			const std::vector<float>& input_joints, Eigen::Vector3f& translation,
			ShapeCoefficients& betas, PoseEulerCoefficients& thetas) const;

		void BodyFromJoints(
			const std::string& output_path,
			const std::vector<float>& input_joints, Eigen::Vector3f& translation,
			ShapeCoefficients& betas, PoseEulerCoefficients& thetas) const;

		void TranslationFromJoints(
			const std::string& output_path,
			const std::vector<float>& input_joints,
			Eigen::Vector3f& translation) const;

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

		void BodyFromBody(
			const std::string& output_path,
			const std::vector<float3>& input_body,
			const Eigen::Vector3f& translation,
			ShapeCoefficients& betas, PoseEulerCoefficients& thetas) const;

		const Generator& GetGenerator() const { return generator_; }
		const Projector& GetProjector() const { return projector_; }
		const JointsRegressor& GetJointsRegressor() const { return regressor_; }
		const SilhouetteRenderer& GetSilhouetteRenderer() const { return silhouette_renderer_; }

	private:

		const Generator generator_;
		const Projector projector_;
		const JointsRegressor regressor_;
		const SilhouetteRenderer silhouette_renderer_;
        const SilhouetteCorrespondencesFinder silhouette_correspondences_finder_;

		// configuration
		const float joints_weight_;
		const float silhouette_weight_;
		const float pose_prior_weight_;
		const float bend_prior_weight_;
		const float shape_prior_weight_;
		// increase the linear delta step
		const float shape_delta_weight_ = 1.3f;

		const int iterations_;
		const int ray_dist_;
		const int pruning_derivative_half_dx_;
	};
}