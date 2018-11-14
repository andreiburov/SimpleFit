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
			std::string configuration_path;

			// json
			int iterations_;
			std::string output_path_;
			std::vector<float> translation_;

			Configuration() {}

			Configuration(const std::string& configuration_path) :
				configuration_path(configuration_path)
			{
				std::string filename(configuration_path + std::string("/reconstruction.json"));
				std::ifstream in(filename);
				if (!in) MessageBoxA(nullptr, std::string("File not found: ").append(filename).c_str(), "Error", MB_OK);
				cereal::JSONInputArchive archive(in);

				Serialize(archive);
			}

			template<class Archive>
			void Serialize(Archive& archive)
			{
				archive(
					CEREAL_NVP(iterations_),
					CEREAL_NVP(output_path_),
					CEREAL_NVP(translation_)
				);
			}

			void Dump(const std::string& path)
			{
				std::ofstream out(path);
				cereal::JSONOutputArchive archive(out);

				Serialize(archive);
			}
		};

		Reconstruction(Configuration&& configuration);

		void BodyFromSilhouette(const Image& silhouette, ShapeCoefficients& betas, PoseEulerCoefficients& thetas) const;

		void BodyFromJoints(const std::vector<Point<int> >& joints, Eigen::Vector3f& translation, ShapeCoefficients& betas, PoseEulerCoefficients& thetas) const;

		void ShapeFromJoints(const std::vector<Point<int> >& joints, ShapeCoefficients& betas) const;

	private:

		const Generator generator_;
		const Projector projector_;
		const SilhouetteRenderer silhouette_renderer_;

		// configuration

		const std::string output_path_;
		const int iterations_;
		const std::vector<float> translation_;
	};
}