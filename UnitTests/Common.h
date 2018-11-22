#pragma once

#include <map>
#include <cereal/cereal.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/map.hpp>
#include <cereal/experimental/eigen.hpp>
#include <Eigen/Eigen>
#include <SMPL.h>

struct TestConfiguration
{
	std::map<int, float> betas;
	std::map<std::string, smpl::float3> thetas;
	Eigen::Vector3f translation;
	std::string model_path;
	std::string output_path;
	std::string reconstruction_config;

	TestConfiguration() {};

	TestConfiguration(const std::string& path)
	{
		std::ifstream in(path);
		if (!in) MessageBoxA(nullptr, 
			std::string("File not found: ").append(path).c_str(),
			"Error", MB_OK);
		cereal::JSONInputArchive archive(in);

		Serialize(archive);
	}

	void Dump(const std::string& path)
	{
		std::ofstream out(path);
		cereal::JSONOutputArchive archive(out);

		Serialize(archive);
	}

	template <class Archive>
	void Serialize(Archive& archive)
	{
		archive(
			CEREAL_NVP(betas),
			CEREAL_NVP(thetas),
			CEREAL_NVP(translation),
			CEREAL_NVP(model_path),
			CEREAL_NVP(output_path),
			CEREAL_NVP(reconstruction_config)
		);
	}
};