#include "Utilities.h"

#include <cereal/cereal.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/boost_variant.hpp>
#include <cereal/types/map.hpp>

#include <cereal/experimental/cereal_optional_nvp.h>
#include <cereal/experimental/boost_optional.hpp>
#include <cereal/experimental/eigen.hpp>

#include <boost/optional.hpp>
#include <boost/variant.hpp>

#include <Eigen/Eigen>

#include <SMPL.h>

#include <string>
#include <vector>
#include <fstream>
#include <iostream>

struct OptionalConfiguration
{
	boost::optional<int> x;
	boost::optional<int> y;
	boost::optional<Eigen::Vector3f> vector;
	boost::optional<Eigen::Matrix4f> matrix;

	template<class Archive>
	void Serialize(Archive& archive)
	{
		archive(
			CEREAL_NVP(x),
			CEREAL_NVP(y),
			CEREAL_NVP(vector),
			CEREAL_NVP(matrix)
		);
	}
};

struct VariantConfiguration
{
	boost::variant<int, double, std::string> i;
	boost::variant<int, double, std::string> d;
	boost::variant<int, double, std::string> s;

	template <class Archive>
	void Serialize(Archive& archive)
	{
		archive(
			CEREAL_NVP(i),
			CEREAL_NVP(d),
			CEREAL_NVP(s)
		);
	}
};

struct TestConfiguration
{
	std::map<int, float> betas;
	std::map<std::string, smpl::float3> thetas;
	Eigen::Vector3f translation;
	std::string model_path;
	std::string output_path;

	template <class Archive>
	void Serialize(Archive& archive)
	{
		archive(
			CEREAL_NVP(betas),
			CEREAL_NVP(thetas),
			CEREAL_NVP(translation),
			CEREAL_NVP(model_path),
			CEREAL_NVP(output_path)
		);
	}
};

int Serialize(int argc, char** argv)
{
	{
		std::ofstream out("configuration.json");
		cereal::JSONOutputArchive archive(out);

		/*VariantConfiguration conf;
		conf.i = 12;
		conf.d = 2.;
		conf.s = "Hello";*/

		/*OptionalConfiguration conf;
		conf.vector = Eigen::Vector3f(1.f, -2.f, 3.f);*/

		TestConfiguration conf;
		//conf.betas[1] = -7.f;
		conf.thetas[std::string(smpl::JOINT_FROM_INDEX[smpl::SHOULDER_RIGHT])].z = 1.f;
		conf.translation = Eigen::Vector3f(1.f, 2.f, 3.f);
		conf.model_path = "Model";
		conf.output_path = "Out";

		conf.Serialize(archive);
	}

	{
		std::ifstream in("configuration.json");
		cereal::JSONInputArchive archive(in);

		//VariantConfiguration conf;
		//OptionalConfiguration conf;
		TestConfiguration conf;
		conf.Serialize(archive);

		std::cout << conf.thetas.begin()->first << " "
			<< smpl::GetIndexFromJoint(conf.thetas.begin()->first.c_str()) << " "
			<< conf.thetas.begin()->second
			<< std::endl;
	}

	return 0;
}