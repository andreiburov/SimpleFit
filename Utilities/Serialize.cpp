#include "Utilities.h"

#include <cereal/cereal.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/vector.hpp>

#include <string>
#include <vector>
#include <fstream>
#include <iostream>

struct Configuration
{
	bool with_normals;
	std::string output_path;
	float precision;
	int iterations;
	std::vector<int> numbers;

	Configuration() {}

	Configuration(
		bool with_normals,
		std::string output_path,
		float precision,
		int iterations,
		std::vector<int> numbers
	) :
		with_normals(with_normals),
		output_path(output_path),
		precision(precision),
		iterations(iterations),
		numbers(numbers)
	{}

	template<class Archive>
	void Serialize(Archive& archive)
	{
		archive(
			CEREAL_NVP(with_normals),
			CEREAL_NVP(output_path),
			CEREAL_NVP(precision),
			CEREAL_NVP(iterations),
			CEREAL_NVP(numbers));
	}
};

int Serialize(int argc, char** argv)
{
	{
		std::ofstream out("configuration.json");
		cereal::JSONOutputArchive archive(out);

		Configuration configuration = { true, "hello world", 0.1f, 10, { 3, 2, 1 } };
		
		configuration.Serialize(archive);
	}  

	{
		std::ifstream in("configuration.json");
		cereal::JSONInputArchive archive(in);

		Configuration configuration;

		configuration.Serialize(archive);

		std::cout << configuration.output_path << std::endl;
	}

	return 0;
}