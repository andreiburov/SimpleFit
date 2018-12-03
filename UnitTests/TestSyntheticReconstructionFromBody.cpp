#include <catch.hpp>
#include <SMPL.h>
#include "Common.h"

using namespace smpl;

TEST_CASE("Synthetic Body From Body")
{
	TestConfiguration test;

	SECTION("0") { test = TestConfiguration("Confs/synthetic_body_from_body0.json"); }

	ShapeCoefficients input_betas(test.betas), betas;
	PoseEulerCoefficients input_thetas(test.thetas), thetas;
	Eigen::Vector3f translation(test.translation);

	Reconstruction::Configuration conf(test.model_path, test.reconstruction_config);
	Reconstruction reconstruction(conf);

	const Generator& generator = reconstruction.GetGenerator();

	Body input_body = generator(input_betas, input_thetas, false);

	reconstruction.BodyFromBody(test.output_path,
		input_body.vertices, translation, betas, thetas);

	input_body.Dump(test.output_path + "input_body.obj");
	Body body = generator(betas, thetas, false);
	body.Dump(test.output_path + "body.obj");
}