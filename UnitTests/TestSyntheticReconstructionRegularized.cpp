#include <catch.hpp>
#include "SMPL.h"
#include "Common.h"

using namespace smpl;

TEST_CASE("Synthetic Body Regularized From Joints")
{
	TestConfiguration test;

	SECTION("0") { test = TestConfiguration("Confs/synthetic_body_regularized_from_joints0.json"); }
	SECTION("1") { test = TestConfiguration("Confs/synthetic_body_regularized_from_joints1.json"); }
	SECTION("2") { test = TestConfiguration("Confs/synthetic_body_regularized_from_joints2.json"); }

	ShapeCoefficients input_betas(test.betas), betas;
	PoseEulerCoefficients input_thetas(test.thetas), thetas;
	Eigen::Vector3f input_translation(test.translation), translation;

	Reconstruction::Configuration conf(test.model_path, test.reconstruction_config);
	Reconstruction reconstruction(conf);

	const Generator& generator = reconstruction.GetGenerator();
	const Projector& projector = reconstruction.GetProjector();
	const JointsRegressor& joints_regressor = reconstruction.GetJointsRegressor();

	Body input_body = generator(input_betas, input_thetas);
	std::vector<float> input_joints = Image::Coordinates(
		projector.FromRegressed(joints_regressor(input_body.vertices), input_translation));

	reconstruction.BodyFromJointsRegularized(test.output_path, input_joints, translation, betas, thetas);

	Body body = generator(betas, thetas);
	body.Dump(test.output_path + "body.obj");
}