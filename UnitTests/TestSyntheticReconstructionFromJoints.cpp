#include <catch.hpp>
#include <SMPL.h>
#include "Common.h"

using namespace smpl;

TEST_CASE("Synthetic Translation From Joints")
{
	TestConfiguration test;

	SECTION("0") { test = TestConfiguration("Confs/synthetic_translation_from_joints0.json"); }
	SECTION("1") { test = TestConfiguration("Confs/synthetic_translation_from_joints1.json"); }

	Eigen::Vector3f input_translation(test.translation), translation(0.f, 0.f, 0.f);

	Reconstruction::Configuration conf(test.model_path, test.reconstruction_config);
	Reconstruction reconstruction(conf);

	const Generator& generator = reconstruction.GetGenerator();
	const Projector& projector = reconstruction.GetProjector();
	const JointsRegressor& joints_regressor = reconstruction.GetJointsRegressor();

	Body input_body = generator();
	std::vector<float> input_joints = Image::Coordinates(
		projector.FromRegressed(joints_regressor(input_body.vertices), input_translation));

	reconstruction.TranslationFromJoints(test.output_path, input_joints, translation);
}

TEST_CASE("Synthetic Shape From Joints")
{
	TestConfiguration test;

	SECTION("0") { test = TestConfiguration("Confs/synthetic_shape_from_joints0.json"); }
	SECTION("1") { test = TestConfiguration("Confs/synthetic_shape_from_joints1.json"); }

	ShapeCoefficients input_betas(test.betas), betas;
	PoseEulerCoefficients thetas;
	Eigen::Vector3f translation(test.translation);

	Reconstruction::Configuration conf(test.model_path, test.reconstruction_config);
	Reconstruction reconstruction(conf);

	const Generator& generator = reconstruction.GetGenerator();
	const Projector& projector = reconstruction.GetProjector();
	const JointsRegressor& joints_regressor = reconstruction.GetJointsRegressor();
	
	Body input_body = generator(input_betas, thetas);
	std::vector<float> input_joints = Image::Coordinates(
		projector.FromRegressed(joints_regressor(input_body.vertices), translation));

	reconstruction.ShapeFromJoints(test.output_path, input_joints, translation, betas);
}

TEST_CASE("Synthetic Pose From Joints")
{
	TestConfiguration test;

	SECTION("0") { test = TestConfiguration("Confs/synthetic_pose_from_joints0.json"); }
	SECTION("1") { test = TestConfiguration("Confs/synthetic_pose_from_joints1.json"); }
	SECTION("2") { test = TestConfiguration("Confs/synthetic_pose_from_joints2.json"); }

	ShapeCoefficients betas;
	PoseEulerCoefficients input_thetas(test.thetas), thetas;
	Eigen::Vector3f translation(test.translation);

	Reconstruction::Configuration conf(test.model_path, test.reconstruction_config);
	Reconstruction reconstruction(conf);

	const Generator& generator = reconstruction.GetGenerator();
	const Projector& projector = reconstruction.GetProjector();
	const JointsRegressor& joints_regressor = reconstruction.GetJointsRegressor();

	Body input_body = generator(betas, input_thetas);
	std::vector<float> input_joints = Image::Coordinates(
		projector.FromRegressed(joints_regressor(input_body.vertices), translation));

	reconstruction.PoseFromJoints(test.output_path, input_joints, translation, thetas);
}

TEST_CASE("Synthetic Body From Joints")
{
	TestConfiguration test;

	SECTION("0") { test = TestConfiguration("Confs/synthetic_body_from_joints0.json"); }
	SECTION("1") { test = TestConfiguration("Confs/synthetic_body_from_joints1.json"); }
	SECTION("2") { test = TestConfiguration("Confs/synthetic_body_from_joints2.json"); }

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

	reconstruction.BodyFromJoints(test.output_path, input_joints, translation, betas, thetas);
}