#include <catch.hpp>
#include "SMPL.h"
#include "Common.h"

using namespace smpl;

TEST_CASE("Synthetic Body From Joints And Silhouette")
{
	TestConfiguration test;

	SECTION("0") { test = TestConfiguration("Confs/synthetic_body_from_joints_and_silhouette0.json"); }
	/*SECTION("1") { test = TestConfiguration("Confs/synthetic_body_from_joints_and_silhouette1.json"); }
	SECTION("2") { test = TestConfiguration("Confs/synthetic_body_from_joints_and_silhouette2.json"); }*/

	ShapeCoefficients input_betas(test.betas), betas;
	PoseEulerCoefficients input_thetas(test.thetas), thetas;
	Eigen::Vector3f input_translation(test.translation), translation;

	Reconstruction::Configuration conf(test.model_path, test.reconstruction_config);
	Reconstruction reconstruction(conf);

	const Generator& generator = reconstruction.GetGenerator();
	const Projector& projector = reconstruction.GetProjector();
	const JointsRegressor& joints_regressor = reconstruction.GetJointsRegressor();
	const SilhouetteRenderer& renderer = reconstruction.GetSilhouetteRenderer();

	Body input_body = generator(input_betas, input_thetas, true);
	std::vector<float> input_joints = Image::Coordinates(
		projector.FromRegressed(joints_regressor(input_body.vertices), input_translation));
	Silhouette input_silhouette = renderer(input_body,
		projector.CalculateView(input_translation),
		projector.DirectXProjection(IMAGE_WIDTH, IMAGE_HEIGHT));

	reconstruction.BodyFromJointsAndSilhouette(test.output_path, input_joints, 
		input_silhouette.GetImage(), translation, betas, thetas);

	input_silhouette.GetImage().SavePNG(test.output_path + "input_silhouette.png");
	Body body = generator(betas, thetas);
	input_body.Dump(test.output_path + "input_body.obj");
	body.Dump(test.output_path + "body.obj");
}