//#include <direct.h>  
//#include <stdlib.h>
//#include <thread>
#include <catch.hpp>
#include <SMPL.h>
#include "Common.h"

using namespace smpl;

TEST_CASE("Synthetic Shape From Silhouette")
{
	TestConfiguration test;

	SECTION("0") { test = TestConfiguration("Confs/synthetic_shape_from_silhouette0.json"); }
	SECTION("1") { test = TestConfiguration("Confs/synthetic_shape_from_silhouette1.json"); }

	ShapeCoefficients input_betas(test.betas), betas;
	PoseEulerCoefficients thetas;
	Eigen::Vector3f translation(test.translation);

	Reconstruction::Configuration conf(test.model_path, test.reconstruction_config);
	Reconstruction reconstruction(conf);

	const Generator& generator = reconstruction.GetGenerator();
	const Projector& projector = reconstruction.GetProjector();
	const SilhouetteRenderer& renderer = reconstruction.GetSilhouetteRenderer();

	Body input_body = generator(input_betas, thetas, true);
	Silhouette input_silhouette = renderer(input_body,
		projector.CalculateView(translation),
		projector.DirectXProjection(IMAGE_WIDTH, IMAGE_HEIGHT));

	reconstruction.ShapeFromSilhouette(test.output_path, 
		input_silhouette.GetImage(), translation, betas);
}

TEST_CASE("Synthetic Pose From Silhouette")
{
	TestConfiguration test;

	SECTION("0") { test = TestConfiguration("Confs/synthetic_pose_from_silhouette0.json"); }
	//SECTION("1") { test = TestConfiguration("Confs/synthetic_pose_from_silhouette1.json"); }
	//SECTION("2") { test = TestConfiguration("Confs/synthetic_pose_from_silhouette2.json"); }

	ShapeCoefficients betas;
	PoseEulerCoefficients input_thetas(test.thetas), thetas;
	Eigen::Vector3f translation(test.translation);

	Reconstruction::Configuration conf(test.model_path, test.reconstruction_config);
	Reconstruction reconstruction(conf);

	const Generator& generator = reconstruction.GetGenerator();
	const Projector& projector = reconstruction.GetProjector();
	const SilhouetteRenderer& renderer = reconstruction.GetSilhouetteRenderer();

	Body input_body = generator(betas, input_thetas, true);
	Silhouette input_silhouette = renderer(input_body,
		projector.CalculateView(translation),
		projector.DirectXProjection(IMAGE_WIDTH, IMAGE_HEIGHT));

	reconstruction.PoseFromSilhouette(test.output_path,
		input_silhouette.GetImage(), translation, thetas);

	// 35, 65, 80
}

TEST_CASE("Synthetic Body From Silhouette")
{
	TestConfiguration test;

	SECTION("0") { test = TestConfiguration("Confs/synthetic_body_from_silhouette0.json"); }
	//SECTION("1") { test = TestConfiguration("Confs/synthetic_body_from_silhouette1.json"); }
	//SECTION("2") { test = TestConfiguration("Confs/synthetic_body_from_silhouette2.json"); }

	ShapeCoefficients input_betas(test.betas), betas;
	PoseEulerCoefficients input_thetas(test.thetas), thetas(test.thetas);
	Eigen::Vector3f translation(test.translation);

	Reconstruction::Configuration conf(test.model_path, test.reconstruction_config);
	Reconstruction reconstruction(conf);

	const Generator& generator = reconstruction.GetGenerator();
	const Projector& projector = reconstruction.GetProjector();
	const SilhouetteRenderer& renderer = reconstruction.GetSilhouetteRenderer();

	Body input_body = generator(input_betas, input_thetas, true);
	Silhouette input_silhouette = renderer(input_body,
		projector.CalculateView(translation),
		projector.DirectXProjection(IMAGE_WIDTH, IMAGE_HEIGHT));

	reconstruction.BodyFromSilhouette(test.output_path,
		input_silhouette.GetImage(), translation, betas, thetas);

	Body body = generator(betas, thetas, false);
	body.Dump(test.output_path + "body.obj");
}