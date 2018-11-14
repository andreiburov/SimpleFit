#include <catch.hpp>
#include <EulerAngles.h>
#include <SMPL.h>

Eigen::Matrix4f CreateView()
{
	/*
	-1	0	0	0
	0	1	0	.2
	0	0	-1	4.
	0	0	0	1
	*/

	Eigen::Matrix4f view(Eigen::Matrix4f::Identity());
	// mesh is facing from us, rotate 180 around y
	view(0, 0) = -1;
	view(1, 1) = 1;
	view(2, 2) = -1;
	// mesh should be put at distance and up the y
	view(1, 3) = 0.5f;
	view(2, 3) = 4.0f;
	/*view(0, 3) = 0.f;
	view(1, 3) = 0.3f;
	view(2, 3) = -4.5f;*/
	return view;
}

Eigen::Matrix4f CreateNDC(float _near, float _far)
{
	Eigen::Matrix4f ndc(Eigen::Matrix4f::Zero());

	// 0 x 0 ; width x height -> -1 x -1 ; 1 x 1
	ndc(0, 0) = 2.f / smpl::IMAGE_WIDTH;
	ndc(0, 2) = -1.f;
	ndc(1, 1) = 2.f / smpl::IMAGE_HEIGHT;
	ndc(1, 2) = -1.f;
	
	// RHS
	float range = -1.f / (_far - _near);
	ndc(2, 2) = range;
	ndc(2, 3) = range * _near;
	ndc(3, 2) = -1.f;

	return ndc;
}

using namespace smpl;

TEST_CASE("Create Silhouette from Shape")
{
	Generator generator(Generator::Configuration(std::string("../Model")));
	Projector projector(Projector::Configuration(std::string("../Model")));
	SilhouetteEnergy silhouette_optimizer(generator, projector);

	Eigen::Vector3f input_translation(0.f, 0.4f, 4.5f);
	ShapeCoefficients input_betas;
	PoseEulerCoefficients input_thetas;
	input_betas << std::string("-1 -5");
	Silhouette input = silhouette_optimizer.Infer("", input_translation, input_betas, input_thetas);

	Silhouette test(Silhouette::Loader("TestSilhouettes/input"));
	REQUIRE(input == test);
}

TEST_CASE("Create Silhouette from Pose")
{
	Projector projector(Projector::Configuration(std::string("../Model")));
	auto view = CreateView();
	auto projection = projector.
		GetDirectXProjection(static_cast<float>(IMAGE_WIDTH), static_cast<float>(IMAGE_HEIGHT));

	Generator generator(Generator::Configuration(std::string("../Model")));
	SilhouetteRenderer silhouette_renderer(generator(true));

	ShapeCoefficients input_betas;
	PoseEulerCoefficients input_thetas;

	SECTION("z rotation")
	{
		input_thetas[SHOULDER_RIGHT].z = 1.f;
		Body body = generator(input_betas, input_thetas, true);
		Silhouette silhouette = silhouette_renderer(body, view, projection);
		body.Dump("body.obj");
		silhouette.GetImage().SavePNG("silhouette.png");
		//Silhouette test(Silhouette::Loader("TestSilhouettes/input"));
		//REQUIRE(silhouette == test);
	}
}