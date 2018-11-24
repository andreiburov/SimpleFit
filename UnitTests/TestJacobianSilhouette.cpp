#include <catch.hpp>
#include <SMPL.h>
#include "Common.h"

using namespace smpl;

TEST_CASE("Jacobian Silhouette From Shape")
{
	TestConfiguration conf;

	SECTION("0")
	{
		conf = TestConfiguration("Confs/jacobian_silhouette_from_shape0.json");
	}
	SECTION("1")
	{
		conf = TestConfiguration("Confs/jacobian_silhouette_from_shape1.json");
	}

	Generator generator(conf.model_path);
	Projector projector(conf.model_path);
	SilhouetteRenderer renderer(generator(true));
	SilhouetteEnergy silhouette_energy(generator, projector, renderer, 35, 5);

	Eigen::Vector3f translation(conf.translation);
	ShapeCoefficients input_betas(conf.betas), model_betas;
	PoseEulerCoefficients thetas;

	Body input_body = generator(input_betas, thetas, true);
	Silhouette input_silhouette = renderer(
		input_body, projector.CalculateView(translation),
		projector.DirectXProjection(IMAGE_WIDTH, IMAGE_HEIGHT));
	input_silhouette.GetImage().SavePNG(conf.output_path + "input_silhouette.png");

	Body model_body = generator(model_betas, thetas, true);
	Silhouette model_silhouette = renderer(
		model_body, projector.CalculateView(translation),
		projector.DirectXProjection(IMAGE_WIDTH, IMAGE_HEIGHT));
	model_silhouette.GetImage().SavePNG(conf.output_path + "model_silhouette.png");

	Correspondences correspondences = silhouette_energy.FindCorrespondences(
		input_silhouette.GetImage(),
		model_silhouette.GetImage(), model_silhouette.GetNormals());

	correspondences.image.SavePNG(conf.output_path + "correspondences.png");

	const int residuals = static_cast<int>(correspondences.input_border.size() * 2);
	std::vector<float3> dshape(VERTEX_COUNT * BETA_COUNT);
	Eigen::MatrixXf dsilhouette(residuals, BETA_COUNT);
	generator.ComputeBodyFromShapeJacobian(dshape);
	silhouette_energy.ComputeSilhouetteFromShapeJacobian(
		model_body, dshape, translation,
		model_silhouette, correspondences, 
		residuals, 1.f, dsilhouette);

	// print model silhouette border before the update
	{
		Image image;
		for (auto& p : correspondences.model_border)
		{
			image(p.x, p.y) = BLUE;
		}
		image.SavePNG(conf.output_path + "model_border_before.png");
	}

	for (int m = 0; m < residuals; m += 2)
	{
		float x = static_cast<float>(correspondences.model_border[m / 2].x);
		float y = static_cast<float>(correspondences.model_border[m / 2].y);

		for (int j = 0; j < BETA_COUNT; j++)
		{
			x += input_betas[j] * dsilhouette(m, j);
			y += input_betas[j] * dsilhouette(m + 1, j);
		}

		correspondences.model_border[m / 2].x = static_cast<int>(x);
		correspondences.model_border[m / 2].y = static_cast<int>(y);
	}

	// print model silhouette border after the update
	{
		Image image;
		for (auto& p : correspondences.model_border)
		{
			image(p.x, p.y) = BLUE;
		}
		image.SavePNG(conf.output_path + "model_border_after.png");
	}
}

TEST_CASE("Jacobian Silhouette From Pose")
{
	TestConfiguration conf;

	SECTION("0")
	{
		conf = TestConfiguration("Confs/jacobian_silhouette_from_pose0.json");
	}
	SECTION("1")
	{
		conf = TestConfiguration("Confs/jacobian_silhouette_from_pose1.json");
	}
	SECTION("2")
	{
		conf = TestConfiguration("Confs/jacobian_silhouette_from_pose2.json");
	}

	Generator generator(Generator::Configuration(conf.model_path));
	Projector projector(Projector::Configuration(conf.model_path));
	SilhouetteRenderer renderer(generator(true));
	SilhouetteEnergy silhouette_energy(generator, projector, renderer, 35, 5);

	Eigen::Vector3f translation(conf.translation);
	ShapeCoefficients betas;
	PoseEulerCoefficients input_thetas(conf.thetas), model_thetas;

	Body input_body = generator(betas, input_thetas, true);
	input_body.Dump(conf.output_path + "input_body.obj");
	Silhouette input_silhouette = renderer(
		input_body, projector.CalculateView(translation),
		projector.DirectXProjection(IMAGE_WIDTH, IMAGE_HEIGHT));
	input_silhouette.GetImage().SavePNG(conf.output_path + "input_silhouette.png");

	Body model_body = generator(betas, model_thetas);
	Silhouette model_silhouette = renderer(
		model_body, projector.CalculateView(translation),
		projector.DirectXProjection(IMAGE_WIDTH, IMAGE_HEIGHT));
	model_silhouette.GetImage().SavePNG(conf.output_path + "model_silhouette.png");

	Correspondences correspondences = silhouette_energy.FindCorrespondences(
		input_silhouette.GetImage(),
		model_silhouette.GetImage(), model_silhouette.GetNormals());
	silhouette_energy.PruneCorrepondences(input_silhouette.GetImage(),
		model_silhouette.GetImage(), model_silhouette.GetNormals(), correspondences);
	correspondences.image.SavePNG(conf.output_path + "correspondences.png");

	const int residuals = static_cast<int>(correspondences.input_border.size() * 2);
	std::vector<float3> dpose(VERTEX_COUNT * THETA_COMPONENT_COUNT);
	Eigen::MatrixXf dsillhouette(residuals, THETA_COMPONENT_COUNT);
	generator.ComputeBodyFromPoseJacobian(model_body, dpose);
	silhouette_energy.ComputeSilhouetteFromPoseJacobian(
		model_body, dpose, translation,
		model_silhouette, correspondences,
		residuals, 1.f, dsillhouette);

	// print model silhouette border before the update
	{
		Image image;
		for (auto& p : correspondences.model_border)
		{
			image(p.x, p.y) = BLUE;
		}
		image.SavePNG(conf.output_path + "model_border_before.png");
	}

	for (int m = 0; m < residuals; m += 2)
	{
		float x = static_cast<float>(correspondences.model_border[m / 2].x);
		float y = static_cast<float>(correspondences.model_border[m / 2].y);

		for (int k = 0; k < THETA_COMPONENT_COUNT; k++)
		{
			x += input_thetas(k) * dsillhouette(m, k);
			y += input_thetas(k) * dsillhouette(m + 1, k);
		}

		correspondences.model_border[m / 2].x = static_cast<int>(x);
		correspondences.model_border[m / 2].y = static_cast<int>(y);
	}

	// print model silhouette border after the update
	{
		Image image;
		for (auto& p : correspondences.model_border)
		{
			image(p.x, p.y) = BLUE;
		}
		image.SavePNG(conf.output_path + "model_border_after.png");
	}
}