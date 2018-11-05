#include <direct.h>  
#include <stdlib.h>
#include <thread>
#include <catch.hpp>
#include <EulerAngles.h>
#include <SMPL.h>

using namespace smpl;

TEST_CASE("Find Correspondences")
{
	Generator generator(Generator::Configuration(std::string("../Model")));
	Projector projector(Projector::Configuration(std::string("../Model")));
	SilhouetteOptimizer silhouette_optimizer(generator, projector);

	Eigen::Vector3f translation(0.f, 0.2f, 4.0f);
	ShapeCoefficients input_betas;
	PoseEulerCoefficients input_thetas;
	input_betas << std::string("-1 -5");
	Silhouette input = silhouette_optimizer.Infer("", translation, input_betas, input_thetas);

	ShapeCoefficients model_betas;
	PoseEulerCoefficients model_thetas;
	Silhouette model = silhouette_optimizer.Infer("", translation, model_betas, model_thetas);

	Correspondences correspondences = silhouette_optimizer.FindCorrespondences(input.GetImage(), model.GetImage(), model.GetNormals());

	Image test("TestSilhouettes/correspondences.png");
	REQUIRE(correspondences.image == test);
}

TEST_CASE("Prune Correspondences")
{
	Generator generator(Generator::Configuration(std::string("../Model")));
	Projector projector(Projector::Configuration(std::string("../Model")));
	SilhouetteOptimizer silhouette_optimizer(generator, projector);

	Eigen::Vector3f translation(0.f, 0.2f, 4.0f);
	ShapeCoefficients input_betas;
	PoseEulerCoefficients input_thetas;
	input_thetas[SHOULDER_RIGHT].z = 1.f;
	Silhouette input = silhouette_optimizer.Infer("", translation, input_betas, input_thetas);
	input.GetImage().SavePNG("input.png");

	ShapeCoefficients model_betas;
	PoseEulerCoefficients model_thetas;
	Silhouette model = silhouette_optimizer.Infer("", translation, model_betas, model_thetas);
	input.GetImage().SavePNG("model.png");

	Correspondences correspondences = silhouette_optimizer.FindCorrespondences(input.GetImage(), model.GetImage(), model.GetNormals());
	correspondences.image.SavePNG("pure_correspondences.png");
	silhouette_optimizer.PruneCorrepondences(input.GetImage(), model.GetImage(), model.GetNormals(), correspondences);
	correspondences.image.SavePNG("pruned_correspondences.png");
}

TEST_CASE("Jacobian Silhouette From Shape")
{
	Generator generator(Generator::Configuration(std::string("../Model")));
	Projector projector(Projector::Configuration(std::string("../Model")));
	SilhouetteOptimizer silhouette_optimizer(generator, projector);

	Eigen::Vector3f translation(0.f, 0.2f, 4.0f);
	ShapeCoefficients input_betas;
	PoseEulerCoefficients input_thetas;
	input_betas[0] = -1.f;
	input_betas[1] = -5.f;
	Body input_body = generator(input_betas, input_thetas);
	Silhouette input_silhouette = silhouette_optimizer.Infer("", translation, input_betas, input_thetas);
	input_silhouette.GetImage().SavePNG("input_silhouette.png");

	ShapeCoefficients model_betas;
	PoseEulerCoefficients model_thetas;
	Body model_body = generator(model_betas, model_thetas);
	Silhouette model_silhouette = silhouette_optimizer.Infer("", translation, model_betas, model_thetas);
	model_silhouette.GetImage().SavePNG("model_silhouette.png");

	Correspondences correspondences = silhouette_optimizer.FindCorrespondences(input_silhouette.GetImage(),
		model_silhouette.GetImage(), model_silhouette.GetNormals());

	std::vector<float3> dshape(VERTEX_COUNT * BETA_COUNT);
	generator.ComputeBodyFromShapeJacobian(dshape);

	const int residuals = correspondences.input_border.size() * 2;
	Eigen::MatrixXf dsillhouette_shape(residuals, BETA_COUNT);
	silhouette_optimizer.ComputeSilhouetteFromShapeJacobian(model_body, dshape, translation,
		model_silhouette, correspondences, residuals, dsillhouette_shape);

	// print model silhouette border before the update
	{
		Image image;
		for (auto& p : correspondences.model_border)
		{
			image(p.x, p.y) = BLUE;
		}
		image.SavePNG("model_border_before.png");
	}

	for (uint m = 0; m < residuals; m += 2)
	{
		float x = correspondences.model_border[m / 2].x;
		float y = correspondences.model_border[m / 2].y;

		for (uint j = 0; j < BETA_COUNT; j++)
		{
			x += input_betas[j] * dsillhouette_shape(m, j);
			y += input_betas[j] * dsillhouette_shape(m + 1, j);
		}

		correspondences.model_border[m / 2].x = x;
		correspondences.model_border[m / 2].y = y;
	}

	// print model silhouette border after the update
	{
		Image image;
		for (auto& p : correspondences.model_border)
		{
			image(p.x, p.y) = BLUE;
		}
		image.SavePNG("model_border_after.png");
	}
}

TEST_CASE("Jacobian Silhouette From Pose")
{
	Generator generator(Generator::Configuration(std::string("../Model")));
	Projector projector(Projector::Configuration(std::string("../Model")));
	SilhouetteOptimizer silhouette_optimizer(generator, projector);

	Eigen::Vector3f translation(0.f, 0.2f, 4.0f);
	ShapeCoefficients input_betas;
	PoseEulerCoefficients input_thetas;
	input_thetas[HIP_RIGHT].z = 1.f;
	Body input_body = generator(input_betas, input_thetas);
	input_body.Dump("input_body.obj");
	Silhouette input_silhouette = silhouette_optimizer.Infer("", translation, input_betas, input_thetas);
	input_silhouette.GetImage().SavePNG("input_silhouette.png");

	ShapeCoefficients model_betas;
	PoseEulerCoefficients model_thetas;
	Body model_body = generator(model_betas, model_thetas);
	Silhouette model_silhouette = silhouette_optimizer.Infer("", translation, model_betas, model_thetas);
	model_silhouette.GetImage().SavePNG("model_silhouette.png");

	Correspondences correspondences = silhouette_optimizer.FindCorrespondences(input_silhouette.GetImage(),
		model_silhouette.GetImage(), model_silhouette.GetNormals());
	silhouette_optimizer.PruneCorrepondences(input_silhouette.GetImage(),
		model_silhouette.GetImage(), model_silhouette.GetNormals(), correspondences);
	correspondences.image.SavePNG("correspondences.png");

	std::vector<float3> dpose(VERTEX_COUNT * THETA_COMPONENT_COUNT);
	generator.ComputeBodyFromPoseJacobian(model_body, dpose);

	const int residuals = correspondences.input_border.size() * 2;
	Eigen::MatrixXf dsillhouette_pose(residuals, THETA_COMPONENT_COUNT);
	silhouette_optimizer.ComputeSilhouetteFromPoseJacobian(model_body, dpose, translation,
		model_silhouette, correspondences, residuals, dsillhouette_pose);

	// print model silhouette border before the update
	{
		Image image;
		for (auto& p : correspondences.model_border)
		{
			image(p.x, p.y) = BLUE;
		}
		image.SavePNG("model_border_before.png");
	}

	for (uint m = 0; m < residuals; m += 2)
	{
		float x = correspondences.model_border[m / 2].x;
		float y = correspondences.model_border[m / 2].y;

		for (uint k = 0; k < THETA_COMPONENT_COUNT; k++)
		{
			x += input_thetas(k) * dsillhouette_pose(m, k);
			y += input_thetas(k) * dsillhouette_pose(m + 1, k);
		}

		correspondences.model_border[m / 2].x = x;
		correspondences.model_border[m / 2].y = y;
	}

	// print model silhouette border after the update
	{
		Image image;
		for (auto& p : correspondences.model_border)
		{
			image(p.x, p.y) = BLUE;
		}
		image.SavePNG("model_border_after.png");
	}
}

TEST_CASE("Reconstruct Shape From Synthetic Silhouette")
{
	Generator generator(Generator::Configuration(std::string("../Model")));
	Projector projector(Projector::Configuration(std::string("../Model")));
	SilhouetteOptimizer silhouette_optimizer(generator, projector);

	Eigen::Vector3f input_translation(0.f, 0.2f, 4.0f);
	ShapeCoefficients input_betas;
	PoseEulerCoefficients input_thetas;
	input_betas << std::string("-1 -5");
	Silhouette input = silhouette_optimizer.Infer("", input_translation, input_betas, input_thetas);

	Eigen::Vector3f model_translation(0.f, 0.2f, 4.0f);
	ShapeCoefficients model_betas;
	PoseEulerCoefficients model_thetas;

	silhouette_optimizer.ReconstructShape("SilhouetteSyntheticShapeImages/", input.GetImage(),
		model_translation, model_betas, model_thetas);

	Body body = generator(model_betas, model_thetas);
	body.Dump("from_silhouette.obj");
}

TEST_CASE("Reconstruct Pose From Synthetic Silhouette")
{
	Generator generator(Generator::Configuration(std::string("../Model")));
	Projector projector(Projector::Configuration(std::string("../Model")));
	SilhouetteOptimizer silhouette_optimizer(generator, projector);

	Eigen::Vector3f input_translation(0.f, 0.2f, 4.0f);
	ShapeCoefficients input_betas;
	PoseEulerCoefficients input_thetas;
	input_thetas[SHOULDER_RIGHT].z = 0.5f;
	Silhouette input = silhouette_optimizer.Infer("", input_translation, input_betas, input_thetas);
	input.GetImage().SavePNG("input_silhouette.png");

	Eigen::Vector3f model_translation(0.f, 0.2f, 4.0f);
	ShapeCoefficients model_betas;
	PoseEulerCoefficients model_thetas;

	silhouette_optimizer.ReconstructPose("SilhouetteSyntheticPoseImages/", input.GetImage(),
		model_translation, model_betas, model_thetas);

	Body body = generator(model_betas, model_thetas);
	body.Dump("model_body.obj");
}

