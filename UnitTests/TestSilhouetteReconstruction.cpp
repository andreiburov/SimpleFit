#include <direct.h>  
#include <stdlib.h>
#include <thread>
#include <catch.hpp>
#include <EulerAngles.h>
#include <SMPL.h>

using namespace smpl;

Eigen::Matrix4f CreateNDC(float left, float right, float bottom, float top, float _near, float _far)
{
	// map a clipping space cube to directx ndc space
	float reciprocal_width = 1.0f / (right - left);
	float reciprocal_height = 1.0f / (top - bottom);
	float reciprocal_range = 1.0f / (_far - _near);

	Eigen::Matrix4f NDC(Eigen::Matrix4f::Zero());
	NDC(0, 0) = reciprocal_width + reciprocal_width;
	NDC(1, 1) = reciprocal_height + reciprocal_height;
	NDC(2, 2) = reciprocal_range;
	NDC(0, 3) = -(left + right) * reciprocal_width;
	NDC(1, 3) = -(top + bottom) * reciprocal_height;
	NDC(2, 3) = -reciprocal_range * _near;
	NDC(3, 3) = 1.f;
	return NDC;
}

Eigen::Matrix4f CreateProjection(const Eigen::Matrix3f& calibration, float _near, float _far)
{
	Eigen::Matrix4f projection(Eigen::Matrix4f::Zero());
	// intrinsics in pixel width (the unit does matter)
	projection(0, 0) = 861.407029f;
	projection(0, 2) = 323.240491f;
	projection(1, 1) = 830.867142f; 
	projection(1, 2) = 256.050359f;
	projection(2, 2) = 1.f;
	projection(3, 2) = 1.f;
	// we decided not to preserve the depth info, near and far not used
	return projection;
}

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
	view(1, 3) = 0.2f;
	view(2, 3) = 4.f;
	return view;
}

void AttachToProcess()
{
	int attach = 1; 
	while (attach) 
	{ 
 		std::this_thread::sleep_for(std::chrono::seconds(1)); 
	}
}

TEST_CASE("Hello World")
{
	WARN("Hello World");
}

TEST_CASE("Create Silhouette")
{
	Generator generator(Generator::Configuration(std::string("../Model")));
	Projector projector(Projector::Configuration(std::string("../Model")));
	SilhouetteOptimizer silhouette_optimizer(generator, projector);

	Eigen::Vector3f input_translation(0.f, 0.2f, 4.0f);
	ShapeCoefficients input_betas;
	PoseEulerCoefficients input_thetas;
	input_betas << std::string("-1 -5");
	Silhouette input = silhouette_optimizer.Infer("", input_translation, input_betas, input_thetas);

	Silhouette test(Silhouette::Loader("TestSilhouettes/input"));
	REQUIRE(input == test);
}

TEST_CASE("Find Correspondences")
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
	Silhouette model = silhouette_optimizer.Infer("", model_translation, model_betas, model_thetas);

	Correspondences correspondences = silhouette_optimizer.FindCorrespondences(input.GetImage(), model.GetImage(), model.GetNormals());

	Image test("TestSilhouettes/correspondences.png");
	REQUIRE(correspondences.image == test);
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
	input_thetas << std::string("0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0\
								 3.08425 -3.04993 -2.89101 2.77377 3.02695 -2.81275 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ");
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