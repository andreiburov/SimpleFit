#include <catch.hpp>
#include <SMPL.h>

using namespace smpl;

TEST_CASE("Jacobian Joints From Shape")
{
	Reconstruction::Configuration configuration("Configurations/jacobian_joints_from_shape.json");
	Generator generator(configuration.model_path_);
	Projector projector(configuration.model_path_);
	JointsRegressor joints_regressor(
		JointsRegressor::Configuration(configuration.model_path_, JointsRegressor::COCO));
	JointsEnergy joints_energy(generator, projector, joints_regressor);

	ShapeCoefficients input_betas, model_betas;
	input_betas[1] = -7.f;
	PoseEulerCoefficients thetas;
	Eigen::Vector3f translation(configuration.translation_);

	Body input_body = generator(input_betas, thetas);
	std::vector<float> input_joints = Image::Coordinates(
		projector.FromRegressed(joints_regressor(input_body.vertices), translation));

	Body model_body = generator(model_betas, thetas);
	std::vector<float> model_joints = Image::Coordinates(
		projector.FromRegressed(joints_regressor(model_body.vertices), translation));

	const int residuals = static_cast<int>(model_joints.size());

	std::vector<float3> dshape(VERTEX_COUNT * BETA_COUNT);
	generator.ComputeBodyFromShapeJacobian(dshape);	
	Eigen::MatrixXf djoints_shape(residuals, BETA_COUNT);
	joints_energy.ComputeJacobianFromShape(
		model_body, dshape, translation, 
		residuals, 1.f, djoints_shape);

	// print model before the update
	{
		Image image;
		Image::Draw3D(image, Pixel::White(), projector, translation, model_body.vertices);
		Image::Draw2D(image, Pixel::Blue(), model_joints);
		Image::Draw2D(image, Pixel::Yellow(), input_joints);
		image.SavePNG(configuration.output_path_ + "model_before.png");
	}

	for (int m = 0; m < residuals; m++)
	{
		for (int j = 0; j < BETA_COUNT; j++)
		{
			model_joints[m] += input_betas[j] * djoints_shape(m, j);
		}
	}

	// print model after the update
	{
		Image image;
		Image::Draw3D(image, Pixel::Red(), projector, translation, input_body.vertices);
		Image::Draw2D(image, Pixel::Blue(), model_joints);
		Image::Draw2D(image, Pixel::Yellow(), input_joints);
		image.SavePNG(configuration.output_path_ + "model_after.png");
	}
}