#include <catch.hpp>
#include <SMPL.h>

using namespace smpl;

TEST_CASE("Synthetic Shape From Joints")
{
	Reconstruction::Configuration configuration("Configurations/synthetic_shape_from_joints.json");
	Reconstruction reconstruction(configuration);
	Generator generator(configuration.model_path_);
	Projector projector(configuration.model_path_);
	JointsRegressor joints_regressor(
		JointsRegressor::Configuration(configuration.model_path_, JointsRegressor::COCO));

	ShapeCoefficients input_betas, model_betas;
	PoseEulerCoefficients thetas;
	Eigen::Vector3f translation(configuration.translation_);

	SECTION("Heavy weight")
	{
		input_betas[1] = -7.f;

		Body input_body = generator(input_betas, thetas);
		std::vector<float> input_joints = Image::Coordinates(
			projector.FromRegressed(joints_regressor(input_body.vertices), translation));

		reconstruction.ShapeFromJoints(input_joints, model_betas);
		Body model_body = generator(model_betas, thetas);

		model_body.Dump(configuration.output_path_ + "model_body.obj");
	}
}

TEST_CASE("Draw the Body")
{
	Reconstruction::Configuration configuration("Configurations/synthetic_shape_from_joints.json");

	ShapeCoefficients betas;
	PoseEulerCoefficients thetas;
	Generator generator(configuration.model_path_);
	Projector projector(configuration.model_path_);
	Eigen::Vector3f translation(
		configuration.translation_[0], 
		configuration.translation_[1], 
		configuration.translation_[2]);
	JointsRegressor joints_regressor(
		JointsRegressor::Configuration(configuration.model_path_, JointsRegressor::COCO));

	betas[1] = -7.f;

	Body body = generator(betas, thetas);
	Body model = generator(ShapeCoefficients(), PoseEulerCoefficients());
	std::vector<float> joints = Image::Coordinates(
		projector.FromRegressed(joints_regressor(body.vertices), translation));
	std::vector<float> model_joints = Image::Coordinates(
		projector.FromRegressed(joints_regressor(model.vertices), translation));
	
	Image image;
	Image::Draw3D(image, Pixel::White(), projector, translation, model.vertices);
	Image::Draw2D(image, Pixel::Yellow(), joints);
	Image::Draw2D(image, Pixel::Blue(), model_joints);
	image.SavePNG("image.png");
}