#include <catch.hpp>
#include <SMPL.h>
#include "Common.h"

using namespace smpl;

TEST_CASE("Jacobian Joints From Translation")
{
	TestConfiguration conf;

	SECTION("0")
	{
		conf = TestConfiguration("Confs/jacobian_joints_from_translation0.json");
	}
	SECTION("1")
	{
		conf = TestConfiguration("Confs/jacobian_joints_from_translation1.json");
	}

	Generator generator(conf.model_path);
	Projector projector(conf.model_path);
	JointsRegressor joints_regressor(
		JointsRegressor::Configuration(conf.model_path, JointsRegressor::COCO));
	JointsEnergy joints_energy(generator, projector, joints_regressor);

	ShapeCoefficients betas;
	PoseEulerCoefficients thetas;
	Eigen::Vector3f input_translation(conf.translation), translation, dtranslation(0.5f, 0.5f, 0.5f);

	Body input_body = generator(betas, thetas);
	std::vector<float> input_joints = Image::Coordinates(
		projector.FromRegressed(joints_regressor(input_body.vertices), input_translation));

	Body model_body = generator(betas, thetas);
	RegressedJoints regressed_model_joints = joints_regressor(model_body.vertices);

	joints_energy.InitializeCameraPosition(joints_regressor.GetJointType(),
		input_joints, projector.GetIntrinsics()(1, 1), translation);

	std::vector<float> model_joints = Image::Coordinates(
		projector.FromRegressed(regressed_model_joints, translation));

	const int residuals = static_cast<int>(model_joints.size());
	
	Eigen::MatrixXf djoints(residuals, 3);
	joints_energy.ComputeJacobianFromTranslation(
		model_body, regressed_model_joints, translation, residuals, 1.0f, djoints);

	// print model before the update
	{
		Image image;
		Image::Draw2D(image, Pixel::Yellow(), input_joints);
		Image::Draw2D(image, Pixel::Blue(), model_joints);
		image.SavePNG(conf.output_path + "model_before.png");
	}

	for (int m = 0; m < residuals; m++)
	{
		for (int j = 0; j < 3; j++)
		{
			model_joints[m] += dtranslation(j) * djoints(m, j);
		}
	}

	// print model after the update
	{
		Image image;
		Image::Draw2D(image, Pixel::Yellow(), input_joints);
		Image::Draw2D(image, Pixel::Blue(), model_joints);
		image.SavePNG(conf.output_path + "model_after.png");
	}
}

TEST_CASE("Jacobian Joints From Shape")
{
	TestConfiguration conf;
	
	SECTION("0")
	{
		conf = TestConfiguration("Confs/jacobian_joints_from_shape0.json");
	}
	SECTION("1")
	{
		conf = TestConfiguration("Confs/jacobian_joints_from_shape1.json");
	}

	Generator generator(conf.model_path);
	Projector projector(conf.model_path);
	JointsRegressor joints_regressor(
		JointsRegressor::Configuration(conf.model_path, JointsRegressor::COCO));
	JointsEnergy joints_energy(generator, projector, joints_regressor);

	ShapeCoefficients input_betas(conf.betas), model_betas;
	PoseEulerCoefficients thetas;
	Eigen::Vector3f translation(conf.translation);

	Body input_body = generator(input_betas, thetas);
	std::vector<float> input_joints = Image::Coordinates(
		projector.FromRegressed(joints_regressor(input_body.vertices), translation));

	Body model_body = generator(model_betas, thetas);
	RegressedJoints regressed_model_joints = joints_regressor(model_body.vertices);
	std::vector<float> model_joints = Image::Coordinates(
		projector.FromRegressed(regressed_model_joints, translation));

	const int residuals = static_cast<int>(model_joints.size());
	std::vector<float3> dshape(VERTEX_COUNT * BETA_COUNT);
	Eigen::MatrixXf djoints(residuals, BETA_COUNT);
	generator.ComputeBodyFromShapeJacobian(dshape);	
	joints_energy.ComputeJacobianFromShape(
		model_body, regressed_model_joints, 
		dshape, translation,
		residuals, 1.f, djoints);

	// print model before the update
	{
		Image image;
		Image::Draw3D(image, Pixel::White(), projector, translation, model_body.vertices);
		Image::Draw2D(image, Pixel::Yellow(), input_joints);
		Image::Draw2D(image, Pixel::Blue(), model_joints);
		image.SavePNG(conf.output_path + "model_before.png");
	}

	for (int m = 0; m < residuals; m++)
	{
		for (int j = 0; j < BETA_COUNT; j++)
		{
			model_joints[m] += input_betas[j] * djoints(m, j);
		}
	}

	// print model after the update
	{
		Image image;
		Image::Draw3D(image, Pixel::Red(), projector, translation, input_body.vertices);
		Image::Draw2D(image, Pixel::Yellow(), input_joints);
		Image::Draw2D(image, Pixel::Blue(), model_joints);
		image.SavePNG(conf.output_path + "model_after.png");
	}
}

TEST_CASE("Jacobian Joints From Pose")
{
	TestConfiguration conf;

	SECTION("0")
	{
		conf = TestConfiguration("Confs/jacobian_joints_from_pose0.json");
	}
	SECTION("1")
	{
		conf = TestConfiguration("Confs/jacobian_joints_from_pose1.json");
	}
	SECTION("2")
	{
		conf = TestConfiguration("Confs/jacobian_joints_from_pose2.json");
	}

	Generator generator(conf.model_path);
	Projector projector(conf.model_path);
	JointsRegressor joints_regressor(
		JointsRegressor::Configuration(conf.model_path, JointsRegressor::COCO));
	JointsEnergy joints_energy(generator, projector, joints_regressor);

	ShapeCoefficients betas;
	PoseEulerCoefficients input_thetas(conf.thetas), model_thetas;
	Eigen::Vector3f translation(conf.translation);

	Body input_body = generator(betas, input_thetas);
	std::vector<float> input_joints = Image::Coordinates(
		projector.FromRegressed(joints_regressor(input_body.vertices), translation));

	Body model_body = generator(betas, model_thetas);
	RegressedJoints regressed_model_joints = joints_regressor(model_body.vertices);
	std::vector<float> model_joints = Image::Coordinates(
		projector.FromRegressed(regressed_model_joints, translation));

	const int residuals = static_cast<int>(model_joints.size());
	std::vector<float3> dpose(VERTEX_COUNT * THETA_COMPONENT_COUNT);
	Eigen::MatrixXf djoints(residuals, THETA_COMPONENT_COUNT);
	generator.ComputeBodyFromPoseJacobian(model_body, dpose);
	joints_energy.ComputeJacobianFromPose(
		model_body, regressed_model_joints, dpose, translation,
		residuals, 1.f, djoints);

	// print model before the update
	{
		Image image;
		Image::Draw3D(image, Pixel::White(), projector, translation, model_body.vertices);
		Image::Draw2D(image, Pixel::Yellow(), input_joints);
		Image::Draw2D(image, Pixel::Blue(), model_joints);
		image.SavePNG(conf.output_path + "model_before.png");
	}

	for (int m = 0; m < residuals; m++)
	{
		for (int k = 0; k < THETA_COMPONENT_COUNT; k++)
		{
			model_joints[m] += input_thetas(k) * djoints(m, k);
		}
	}

	// print model after the update
	{
		Image image;
		Image::Draw3D(image, Pixel::Red(), projector, translation, input_body.vertices);
		Image::Draw2D(image, Pixel::Yellow(), input_joints);
		Image::Draw2D(image, Pixel::Blue(), model_joints);
		image.SavePNG(conf.output_path + "model_after.png");
	}
}