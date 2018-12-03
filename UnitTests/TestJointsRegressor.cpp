#include <SMPL.h>
#include <catch.hpp>
#include "Common.h"

using namespace smpl;

TEST_CASE("Estimate for depth")
{
	const std::string model_path("../Model/");
	Generator generator(model_path);
	Projector projector(model_path);
	JointsRegressor joints_regressor(
		JointsRegressor::Configuration(model_path, JointsRegressor::COCO));

	Body body = generator();
	RegressedJoints joints = joints_regressor(body.vertices);

	float model_avg_shoulders_to_hips_norm = ((joints.col(COCO_SHOULDER_LEFT) - joints.col(COCO_HIP_LEFT)) + 
		(joints.col(COCO_SHOULDER_RIGHT) - joints.col(COCO_HIP_RIGHT))).norm();

	float _x = (joints.col(COCO_SHOULDER_LEFT).x() - joints.col(COCO_HIP_LEFT).x()) +
		(joints.col(COCO_SHOULDER_RIGHT).x() - joints.col(COCO_HIP_RIGHT).x());
	float _y = (joints.col(COCO_SHOULDER_LEFT).y() - joints.col(COCO_HIP_LEFT).y()) +
		(joints.col(COCO_SHOULDER_RIGHT).y() - joints.col(COCO_HIP_RIGHT).y());
	float _z = (joints.col(COCO_SHOULDER_LEFT).z() - joints.col(COCO_HIP_LEFT).z()) +
		(joints.col(COCO_SHOULDER_RIGHT).z() - joints.col(COCO_HIP_RIGHT).z());

	float model_avg_shoulders_to_hips_norm_manual = sqrtf(_x * _x + _y * _y);
	float debug = sqrtf(_x * _x + _y * _y + _z * _z);

	std::cout << "model_avg_shoulders_to_hips_norm: " << model_avg_shoulders_to_hips_norm << std::endl;
	std::cout << "model_avg_shoulders_to_hips_norm_manual: " << model_avg_shoulders_to_hips_norm_manual << std::endl;
	std::cout << "debug: " << debug << std::endl;
}

TEST_CASE("Body25 Regressor From Different Views")
{
    TestConfiguration test;
    SECTION("0") { test = TestConfiguration("Confs/body25_regressor_front0.json"); }
    SECTION("1") { test = TestConfiguration("Confs/body25_regressor_side0.json"); }
    SECTION("2") { test = TestConfiguration("Confs/body25_regressor_back0.json"); }
    SECTION("3") { test = TestConfiguration("Confs/body25_regressor_front1.json"); }
    SECTION("4") { test = TestConfiguration("Confs/body25_regressor_side1.json"); }
    SECTION("5") { test = TestConfiguration("Confs/body25_regressor_back1.json"); }

    Generator generator(test.model_path);
    Projector projector(test.model_path);
    JointsRegressor joints_regressor(
        JointsRegressor::Configuration(test.model_path, JointsRegressor::BODY25));
    ShapeCoefficients betas(test.betas);
    PoseEulerCoefficients thetas(test.thetas);

    Body body = generator(betas, thetas);
    RegressedJoints joints = joints_regressor(body.vertices);
    std::vector<float> joints2d = Image::Coordinates(
        projector.FromRegressed(joints, test.translation));

    Image image;
    Image::Draw3D(image, Pixel::White(), projector, test.translation, body.vertices);
    Image::Draw2D(image, Pixel::Red(), 2, joints2d);
    image.SavePNG(test.output_path + "body.png");
}