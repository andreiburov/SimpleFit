#include <SMPL.h>
#include <catch.hpp>

using namespace smpl;

TEST_CASE("Joints COCO")
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