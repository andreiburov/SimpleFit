#include <catch.hpp>
#include <SMPL.h>
#include <Utils.h>
#include <vector>
#include <fstream>

using namespace smpl;

namespace shape_reconstruction
{
	ShapeCoefficients shape;
	PoseEulerCoefficients pose;
	Generator generator(smpl::Generator::Configuration(std::string("../Model")));
	Eigen::Vector3f scaling(1.f, 1.f, 1.f);
	Eigen::Vector3f	translation(0.f, 0.2f, -4.f);

	void LogBodyAndProjection(const Body& body, const Joints& joints, const Projector& project,
		const std::string& body_filename, const std::string& projection_filename)
	{
		body.Dump(body_filename);
		Image image;
		Image::Draw3D(image, project.GetIntrinsics(), scaling, translation, WHITE, body.vertices);
		Image::Draw3D(image, project.GetIntrinsics(), scaling, translation, BLUE, 2, Joints2Vector(joints));
		image.SavePNG(projection_filename);
	}

	void TestSyntheticShapeReconstruction2D(
		const smpl::Optimizer::JOINT_TYPE& joint_type, ShapeCoefficients& shape)
	{
		ZeroMemory(&pose, sizeof(pose));

		Optimizer::Configuration configuration("../Model");
		JointRegressor coco_regressor(configuration.coco_regressor, smpl::COCO_JOINT_COUNT);
		Projector project(configuration.intrinsics);

		Body body2 = generator(shape, pose);
		Joints joints2 = coco_regressor(body2.vertices);
		LogBodyAndProjection(body2, joints2, project, "Body2.obj", "Body2.png");

		std::vector<float> tracked_joints;
		tracked_joints.reserve(smpl::COCO_JOINT_COUNT * 2);
		for (UINT i = 0; i < smpl::COCO_JOINT_COUNT; i++)
		{
			auto j = project(joints2.col(i), scaling, translation);
			tracked_joints.push_back(j(0));
			tracked_joints.push_back(j(1));
		}

		smpl::Optimizer optimize(configuration, generator, tracked_joints);

		ZeroMemory(&shape, sizeof(shape));
		optimize.OptimizeShapeFromJoints2D(joint_type, std::string(""), 
			scaling, translation, pose, shape);

		Body body1 = generator(shape, pose);
		Joints joints1 = coco_regressor(body1.vertices);
		LogBodyAndProjection(body1, joints1, project, "Body1.obj", "Body1.png");
	}

	TEST_CASE("SR From File")
	{
		ZeroMemory(&shape, sizeof(shape));
		std::ifstream in("betas.txt");

		if (in.fail())
		{
			std::cerr << "The file betas.txt does not exist!\n";
		}

		std::string str;
		std::getline(in, str);
		shape << str;
		TestSyntheticShapeReconstruction2D(smpl::Optimizer::JOINT_TYPE::COCO, shape);
	}
}