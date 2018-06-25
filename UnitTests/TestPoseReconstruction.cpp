#include <catch.hpp>
#include <SMPL.h>
#include <Utils.h>
#include <vector>

using namespace smpl;

namespace pose_reconstruction
{
	float delta = 0.5f;

	ShapeCoefficients shape;
	PoseEulerCoefficients pose;
	Generator generator(smpl::Generator::Configuration(std::string("../Model")));
	SparseMatrix smpl_matrix;

	void TestPoseReconstruction(PoseEulerCoefficients& pose)
	{
		ZeroMemory(&shape, sizeof(shape));

		Body body2 = generator(shape, pose);
		body2.Dump("Body2.obj");

		ReadSparseMatrixFile("Model/smpl_regressor.txt", smpl_matrix);
		JointRegressor smpl_regressor(smpl_matrix, smpl::JOINT_COUNT);
		Joints smpl_joints = smpl_regressor(body2.vertices);

		std::vector<float> tracked_joints;
		tracked_joints.reserve(smpl::JOINT_COUNT * 3);
		for (UINT i = 0; i < smpl::JOINT_COUNT; i++)
		{
			tracked_joints.push_back(smpl_joints.col(i)(0));
			tracked_joints.push_back(smpl_joints.col(i)(1));
			tracked_joints.push_back(smpl_joints.col(i)(2));
		}

		smpl::Optimizer optimize(smpl::Optimizer::Configuration(std::string("../Model")),
			generator, tracked_joints);

		ZeroMemory(&pose, sizeof(pose));
		optimize.OptimizePoseFrom3D(smpl::Optimizer::JOINT_TYPE::SMPL, shape, pose);

		Body body1 = generator(shape, pose);
		body1.Dump("Body1.obj");
	}

	TEST_CASE("PR 16th in Gamma")
	{
		ZeroMemory(&pose, sizeof(pose));
		pose[16] = float3(0, 0, delta);

		TestPoseReconstruction(pose);
	}

	TEST_CASE("PR 16th")
	{
		ZeroMemory(&pose, sizeof(pose));
		pose[16] = float3(-delta, -delta, delta);

		TestPoseReconstruction(pose);
	}

	TEST_CASE("PR 16th and 18th")
	{
		ZeroMemory(&pose, sizeof(pose));

		//0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 - 0.824606 - 0.321571 0.432364 0 0 0 0.0898455 - 1.41424 0.540512 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
		//pose[16] = float3(-0.824606f, -0.321571f, 0.432364f);
		pose[18] = float3(0.0898455f, -1.41424f, 0.540512f);
		// 0 0 0 -0.874163 0.817765 0.0412538 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -0.673621 -0.970013 0.375232 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 
		//pose[16] = float3(-0.673621f, -0.970013f, 0.375232f);
		//pose[1] = float3(-0.874163f, 0.817765f, 0.0412538f);

		TestPoseReconstruction(pose);
	}
}