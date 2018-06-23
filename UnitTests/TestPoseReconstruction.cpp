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

	TEST_CASE("PR 16th in Gamma")
	{
		// Read mesh to body
		// Regress to 3D smpl/coco joints
		// Run optimizer from 3D
		// Compare reconstructed and original mesh

		uint change = 16;

		ZeroMemory(&shape, sizeof(shape));
		ZeroMemory(&pose, sizeof(pose));

		pose[16] = float3(0, 0, delta);

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

		ZeroMemory(&pose, sizeof(pose));
		smpl::Optimizer optimize(smpl::Optimizer::Configuration(std::string("../Model")),
			generator, tracked_joints);

		optimize.OptimizePoseFrom3D(smpl::Optimizer::JOINT_TYPE::SMPL, shape, pose);
		
		Body body1 = generator(shape, pose);
		body1.Dump("Body1.obj");
	}

	TEST_CASE("PR 16th")
	{
		uint change = 16;

		ZeroMemory(&shape, sizeof(shape));
		ZeroMemory(&pose, sizeof(pose));

		pose[16] = float3(-delta, -delta, delta);

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

		ZeroMemory(&pose, sizeof(pose));
		smpl::Optimizer optimize(smpl::Optimizer::Configuration(std::string("../Model")),
			generator, tracked_joints);

		optimize.OptimizePoseFrom3D(smpl::Optimizer::JOINT_TYPE::SMPL, shape, pose);

		Body body1 = generator(shape, pose);
		body1.Dump("Body1.obj");
	}

	TEST_CASE("PR 16th and 18th")
	{
		ZeroMemory(&shape, sizeof(shape));
		ZeroMemory(&pose, sizeof(pose));

		//0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 - 0.824606 - 0.321571 0.432364 0 0 0 0.0898455 - 1.41424 0.540512 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
		pose[16] = float3(-0.824606f, -0.321571f, 0.432364f);
		pose[18] = float3(0.0898455f, -1.41424f, 0.540512f);

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

		ZeroMemory(&pose, sizeof(pose));
		smpl::Optimizer optimize(smpl::Optimizer::Configuration(std::string("../Model")),
			generator, tracked_joints);

		optimize.OptimizePoseFrom3D(smpl::Optimizer::JOINT_TYPE::SMPL, shape, pose);

		Body body1 = generator(shape, pose);
		body1.Dump("Body1.obj");
	}

	/*TEST_CASE("Right Hand Up")
	{
		
		ZeroMemory(&shape, sizeof(shape));

		float local_pose[smpl::JOINT_COUNT * 3] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.680623,-0.701102,0.87854,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
		for (UINT i = 0; i < smpl::JOINT_COUNT * 3; i++)
		{
			pose(i) = local_pose[i];
		}

		smpl::Generator generator(smpl::Generator::Configuration(std::string("Model")));
		smpl::Body body = generator(shape, pose);
		body.Dump("mesh.obj");

		smpl::SparseMatrix smpl_matrix;
		ReadSparseMatrixFile("Model/smpl_regressor.txt", smpl_matrix);
		smpl::JointRegressor smpl_regressor(smpl_matrix, smpl::JOINT_COUNT);
		smpl::Joints smpl_joints = smpl_regressor(body.vertices);

		std::vector<float> tracked_joints;
		tracked_joints.reserve(smpl::JOINT_COUNT * 3);
		for (UINT i = 0; i < smpl::JOINT_COUNT; i++)
		{
			tracked_joints.push_back(smpl_joints.col(i)(0));
			tracked_joints.push_back(smpl_joints.col(i)(1));
			tracked_joints.push_back(smpl_joints.col(i)(2));
		}

		SECTION("Solution is converged to from the bind pose")
		{
			ZeroMemory(&pose, sizeof(pose));
			smpl::Optimizer optimize(smpl::Optimizer::Configuration(std::string("Model")),
				smpl::Generator(smpl::Generator::Configuration(std::string("Model"))), tracked_joints);

			optimize.OptimizePoseFrom3D(smpl::Optimizer::JOINT_TYPE::SMPL, shape, pose);

			for (UINT i = 0; i < smpl::JOINT_COUNT * 3; i++)
			{
				REQUIRE(abs(local_pose[i] - pose(i)) < 0.01);
			}
		}

		SECTION("Solution does not diverge from the correct one")
		{
			smpl::Optimizer optimize(smpl::Optimizer::Configuration(std::string("Model")),
				smpl::Generator(smpl::Generator::Configuration(std::string("Model"))), tracked_joints);

			optimize.OptimizePoseFrom3D(smpl::Optimizer::JOINT_TYPE::SMPL, shape, pose);

			for (UINT i = 0; i < smpl::JOINT_COUNT * 3; i++)
			{
				REQUIRE(abs(local_pose[i] - pose(i)) < 0.01);
			}
		}
	}*/
}

// 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -0.680623 -0.701102 0.87854 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 
// PoseReconstruction/right_hand_up.obj
// PoseReconstruction/right_hand_up_euler.obj
//TEST_CASE("Right Hand Up")
//{
//	// Read mesh to body
//	// Regress to 3D smpl/coco joints
//	// Run optimizer from 3D
//	// Compare reconstructed and original mesh
//
//	std::vector<smpl::float3> vertices;
//	std::vector<smpl::uint> indices;
//	std::vector<smpl::Skin> skins;
//
//	smpl::ReadObjFile("PoseReconstruction/right_hand_up_euler.obj", vertices, indices, skins);
//	smpl::Body body(vertices, indices);
//
//	smpl::SparseMatrix coco_matrix;
//	smpl::SparseMatrix smpl_matrix;
//	ReadSparseMatrixFile("Model/coco_regressor.txt", coco_matrix);
//	ReadSparseMatrixFile("Model/smpl_regressor.txt", smpl_matrix);
//	smpl::JointRegressor coco_regressor(coco_matrix, smpl::COCO_JOINT_COUNT);
//	smpl::JointRegressor smpl_regressor(smpl_matrix, smpl::JOINT_COUNT);
//
//	smpl::Joints coco_joints = coco_regressor(body.vertices);
//	smpl::Joints smpl_joints = smpl_regressor(body.vertices);
//
//	std::vector<float> tracked_joints;
//	tracked_joints.reserve(smpl::JOINT_COUNT * 3);
//
//	for (UINT i = 0; i < smpl::JOINT_COUNT; i++)
//	{
//		tracked_joints.push_back(smpl_joints.col(i)(0));
//		tracked_joints.push_back(smpl_joints.col(i)(1));
//		tracked_joints.push_back(smpl_joints.col(i)(2));
//	}
//
//	smpl::Optimizer optimize(smpl::Optimizer::Configuration(std::string("Model")),
//		smpl::Generator(smpl::Generator::Configuration(std::string("Model"))), tracked_joints);
//
//	smpl::ShapeCoefficients shape;
//	smpl::PoseEulerCoefficients pose;
//	ZeroMemory(&shape, sizeof(shape));
//	ZeroMemory(&pose, sizeof(pose));
//	optimize.OptimizePoseFrom3D(smpl::Optimizer::JOINT_TYPE::SMPL, shape, pose);
//
//	smpl::Generator generator(smpl::Generator::Configuration(std::string("Model")));
//
//	smpl::Body reconstructed_body = generator(shape, pose);
//	reconstructed_body.Dump("PoseReconstruction/right_hand_up_reconstructed.obj");
//
//	REQUIRE(true);
//}