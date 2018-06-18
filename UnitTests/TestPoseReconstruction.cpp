#include <catch.hpp>
#include <SMPL.h>
#include <Utils.h>
#include <vector>

TEST_CASE("Right Hand Up - Dumb")
{
	smpl::ShapeCoefficients shape;
	smpl::PoseEulerCoefficients pose;
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

	smpl::Optimizer optimize(smpl::Optimizer::Configuration(std::string("Model")),
		smpl::Generator(smpl::Generator::Configuration(std::string("Model"))), tracked_joints);

	optimize.OptimizePoseFrom3D(smpl::Optimizer::JOINT_TYPE::SMPL, shape, pose);

	for (UINT i = 0; i < smpl::JOINT_COUNT * 3; i++)
	{
		REQUIRE(abs(local_pose[i] - pose(i)) < 0.01);
	}
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