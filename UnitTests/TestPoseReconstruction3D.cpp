#include <catch.hpp>
#include <SMPL.h>
#include <Utils.h>
#include <vector>
#include <fstream>

using namespace smpl;

namespace pose_reconstruction_3d
{
	//float delta = 0.5f;

	//ShapeCoefficients shape;
	//PoseEulerCoefficients pose;
	//Generator generator(smpl::Generator::Configuration(std::string("../Model")));
	//SparseMatrix smpl_matrix;

	//void TestPoseReconstruction3D(PoseEulerCoefficients& pose)
	//{
	//	ZeroMemory(&shape, sizeof(shape));

	//	Body body2 = generator(shape, pose);
	//	body2.Dump("Body2.obj");

	//	ReadSparseMatrixFile("Model/smpl_regressor.txt", smpl_matrix);
	//	JointRegressor smpl_regressor(smpl_matrix, smpl::JOINT_COUNT);
	//	Joints smpl_joints = smpl_regressor(body2.vertices);

	//	std::vector<float> tracked_joints;
	//	tracked_joints.reserve(smpl::JOINT_COUNT * 3);
	//	for (UINT i = 0; i < smpl::JOINT_COUNT; i++)
	//	{
	//		tracked_joints.push_back(smpl_joints.col(i)(0));
	//		tracked_joints.push_back(smpl_joints.col(i)(1));
	//		tracked_joints.push_back(smpl_joints.col(i)(2));
	//	}

	//	smpl::Optimizer optimize(smpl::Optimizer::Configuration(std::string("../Model")),
	//		generator, tracked_joints);

	//	ZeroMemory(&pose, sizeof(pose));
	//	optimize.OptimizePoseFromSmplJoints3D(shape, pose);

	//	Body body1 = generator(shape, pose);
	//	body1.Dump("Body1.obj");
	//}

	//TEST_CASE("PR3D 16th in Gamma")
	//{
	//	ZeroMemory(&pose, sizeof(pose));
	//	pose[16] = float3(0, 0, delta);

	//	TestPoseReconstruction3D(pose);
	//}

	//TEST_CASE("PR3D 16th")
	//{
	//	ZeroMemory(&pose, sizeof(pose));
	//	pose[16] = float3(-delta, -delta, delta);

	//	TestPoseReconstruction3D(pose);
	//}

	//TEST_CASE("PR3D 18th")
	//{
	//	ZeroMemory(&pose, sizeof(pose));
	//	pose[18] = float3(0.0898455f, -1.41424f, 0.540512f);

	//	TestPoseReconstruction3D(pose);
	//}

	//TEST_CASE("PR3D 16th and 18th")
	//{
	//	ZeroMemory(&pose, sizeof(pose));

	//	//0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 - 0.824606 - 0.321571 0.432364 0 0 0 0.0898455 - 1.41424 0.540512 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
	//	//pose[16] = float3(-0.824606f, -0.321571f, 0.432364f);
	//	pose[18] = float3(0.0898455f, -1.41424f, 0.540512f);
	//	// 0 0 0 -0.874163 0.817765 0.0412538 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -0.673621 -0.970013 0.375232 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 
	//	//pose[16] = float3(-0.673621f, -0.970013f, 0.375232f);
	//	//pose[1] = float3(-0.874163f, 0.817765f, 0.0412538f);

	//	TestPoseReconstruction3D(pose);
	//}

	//TEST_CASE("PR3D Arbitrary")
	//{
	//	ZeroMemory(&pose, sizeof(pose));
	//	pose << std::string("0 0 0 -0.824325 0.441603 0.198994 0 0 0 0 0 0 0.824884 0.372528 -0.100031 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1.2863 1.51537 0.506102 0 0 0 0 0 0 0 0 0 0 0 0");

	//	TestPoseReconstruction3D(pose);
	//}

	//TEST_CASE("PR3D From File")
	//{
	//	ZeroMemory(&pose, sizeof(pose));
	//	std::ifstream in("thetas.txt");

	//	if (in.fail())
	//	{
	//		std::cerr << "The file thetas.txt does not exist!\n";
	//	}

	//	std::string str;
	//	std::getline(in, str);
	//	pose << str;
	//	//pose << std::string("0 0 0 -0.83272 0.0960831 0.222415 0 0 0 0 0 0 0 0 0 0.839531 -0.151116 -0.0245641 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0.479258 -2.88858e-17 0.0887515 0 0 0 -0.168738 -0.176219 0.636729 -1.09971 -1.4996 -0.10923 0.389453 1.22399 0.0947874 0 0 0 0 0 0 0 0 0 0 0 0");
	//	TestPoseReconstruction3D(pose);
	//}
}