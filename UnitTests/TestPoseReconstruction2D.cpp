#include <catch.hpp>
#include <SMPL.h>
#include <Utils.h>
#include <vector>
#include <fstream>

using namespace smpl;

namespace pose_reconstruction_2d
{
	//float delta = 0.5f;

	//ShapeCoefficients shape;
	//PoseEulerCoefficients pose;
	//Generator generator(smpl::Generator::Configuration(std::string("../Model")));
	//SparseMatrix smpl_matrix, coco_matrix;
	//Eigen::Vector3f	translation(0.f, 0.2f, -4.f);

	//void LogBodyAndProjection(const Body& body, const Joints& joints, const Projector& project,
	//	const std::string& body_filename, const std::string& projection_filename)
	//{
	//	body.Dump(body_filename);
	//	Image image;
	//	Image::Draw3D(image, project.GetIntrinsics(), translation, WHITE, body.vertices);
	//	Image::Draw3D(image, project.GetIntrinsics(), translation, BLUE, 2, Joints2Vector(joints));
	//	image.SavePNG(projection_filename);
	//}

	//void LogBodyAndProjectionWithCorrespondances(const Body& body, const Joints& joints, 
	//	const std::vector<float>& correspondances, const Projector& project,
	//	const std::string& body_filename, const std::string& projection_filename)
	//{
	//	body.Dump(body_filename);
	//	Image image;
	//	Image::Draw3D(image, project.GetIntrinsics(), translation, WHITE, body.vertices);
	//	Image::Draw3D(image, project.GetIntrinsics(), translation, BLUE, 2, Joints2Vector(joints));
	//	Image::Draw2D(image, YELLOW, 2, correspondances);
	//	image.SavePNG(projection_filename);
	//}

	//void TestSyntheticPoseReconstruction2D(
	//	const smpl::Optimizer::JOINT_TYPE& joint_type, PoseEulerCoefficients& pose)
	//{
	//	ZeroMemory(&shape, sizeof(shape));

	//	Optimizer::Configuration configuration("../Model");
	//	JointRegressor smpl_regressor(configuration.smpl_regressor, smpl::JOINT_COUNT);
	//	JointRegressor coco_regressor(configuration.coco_regressor, smpl::COCO_JOINT_COUNT);
	//	Projector project(configuration.intrinsics);

	//	Body body2 = generator(shape, pose);
	//	Joints joints2 = (joint_type == smpl::Optimizer::JOINT_TYPE::COCO ?
	//		coco_regressor(body2.vertices) : smpl_regressor(body2.vertices));
	//	LogBodyAndProjection(body2, joints2, project, "Body2.obj", "Body2.png");

	//	std::vector<float> tracked_joints;
	//	tracked_joints.reserve(smpl::JOINT_COUNT * 2);
	//	for (UINT i = 0; i < smpl::JOINT_COUNT; i++)
	//	{
	//		auto j = project(joints2.col(i), translation);
	//		tracked_joints.push_back(j(0));
	//		tracked_joints.push_back(j(1));
	//	}

	//	smpl::Optimizer optimize(configuration,	generator, tracked_joints);

	//	ZeroMemory(&pose, sizeof(pose));
	//	optimize.OptimizePoseFromJoints2D(joint_type, "black.png", translation, shape, pose);

	//	Body body1 = generator(shape, pose);
	//	Joints joints1 = (joint_type == smpl::Optimizer::JOINT_TYPE::COCO ?
	//		coco_regressor(body1.vertices) : smpl_regressor(body1.vertices));
	//	LogBodyAndProjection(body1, joints1, project, "Body1.obj", "Body1.png");
	//}

	//void TestRealPoseReconstruction2D(
	//	const std::string& image_filename,
	//	const std::vector<float>& tracked_joints,
	//	const Eigen::Vector3f& translation)
	//{
	//	ZeroMemory(&shape, sizeof(shape));
	//	ZeroMemory(&pose, sizeof(pose));

	//	Optimizer::Configuration configuration("../Model");
	//	JointRegressor coco_regressor(configuration.coco_regressor, smpl::COCO_JOINT_COUNT);
	//	Projector project(configuration.intrinsics);
	//	smpl::Generator generator(smpl::Generator::Configuration(std::string("../Model")));
	//	smpl::Optimizer optimize(configuration,	generator, tracked_joints);

	//	optimize.OptimizePoseFromJoints2D(smpl::Optimizer::JOINT_TYPE::COCO, 
	//		image_filename, translation, shape, pose);
	//	
	//	Body body = generator(shape, pose);
	//	Joints joints = coco_regressor(body.vertices);
	//	LogBodyAndProjectionWithCorrespondances(body, joints, tracked_joints, project, "Body2.obj", "Body2.png");
	//}

	//TEST_CASE("SPR SMPL2D From File")
	//{
	//	ZeroMemory(&pose, sizeof(pose));
	//	std::ifstream in("thetas.txt");

	//	if (in.fail())
	//	{
	//		MessageBoxA(NULL, "The file thetas.txt does not exist!", "Error", MB_OK);
	//	}

	//	std::string str;
	//	std::getline(in, str);
	//	pose << str;
	//	//pose << std::string("0 0 0 -0.83272 0.0960831 0.222415 0 0 0 0 0 0 0 0 0 0.839531 -0.151116 -0.0245641 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0.479258 -2.88858e-17 0.0887515 0 0 0 -0.168738 -0.176219 0.636729 -1.09971 -1.4996 -0.10923 0.389453 1.22399 0.0947874 0 0 0 0 0 0 0 0 0 0 0 0");
	//	TestSyntheticPoseReconstruction2D(smpl::Optimizer::JOINT_TYPE::SMPL, pose);
	//}

	//TEST_CASE("SPR COCO2D From File")
	//{
	//	ZeroMemory(&pose, sizeof(pose));
	//	std::ifstream in("thetas.txt");

	//	if (in.fail())
	//	{
	//		MessageBoxA(NULL, "The file thetas.txt does not exist!", "Error", MB_OK);
	//	}

	//	std::string str;
	//	std::getline(in, str);
	//	pose << str;
	//	TestSyntheticPoseReconstruction2D(smpl::Optimizer::JOINT_TYPE::COCO, pose);
	//}

	//TEST_CASE("RPR From File")
	//{
	//	std::ifstream in("Frames0_joints/1_keypoints.json.2.txt");

	//	if (in.fail())
	//	{
	//		MessageBoxA(NULL, "Frames0_joints/1_keypoints.json.2.txt does not exist!", "Error", MB_OK);
	//	}

	//	std::string line;
	//	std::getline(in, line);
	//	std::istringstream iss(line);
	//	std::vector<float> tracked_joints;
	//	tracked_joints.reserve(36);
	//	float tracked;

	//	while (iss >> tracked)
	//	{
	//		tracked_joints.push_back(tracked);
	//	}

	//	Eigen::Vector3f translation(-0.0401437f, 0.342779f, -3.93008f);
	//	TestRealPoseReconstruction2D("Frames0/1.png", tracked_joints, translation);
	//}
}