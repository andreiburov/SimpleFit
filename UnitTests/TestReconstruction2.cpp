#include <catch.hpp>
#include <SMPL.h>
#include <Utils.h>
#include <vector>
#include <fstream>

using namespace smpl;

namespace reconstruction
{
	//ShapeCoefficients shape;
	//PoseEulerCoefficients pose;
	//Generator generator(smpl::Generator::Configuration(std::string("../Model")));
	//SparseMatrix smpl_matrix, coco_matrix;
	//Eigen::Vector3f	translation(0.f, 0.2f, -4.f);

	//void LogBodyAndProjectionWithCorrespondances(const Body& body, const RegressedJoints& joints,
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

	//void TestReconstruction2D(
	//	const std::string& image_filename,
	//	const std::vector<float>& tracked_joints)
	//{
	//	ZeroMemory(&shape, sizeof(shape));
	//	ZeroMemory(&pose, sizeof(pose));

	//	Optimizer::Configuration configuration("../Model");
	//	JointsRegressor coco_regressor(configuration.coco_regressor, smpl::COCO_JOINT_COUNT);
	//	Projector project(configuration.intrinsics);
	//	smpl::Generator generator(smpl::Generator::Configuration(std::string("../Model")));
	//	smpl::Optimizer optimize(configuration, generator, tracked_joints);

	//	/*for (uint j = 0; j < BETA_COUNT; j++)
	//	{
	//		shape[j] = (float)rand() / (RAND_MAX) / 2 + 0.5f;
	//	}*/
	//	/*for (uint k = 0; k < THETA_COUNT * 3; k++)
	//	{
	//		pose(k) = (float)rand() / (RAND_MAX) / 100;
	//	}*/

	//	uint count = 0;

	//	std::cout << "RECONSTRUCT COARSE\n";
	//	optimize.ReconstructCoarse(image_filename, translation, shape, pose, count);

	//	PoseEulerCoefficients pose2;
	//	for (int i = 0; i < THETA_COUNT; i++)
	//		pose2[i] = pose[i];

	//	std::cout << "RECONSTRUCT DIRECT COARSE\n";
	//	optimize.ReconstructCoarse(image_filename, translation, shape, pose2, count);

	//	pose(0) *= -1;
	//	pose(1) *= -1;
	//	pose(2) *= -1;

	//	std::cout << "RECONSTRUCT INVERSE COARSE\n";
	//	optimize.ReconstructCoarse(image_filename, translation, shape, pose, count);
	//	std::cout << "RECONSTRUCT FINE\n";
	//	optimize.ReconstructFine(image_filename, translation, shape, pose, count);

	//	Body body = generator(shape, pose);
	//	RegressedJoints joints = coco_regressor(body.vertices);
	//	LogBodyAndProjectionWithCorrespondances(body, joints, tracked_joints, project, "Body2.obj", "Body2.png");
	//}

	//void TestTotalReconstruction2D(
	//	const std::string& image_filename,
	//	const std::vector<float>& tracked_joints)
	//{
	//	ZeroMemory(&shape, sizeof(shape));
	//	ZeroMemory(&pose, sizeof(pose));

	//	Optimizer::Configuration configuration("../Model");
	//	JointsRegressor coco_regressor(configuration.coco_regressor, smpl::COCO_JOINT_COUNT);
	//	Projector project(configuration.intrinsics);
	//	smpl::Generator generator(smpl::Generator::Configuration(std::string("../Model")));
	//	smpl::Optimizer optimize(configuration, generator, tracked_joints);

	//	uint count = 0;

	//	translation(0) = -0.187017f;
	//	translation(1) = 0.409917f;
	//	translation(2) = -3.84496f;
	//	pose(0) = -0.0585405f;
	//	pose(1) = -0.496275f;
	//	pose(2) = 0.0685816f;

	//	optimize.ReconstructTotal(image_filename, translation, shape, pose, count);

	//	Body body = generator(shape, pose);
	//	RegressedJoints joints = coco_regressor(body.vertices);
	//	LogBodyAndProjectionWithCorrespondances(body, joints, tracked_joints, project, "Body2.obj", "Body2.png");
	//}

	//void TestCameraReconstruction2D(
	//	const std::string& image_filename,
	//	const std::vector<float>& tracked_joints)
	//{
	//	ZeroMemory(&shape, sizeof(shape));
	//	ZeroMemory(&pose, sizeof(pose));

	//	Optimizer::Configuration configuration("../Model");
	//	JointsRegressor coco_regressor(configuration.coco_regressor, smpl::COCO_JOINT_COUNT);
	//	Projector project(configuration.intrinsics);
	//	smpl::Generator generator(smpl::Generator::Configuration(std::string("../Model")));
	//	smpl::Optimizer optimize(configuration, generator, tracked_joints);

	//	uint count = 0;

	//	optimize.ReconstructCamera(image_filename, translation, shape, pose, count);
	//}

	//void TestCombinedReconstruction2D(
	//	const std::string& image_filename,
	//	const std::vector<float>& tracked_joints)
	//{
	//	ZeroMemory(&shape, sizeof(shape));
	//	ZeroMemory(&pose, sizeof(pose));

	//	Optimizer::Configuration configuration("../Model");
	//	JointsRegressor coco_regressor(configuration.coco_regressor, smpl::COCO_JOINT_COUNT);
	//	Projector project(configuration.intrinsics);
	//	smpl::Generator generator(smpl::Generator::Configuration(std::string("../Model")));
	//	smpl::Optimizer optimize(configuration, generator, tracked_joints);

	//	uint count = 0;
	//	optimize.Reconstruct(image_filename, translation, shape, pose, count);
	//}

	//void TestCombinedSequenceReconstruction2D(
	//	const std::string& image_filename,
	//	const std::vector<float>& tracked_joints, smpl::ShapeCoefficients& shape, smpl::PoseEulerCoefficients& pose)
	//{
	//	
	//}

	//TEST_CASE("LM From File")
	//{
	//	std::ifstream in("Frames0_joints/296_keypoints.json.2.txt");

	//	if (in.fail())
	//	{
	//		MessageBoxA(NULL, "Frames0_joints/296_keypoints.json.2.txt does not exist!", "Error", MB_OK);
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

	//	//TestReconstruction2D("Frames0/296.png", tracked_joints);
	//	TestCombinedReconstruction2D("Frames0/296.png", tracked_joints);
	//}

	//TEST_CASE("LM Sequence")
	//{
	//	smpl::ShapeCoefficients shape;
	//	smpl::PoseEulerCoefficients pose;
	//	ZeroMemory(&shape, sizeof(shape));
	//	ZeroMemory(&pose, sizeof(pose));

	//	Optimizer::Configuration configuration("../Model");
	//	JointsRegressor coco_regressor(configuration.coco_regressor, smpl::COCO_JOINT_COUNT);
	//	Projector project(configuration.intrinsics);
	//	smpl::Generator generator(smpl::Generator::Configuration(std::string("../Model")));

	//	for (uint i = 1; i < 401; i+=4)
	//	{
	//		std::string joints_filename(std::string("Frames0_joints/").append(std::to_string(i)).append("_keypoints.json.2.txt"));
	//		std::ifstream in(joints_filename);

	//		if (in.fail())
	//		{
	//			MessageBoxA(NULL, joints_filename.c_str(), "Error", MB_OK);
	//		}

	//		std::string line;
	//		std::getline(in, line);
	//		std::istringstream iss(line);
	//		std::vector<float> tracked_joints;
	//		tracked_joints.reserve(36);
	//		float tracked;

	//		while (iss >> tracked)
	//		{
	//			tracked_joints.push_back(tracked);
	//		}

	//		smpl::Optimizer optimize(configuration, generator, tracked_joints);
	//		std::string image_filename(std::string("Frames0/").append(std::to_string(i)).append(".png"));
	//		optimize.Reconstruct(image_filename, translation, shape, pose, i);
	//	}
	//}
}