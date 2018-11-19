#include <catch.hpp>
#include <SMPL.h>
#include <Utils.h>
#include <vector>
#include <fstream>

using namespace smpl;

namespace extrinsics
{
	/*ShapeCoefficients shape;
	PoseEulerCoefficients pose;
	Generator generator(smpl::Generator::Configuration(std::string("../Model")));
	Eigen::Vector3f	translation(0, 0, -4.f);

	void LogBodyAndProjection(const Body& body, const RegressedJoints& joints, const Projector& project,
		const std::string& body_filename, const std::string& projection_filename)
	{
		body.Dump(body_filename);
		Image image;
		Image::Draw3D(image, project.GetIntrinsics(), translation, WHITE, body.vertices);
		Image::Draw3D(image, project.GetIntrinsics(), translation, BLUE, 2, Joints2Vector(joints));
		image.SavePNG(projection_filename);
	}

	void TestExtrinsics(const std::string& image_filename,
		const std::vector<float>& tracked_joints)
	{
		ZeroMemory(&shape, sizeof(shape));
		ZeroMemory(&pose, sizeof(pose));

		smpl::Generator generator(smpl::Generator::Configuration(std::string("../Model")));
		smpl::Optimizer optimize(smpl::Optimizer::Configuration(std::string("../Model")),
			generator, tracked_joints);

		Body body = generator(shape, pose);
		optimize.OptimizeExtrinsics(image_filename, body, translation);
	}

	TEST_CASE("TE From File")
	{
		std::ifstream in("Frames0_joints/1_keypoints.json.2.txt");

		if (in.fail())
		{
			MessageBoxA(NULL, "Frames0_joints/1_keypoints.json.2.txt does not exist!", "Error", MB_OK);
		}

		std::string line;
		std::getline(in, line);
		std::istringstream iss(line);
		std::vector<float> tracked_joints;
		tracked_joints.reserve(36);
		float tracked;
		
		while (iss >> tracked)
		{
			tracked_joints.push_back(tracked);
		}

		TestExtrinsics("Frames0/1.png", tracked_joints);
	}*/
}