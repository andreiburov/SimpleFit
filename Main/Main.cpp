// Runner.cpp : Defines the entry point for the console application.
//

#include <string>
#include <Windows.h>
#include <fstream>

#include "SMPL.h"
#include "OpenPoseTracker.h"

void reconstructBodyFromImage(std::string image, smpl::Body body)
{
	/*
	std::vector<Joint2D> joints2D;
	openpose::regressJoints(image, joints2D);

	smpl::ShapeCoefficients shape;
	smpl::PoseCoefficients pose;
	smpl::Body body;
	smpl::Generator generator;
	smpl::Optimizer optimizer(joints2D);
	optimizer.initialize(shape, pose);

	for (int = 0; i < 1000; i++) {
		generator.run(shape, pose, body);
		optimizer.run(body, shape, pose);
	}
	*/
}

int main(int argc, char** argv)
{
	//reconstructBodyFromImage();
	//MessageBoxW(NULL, (std::wstring(L"Could not open ")).c_str(), L"File error", MB_ICONERROR | MB_OK);

	smpl::ShapeCoefficients shape;
	smpl::PoseCoefficients pose;
	ZeroMemory(&shape, sizeof(shape));
	ZeroMemory(&pose, sizeof(pose));
	smpl::float3 translation(0,0,0);
	smpl::float3 scaling(1,1,1);

	//
	///*shape[0] = 4.1f;
	//shape[1] = 2.3f;
	//shape[2] = 3.4f;
	//shape[3] = 3.1f;*/

	///*pose(1) = 0.5f;
	//pose(14) = -0.8f;
	//pose(24) = .2f;
	//pose(46) = -.5f;*/
	pose(50) = 0.5f;
	pose(52) = 0.6f;
	pose(58) = 0.5f;

	//pose[0] = smpl::float3(0.5f, -0.5f, 0.f);
	
	smpl::D3D d3d;

	//smpl::Generator generate(d3d, smpl::Generator::Configuration(std::string("Model")));
	//smpl::Body body = generate(shape, pose);
	//body.dump("new.obj");

	OpenPoseTracker joint_tracker(argc, argv);
	std::vector<float> tracked_joints = joint_tracker("Photos/1.jpg");
	smpl::Optimizer optimize(
		smpl::Generator(d3d, smpl::Generator::Configuration(std::string("Model"))), 
		smpl::JointProjector(d3d, smpl::JointProjector::Configuration("Model")), tracked_joints);
	optimize(shape, pose, scaling, translation);

    return 0;
}