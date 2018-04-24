// Runner.cpp : Defines the entry point for the console application.
//

#include <string>
#include <Windows.h>
#include "SMPL.h"

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

int main()
{
	smpl::printHello();
	
	//reconstructBodyFromImage();
	//MessageBoxW(NULL, (std::wstring(L"Could not open ")).c_str(), L"File error", MB_ICONERROR | MB_OK);

	smpl::ShapeCoefficients shape;
	smpl::PoseCoefficients pose;
	ZeroMemory(&shape, sizeof(shape));
	ZeroMemory(&pose, sizeof(pose));
	
	smpl::Body body;
	smpl::Generator generator;

	generator.run(shape, pose, body);
	body.dump(std::string("new.obj"));

    return 0;
}