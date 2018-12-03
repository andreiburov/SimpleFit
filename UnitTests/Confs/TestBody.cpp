#include <catch.hpp>
#include <SMPL.h>

using namespace smpl;

TEST_CASE("Generate Body")
{
	ShapeCoefficients betas;
	PoseEulerCoefficients thetas;

	thetas[HIP_RIGHT].z = 0.75f;
	thetas[HIP_LEFT].z = -0.75f;

	Generator generator(Generator::Configuration("../Model/"));
	Body body = generator(betas, thetas);

	body.Dump("body.obj");
}