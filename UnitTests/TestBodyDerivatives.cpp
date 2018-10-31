#include <catch.hpp>
#include <EulerAngles.h>
#include <SMPL.h>

using namespace smpl;

TEST_CASE("Jacobian Body From Shape")
{
	Generator generator(Generator::Configuration(std::string("../Model")));

	ShapeCoefficients input_betas;
	PoseEulerCoefficients input_thetas;
	input_betas[0] = -1.f;
	input_betas[1] = -5.0f;
	Body input_body = generator(input_betas, input_thetas);

	ShapeCoefficients model_betas;
	PoseEulerCoefficients model_thetas;
	Body model_body = generator(model_betas, model_thetas);

	std::vector<float3> dshape(VERTEX_COUNT * BETA_COUNT);
	generator.ComputeBodyFromShapeJacobian(dshape);

	for (uint i = 0; i < VERTEX_COUNT; i++)
	{
		for (uint j = 0; j < BETA_COUNT; j++)
		{
			Eigen::Vector3f v(0.f, 0.f, 0.f);
			v += input_betas[j] * dshape[i*BETA_COUNT + j].ToEigen();
			v += model_body.vertices[i].ToEigen();
			model_body.vertices[i] = float3(v);
		}
	}

	REQUIRE(model_body.IsEqual(input_body, 0.1f));
}

TEST_CASE("Jacobian Body From Pose")
{
	Generator generator(Generator::Configuration(std::string("../Model")));
	ShapeCoefficients input_betas;
	PoseEulerCoefficients input_thetas;
	input_thetas << std::string("0 0 0 2.38559 2.99926 -2.76616 0 0 0 0 0 0 \
                                 0 0 0 0       0        0       0 0 0 0 0 0 \
                                 0 0 0 0       0        0       0 0 0 0 0 0 \
                                 0 0 0 0       0        0       0 0 0 0 0 0 \
                                 0.026394 -0.961093 0.784841 0.113302 0.673124 0.452362 \
                                 0        0         0        1.783710 2.204490 -2.93395 \
                                 0        0         0        0.165202 0.104804 0.0485937 0 0 0 0 0 0");
	Body input_body = generator(input_betas, input_thetas);

	input_body.Dump("input_body.obj");

	ShapeCoefficients model_betas;
	PoseEulerCoefficients model_thetas;
	Body model_body = generator(model_betas, model_thetas);

	std::vector<float3> dpose(VERTEX_COUNT * THETA_COMPONENT_COUNT);
	generator.ComputeBodyFromPoseJacobian(model_body, dpose);

	for (uint i = 0; i < VERTEX_COUNT; i++)
	{
		for (uint k = 0; k < THETA_COMPONENT_COUNT; k++)
		{
			Eigen::Vector3f v(0.f, 0.f, 0.f);
			v += input_thetas(k) * dpose[i*THETA_COMPONENT_COUNT + k].ToEigen();
			v += model_body.vertices[i].ToEigen();
			model_body.vertices[i] = float3(v);
		}
	}

	model_body.Dump("model_body.obj");

	REQUIRE(model_body.IsEqual(input_body, 0.1f));
}