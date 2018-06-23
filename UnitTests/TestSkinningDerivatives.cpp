#include <catch.hpp>
#include <EulerAngles.h>
#include <SMPL.h>

using namespace smpl;

namespace skinning_derivatives
{
	float delta = 0.15f;

	ShapeCoefficients shape;
	PoseEulerCoefficients pose;
	Generator generator(smpl::Generator::Configuration(std::string("../Model")));
	Optimizer optimizer(smpl::Optimizer::Configuration(std::string("../Model")),
		generator, std::vector<float>());

	Eigen::Matrix4f palette[smpl::JOINT_COUNT];
	Eigen::Matrix4f* dskinning;
	Eigen::Matrix4f palette2[smpl::JOINT_COUNT];

	void CheckBodies(smpl::Body& body1)
	{
		const std::vector<smpl::Skin>& skins = generator.GetSkins();

		Body body2 = body1;

#pragma omp parallel for
		for (UINT i = 0; i < smpl::VERTEX_COUNT; i++)
		{
			Eigen::Matrix4f skin
				= skins[i].weight.x * palette[skins[i].joint_index.x]
				+ skins[i].weight.y * palette[skins[i].joint_index.y]
				+ skins[i].weight.z * palette[skins[i].joint_index.z]
				+ skins[i].weight.w * palette[skins[i].joint_index.w];


			body1.vertices[i] = smpl::float3((skin * body1.vertices[i].ToEigen().homogeneous()).head(3));
		}

#pragma omp parallel for
		for (UINT i = 0; i < smpl::VERTEX_COUNT; i++)
		{
			Eigen::Matrix4f skin
				= skins[i].weight.x * palette2[skins[i].joint_index.x]
				+ skins[i].weight.y * palette2[skins[i].joint_index.y]
				+ skins[i].weight.z * palette2[skins[i].joint_index.z]
				+ skins[i].weight.w * palette2[skins[i].joint_index.w];


			body2.vertices[i] = smpl::float3((skin * body2.vertices[i].ToEigen().homogeneous()).head(3));
		}

		body1.Dump("Body1.obj");
		body2.Dump("Body2.obj");

		for (UINT i = 0; i < smpl::VERTEX_COUNT; i++)
		{
			INFO("i=" << i);
			CHECK(body1.vertices[i].x == Approx(body2.vertices[i].x).epsilon(delta));
			INFO("i=" << i);
			CHECK(body1.vertices[i].y == Approx(body2.vertices[i].y).epsilon(delta));
			INFO("i=" << i);
			CHECK(body1.vertices[i].z == Approx(body2.vertices[i].z).epsilon(delta));
		}
	}

	
	TEST_CASE("SD 16th Gamma")
	{
		std::cout << "SD 16th in Gamma" << std::endl;
		UINT change = 16;

		ZeroMemory(&shape, sizeof(shape));
		ZeroMemory(&pose, sizeof(pose));

		Body body1 = generator(shape, pose);
		Joints joints = optimizer.RegressJoints(body1, Optimizer::JOINT_TYPE::SMPL);

		dskinning = new Eigen::Matrix4f[JOINT_COUNT * JOINT_COUNT * 3];
		optimizer.ComputeSkinningDerivatives(pose, joints, palette, dskinning);

		// Update the palette
		for (UINT j = 0; j < JOINT_COUNT; j++)
		{
			palette[j] += delta * dskinning[GAMMA(change) * JOINT_COUNT + j];
		}

		pose[change] = float3(0, 0, delta);

		// Initialize palette2
		optimizer.ComputeSkinning(pose, optimizer.RegressJoints(body1, Optimizer::JOINT_TYPE::SMPL), palette2);

		SECTION("Check palettes")
		{
			REQUIRE(palette[change].isApprox(palette2[change], delta));
		}

		SECTION("Check bodies")
		{
			CheckBodies(body1);
		}

		delete[] dskinning;
	}

	TEST_CASE("SD 16th")
	{
		std::cout << "SD 16th" << std::endl;
		UINT change = 16;

		ZeroMemory(&shape, sizeof(shape));
		ZeroMemory(&pose, sizeof(pose));

		Body body1 = generator(shape, pose);
		Joints joints = optimizer.RegressJoints(body1, Optimizer::JOINT_TYPE::SMPL);

		dskinning = new Eigen::Matrix4f[JOINT_COUNT * JOINT_COUNT * 3];
		optimizer.ComputeSkinningDerivatives(pose, joints, palette, dskinning);

		// Update the palette
		for (UINT j = 0; j < JOINT_COUNT; j++)
		{
			palette[j] += 
				delta * dskinning[ALPHA(change) * JOINT_COUNT + j] +
				delta * dskinning[BETA(change) * JOINT_COUNT + j] +
				delta * dskinning[GAMMA(change) * JOINT_COUNT + j];
		}

		pose[change] = float3(delta, delta, delta);

		// Initialize palette2
		optimizer.ComputeSkinning(pose, optimizer.RegressJoints(body1, Optimizer::JOINT_TYPE::SMPL), palette2);

		SECTION("Check palettes")
		{
			REQUIRE(palette[change].isApprox(palette2[change], delta));
		}

		SECTION("Check bodies")
		{
			CheckBodies(body1);
		}

		delete[] dskinning;
	}

	TEST_CASE("SD 16th and 18th")
	{
		std::cout << "SD 16th and 18th" << std::endl;
		UINT change = 16;
		UINT change2 = 18;

		ZeroMemory(&shape, sizeof(shape));
		ZeroMemory(&pose, sizeof(pose));

		smpl::Body body1 = generator(shape, pose);
		smpl::Joints joints = optimizer.RegressJoints(body1, smpl::Optimizer::JOINT_TYPE::SMPL);

		dskinning = new Eigen::Matrix4f[JOINT_COUNT * JOINT_COUNT * 3];
		optimizer.ComputeSkinningDerivatives(pose, joints, palette, dskinning);

		// Update the palette
		for (UINT j = 0; j < JOINT_COUNT; j++)
		{
			palette[j] +=
				delta * dskinning[ALPHA(change) * JOINT_COUNT + j] +
				delta * dskinning[BETA(change) * JOINT_COUNT + j] +
				delta * dskinning[GAMMA(change) * JOINT_COUNT + j];

			palette[j] +=
				delta * dskinning[ALPHA(change2) * JOINT_COUNT + j] +
				delta * dskinning[BETA(change2) * JOINT_COUNT + j] +
				delta * dskinning[GAMMA(change2) * JOINT_COUNT + j];
		}

		pose[change] = float3(delta, delta, delta);
		pose[change2] = float3(delta, delta, delta);

		// Initialize palette2
		optimizer.ComputeSkinning(pose, optimizer.RegressJoints(body1, Optimizer::JOINT_TYPE::SMPL), palette2);	
		
		SECTION("Check palettes")
		{
			REQUIRE(palette[change].isApprox(palette2[change], delta));
			REQUIRE(palette[change2].isApprox(palette2[change2], delta));
		}

		SECTION("Check bodies")
		{
			CheckBodies(body1);
		}
	}

	TEST_CASE("SD 13th and 16th and 18th")
	{
		std::cout << "SD 13th and 16th and 18th" << std::endl;

		UINT change = 13;
		UINT change2 = 16;
		UINT change3 = 18;

		ZeroMemory(&shape, sizeof(shape));
		ZeroMemory(&pose, sizeof(pose));

		smpl::Body body1 = generator(shape, pose);
		smpl::Joints joints = optimizer.RegressJoints(body1, smpl::Optimizer::JOINT_TYPE::SMPL);

		dskinning = new Eigen::Matrix4f[JOINT_COUNT * JOINT_COUNT * 3];
		optimizer.ComputeSkinningDerivatives(pose, joints, palette, dskinning);

		// Update the palette
		for (UINT j = 0; j < JOINT_COUNT; j++)
		{
			palette[j] +=
				delta * dskinning[ALPHA(change) * JOINT_COUNT + j] +
				delta * dskinning[BETA(change) * JOINT_COUNT + j] +
				delta * dskinning[GAMMA(change) * JOINT_COUNT + j];

			palette[j] +=
				delta * dskinning[ALPHA(change2) * JOINT_COUNT + j] +
				delta * dskinning[BETA(change2) * JOINT_COUNT + j] +
				delta * dskinning[GAMMA(change2) * JOINT_COUNT + j];

			palette[j] +=
				delta * dskinning[ALPHA(change3) * JOINT_COUNT + j] +
				delta * dskinning[BETA(change3) * JOINT_COUNT + j] +
				delta * dskinning[GAMMA(change3) * JOINT_COUNT + j];
		}

		pose[change] = float3(delta, delta, delta);
		pose[change2] = float3(delta, delta, delta);
		pose[change3] = float3(delta, delta, delta);

		// Initialize palette2
		optimizer.ComputeSkinning(pose, optimizer.RegressJoints(body1, Optimizer::JOINT_TYPE::SMPL), palette2);

		SECTION("Check palettes")
		{
			REQUIRE(palette[change].isApprox(palette2[change], delta));
			REQUIRE(palette[change2].isApprox(palette2[change2], delta));
		}

		SECTION("Check bodies")
		{
			CheckBodies(body1);
		}

		delete[] dskinning;
	}
}