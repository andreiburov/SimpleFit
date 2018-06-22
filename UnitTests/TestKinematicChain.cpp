#include <catch.hpp>
#include <EulerAngles.h>
#include <SMPL.h>

namespace kinematic_chain
{
	float alpha_a = 0.7f;
	float beta_a = -0.3f;
	float gamma_a = 0.125f;
	float alpha_b = -0.5f;
	float beta_b = 0.3f;
	float gamma_b = 0.4f;

	float delta = 0.1f;

	float delta_alpha = -0.1f;
	float delta_beta = 0.2f;
	float delta_gamma = 0.1f;
	
	Eigen::Vector3f j_a(0.1f, 2.3f, -0.2f);
	Eigen::Vector3f j_b(2.3f, -1.2f, 0.9f);

	TEST_CASE("2 Links Change in Alpha")
	{
		Eigen::Matrix4f A1 = EulerSkinning(alpha_a, beta_a, gamma_a, j_a(0), j_a(1), j_a(2));
		Eigen::Matrix4f B1 = EulerSkinning(alpha_b, beta_b, gamma_b, j_b(0), j_b(1), j_b(2));
		Eigen::Matrix4f B2 = EulerSkinning(alpha_b + delta, beta_b, gamma_b, j_b(0), j_b(1), j_b(2));

		Eigen::Matrix4f S = A1 * B1;
		Eigen::Matrix4f T = A1 * B2;

		Eigen::Matrix4f dB1 = EulerSkinningDerivativeToAlpha(alpha_b, beta_b, gamma_b, j_b(0), j_b(1), j_b(2));
		Eigen::Matrix4f dS = A1 * dB1;

		REQUIRE(T.isApprox(S + delta * dS, delta));
	}

	smpl::ShapeCoefficients shape;
	smpl::PoseEulerCoefficients pose;
	smpl::Generator generator(smpl::Generator::Configuration(std::string("../Model")));
	smpl::Optimizer optimizer(smpl::Optimizer::Configuration(std::string("../Model")),
		generator, std::vector<float>());

	Eigen::Matrix4f palette[smpl::JOINT_COUNT];
	Eigen::Matrix4f dskinning[smpl::JOINT_COUNT * 3];
	Eigen::Matrix4f palette2[smpl::JOINT_COUNT];

	void CheckBodies(smpl::Body& body1)
	{
		const std::vector<smpl::Skin>& skins = generator.GetSkins();

		smpl::Body body2 = body1;

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

		//smpl::Body body2 = generator(shape, pose);

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

	TEST_CASE("SMPL Chain Change in 16th Gamma")
	{
		UINT change_id = 16;

		ZeroMemory(&shape, sizeof(shape));
		ZeroMemory(&pose, sizeof(pose));
		
		smpl::Body body1 = generator(shape, pose);		
		smpl::Joints joints = optimizer.RegressJoints(body1, smpl::Optimizer::JOINT_TYPE::SMPL);
		optimizer.ComputeSkinningDerivatives(pose, joints, palette, dskinning);

		// Update the palette in 16th Gamma and propagate the change
		palette[change_id] += delta_alpha * dskinning[change_id * 3 + 0]
			+ delta_beta * dskinning[change_id * 3 + 1]
			+ delta_gamma * dskinning[change_id * 3 + 2];

		for (UINT i = change_id + 1; i < smpl::JOINT_COUNT; i++)
		{
			palette[i] = palette[smpl::PARENT_INDEX[i]] * 
				EulerSkinning(pose[i].x, pose[i].y, pose[i].z, joints.col(i)(0), joints.col(i)(1), joints.col(i)(2));
		}

		pose[change_id] = smpl::float3(delta_alpha, delta_beta, delta_gamma);

		// Initialize palette2
		optimizer.ComputeSkinningDerivatives(pose, optimizer.RegressJoints(body1, smpl::Optimizer::JOINT_TYPE::SMPL), palette2, dskinning);

		SECTION("Check palettes")
		{
			REQUIRE(palette[change_id].isApprox(palette2[change_id], delta));
		}

		SECTION("Check bodies")
		{
			CheckBodies(body1);
		}
	}

	TEST_CASE("SMPL Chain Change in ith")
	{
		UINT change_id = 16;

		ZeroMemory(&shape, sizeof(shape));
		ZeroMemory(&pose, sizeof(pose));

		smpl::Body body1 = generator(shape, pose);
		smpl::Joints joints = optimizer.RegressJoints(body1, smpl::Optimizer::JOINT_TYPE::SMPL);
		optimizer.ComputeSkinningDerivatives(pose, joints, palette, dskinning);

		// Update the palette in 16th Gamma and propagate the change
		palette[change_id] += delta * dskinning[change_id * 3 + 2];
		for (UINT i = change_id + 1; i < smpl::JOINT_COUNT; i++)
		{
			palette[i] = palette[smpl::PARENT_INDEX[i]] *
				EulerSkinning(pose[i].x, pose[i].y, pose[i].z, joints.col(i)(0), joints.col(i)(1), joints.col(i)(2));
		}

		pose[change_id] = smpl::float3(0, 0, delta);

		// Initialize palette2
		optimizer.ComputeSkinningDerivatives(pose, optimizer.RegressJoints(body1, smpl::Optimizer::JOINT_TYPE::SMPL), palette2, dskinning);

		SECTION("Check palettes")
		{
			REQUIRE(palette[change_id].isApprox(palette2[change_id], delta));
		}

		SECTION("Check bodies")
		{
			CheckBodies(body1);
		}
	}
}