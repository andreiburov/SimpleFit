#include <catch.hpp>
#include <EulerAngles.h>
#include <SMPL.h>

using namespace smpl;

namespace skinning_derivatives
{
//	float prec = 0.9f;
//	float delta = 0.15f;
//
//	ShapeCoefficients shape;
//	PoseEulerCoefficients pose, zero_pose;
//	Generator generator(smpl::Generator::Configuration(std::string("../Model")));
//	Optimizer optimizer(smpl::Optimizer::Configuration(std::string("../Model")),
//		generator, std::vector<float>());
//
//	Eigen::Matrix4f palette[smpl::JOINT_COUNT];
//	Eigen::Matrix4f* dskinning;
//	Eigen::Matrix4f palette2[smpl::JOINT_COUNT];
//
//	void CheckBodies(smpl::Body& body1)
//	{
//		const std::vector<smpl::Skin>& skins = generator.GetSkins();
//
//		Body body2 = body1;
//
//#pragma omp parallel for
//		for (UINT i = 0; i < smpl::VERTEX_COUNT; i++)
//		{
//			Eigen::Matrix4f skin
//				= skins[i].weight.x * palette[skins[i].joint_index.x]
//				+ skins[i].weight.y * palette[skins[i].joint_index.y]
//				+ skins[i].weight.z * palette[skins[i].joint_index.z]
//				+ skins[i].weight.w * palette[skins[i].joint_index.w];
//
//
//			body1.vertices[i] = smpl::float3((skin * body1.vertices[i].ToEigen().homogeneous()).head(3));
//		}
//
//#pragma omp parallel for
//		for (UINT i = 0; i < smpl::VERTEX_COUNT; i++)
//		{
//			Eigen::Matrix4f skin
//				= skins[i].weight.x * palette2[skins[i].joint_index.x]
//				+ skins[i].weight.y * palette2[skins[i].joint_index.y]
//				+ skins[i].weight.z * palette2[skins[i].joint_index.z]
//				+ skins[i].weight.w * palette2[skins[i].joint_index.w];
//
//
//			body2.vertices[i] = smpl::float3((skin * body2.vertices[i].ToEigen().homogeneous()).head(3));
//		}
//
//		body1.Dump("Body1.obj");
//		body2.Dump("Body2.obj");
//
//		for (UINT i = 0; i < smpl::VERTEX_COUNT; i++)
//		{
//			/*INFO("i=" << i);
//			CHECK(body1.vertices[i].x == Approx(body2.vertices[i].x).epsilon(delta));
//			INFO("i=" << i);
//			CHECK(body1.vertices[i].y == Approx(body2.vertices[i].y).epsilon(delta));
//			INFO("i=" << i);
//			CHECK(body1.vertices[i].z == Approx(body2.vertices[i].z).epsilon(delta));*/
//		}
//	}
//
//	void TestSkinningDerivatives(const PoseEulerCoefficients& pose, const std::vector<uint>& indexes)
//	{
//		ZeroMemory(&shape, sizeof(shape));
//		ZeroMemory(&zero_pose, sizeof(zero_pose));
//
//		Body body1 = generator(shape, zero_pose);
//		Joints joints = optimizer.RegressJoints(body1, Optimizer::JOINT_TYPE::SMPL);
//
//		dskinning = new Eigen::Matrix4f[JOINT_COUNT * JOINT_COUNT * 3];
//		optimizer.ComputeSkinningDerivatives(pose, joints, palette, dskinning);
//
//		// Update the palette
//		for (uint j = 0; j < JOINT_COUNT; j++)
//		{
//			for (uint i : indexes)
//			{
//				if (i % 3 == 2)
//					palette[j] += delta * dskinning[GAMMA(i/3) * JOINT_COUNT + j];
//				else if (i % 3 == 1)
//					palette[j] += delta * dskinning[BETA(i/3) * JOINT_COUNT + j];
//				else
//					palette[j] += delta * dskinning[ALPHA(i/3) * JOINT_COUNT + j];
//			}
//		}
//
//		// Initialize palette2
//		optimizer.ComputeSkinning(pose, optimizer.RegressJoints(body1, Optimizer::JOINT_TYPE::SMPL), palette2);
//
//		SECTION("Check palettes")
//		{
//			for (uint i : indexes)
//			{
//				INFO("i/3 = " << i / 3 << " i%3 = " << i % 3);
//				REQUIRE(palette[i/3].isApprox(palette2[i/3], prec));
//			}
//		}
//
//		SECTION("Check bodies")
//		{
//			CheckBodies(body1);
//		}
//
//		delete[] dskinning;
//	}
//
//	
//	TEST_CASE("SD 16th Gamma")
//	{	
//		ZeroMemory(&pose, sizeof(pose));
//		pose[16] = float3(0, 0, delta);
//		std::vector<uint> indexes = { GAMMA(16) };
//
//		TestSkinningDerivatives(pose, indexes);
//	}
//
//	TEST_CASE("SD 16th")
//	{
//		ZeroMemory(&pose, sizeof(pose));
//		pose[16] = float3(delta, delta, delta);
//		std::vector<uint> indexes = { ALPHA(16), BETA(16), GAMMA(16) };
//
//		TestSkinningDerivatives(pose, indexes);
//	}
//
//	TEST_CASE("SD 16th and 18th")
//	{
//		ZeroMemory(&pose, sizeof(pose));
//		pose[16] = float3(-0.389151f, -0.547207f, 0.268224f);
//		pose[18] = float3(-0.448718f, -0.65769f, 0.0789412f);
//		std::vector<uint> indexes = { ALPHA(16), BETA(16), GAMMA(16), ALPHA(18), BETA(18), GAMMA(18) };
//
//		TestSkinningDerivatives(pose, indexes);
//	}
//
//	TEST_CASE("SD 1st and 4th and 7th")
//	{
//		std::cout << "SD 13th and 16th and 18th" << std::endl;
//
//		// 0 0 0 -0.94034 0.883147 0.331726 0 0 0 0 0 0 0.991705 0.313481 -0.21214 0 0 0 0 0 0 -0.470237 0.636203 -0.140866 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 
//		ZeroMemory(&pose, sizeof(pose));
//		pose[1] = float3(-0.64034f, 0.683147f, 0.331726f);
//		pose[4] = float3(0.691705f, 0.313481f, -0.21214f);
//		pose[7] = float3(-0.470237f, 0.636203f, -0.140866f);
//		std::vector<uint> indexes = { ALPHA(1), BETA(1), GAMMA(1), ALPHA(4), BETA(4), GAMMA(4), ALPHA(7), BETA(7), GAMMA(7) };
//
//		TestSkinningDerivatives(pose, indexes);
//	}
}