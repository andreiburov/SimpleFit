#include <catch.hpp>
#include <EulerAngles.h>

namespace euler_angles
{
	float alpha = 0.7f;
	float beta = -0.3f;
	float gamma = 0.125f;
	float delta = 0.01f;

	Eigen::Vector3f t(1.1f, -0.5f, 2.1f);

	TEST_CASE("Euler Rotation")
	{
		Eigen::AngleAxisf aX(alpha, Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf aY(beta, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf aZ(gamma, Eigen::Vector3f::UnitZ());

		Eigen::Matrix3f M = (aZ * aY * aX).matrix();
		Eigen::Matrix3f R = EulerRotation(alpha, beta, gamma);
		REQUIRE(M.isApprox(R));
	}

	TEST_CASE("Euler Skinning")
	{
		Eigen::AngleAxisf aX(alpha, Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf aY(beta, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf aZ(gamma, Eigen::Vector3f::UnitZ());

		Eigen::Matrix4f M = (Eigen::Translation3f(t) * aZ * aY * aX * Eigen::Translation3f(-t)).matrix();
		Eigen::Matrix4f R = EulerSkinning(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE(M.isApprox(R));
	}

	TEST_CASE("Euler Truncated Skinning")
	{
		Eigen::Vector4f x(3.1f, 2.0f, -1.1f, 1.0f);

		Eigen::Matrix4f S = EulerSkinning(alpha, beta, gamma, t(0), t(1), t(2));
		Matrix3x4f T = EulerTruncatedSkinning(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE((S*x).head<3>().isApprox(T*x));
	}

	TEST_CASE("Skinning Derivative w.r.t. alpha")
	{
		Eigen::Matrix4f S1 = EulerSkinning(alpha, beta, gamma, t(0), t(1), t(2));
		Eigen::Matrix4f S2 = EulerSkinning(alpha + delta, beta, gamma, t(0), t(1), t(2));

		Eigen::Matrix4f dS1 = EulerSkinningDerivativeToAlpha(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE(S2.isApprox(S1 + delta * dS1, delta));
	}

	TEST_CASE("Truncated Skinning Derivative w.r.t. alpha")
	{
		Matrix3x4f S1 = EulerTruncatedSkinning(alpha, beta, gamma, t(0), t(1), t(2));
		Matrix3x4f S2 = EulerTruncatedSkinning(alpha + delta, beta, gamma, t(0), t(1), t(2));

		Matrix3x4f dS1 = EulerTruncatedSkinningDerivativeToAlpha(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE(S2.isApprox(S1 + delta * dS1, delta));
	}

	TEST_CASE("Skinning Derivative w.r.t. beta")
	{
		Eigen::Matrix4f S1 = EulerSkinning(alpha, beta, gamma, t(0), t(1), t(2));
		Eigen::Matrix4f S2 = EulerSkinning(alpha, beta + delta, gamma, t(0), t(1), t(2));

		Eigen::Matrix4f dS1 = EulerSkinningDerivativeToBeta(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE(S2.isApprox(S1 + delta * dS1, delta));
	}

	TEST_CASE("Truncated Skinning Derivative w.r.t. beta")
	{
		Matrix3x4f S1 = EulerTruncatedSkinning(alpha, beta, gamma, t(0), t(1), t(2));
		Matrix3x4f S2 = EulerTruncatedSkinning(alpha, beta + delta, gamma, t(0), t(1), t(2));

		Matrix3x4f dS1 = EulerTruncatedSkinningDerivativeToBeta(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE(S2.isApprox(S1 + delta * dS1, delta));
	}

	TEST_CASE("Skinning Derivative w.r.t. gamma")
	{
		Eigen::Matrix4f S1 = EulerSkinning(alpha, beta, gamma, t(0), t(1), t(2));
		Eigen::Matrix4f S2 = EulerSkinning(alpha, beta, gamma + delta, t(0), t(1), t(2));

		Eigen::Matrix4f dS1 = EulerSkinningDerivativeToGamma(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE(S2.isApprox(S1 + delta * dS1, delta));
	}

	TEST_CASE("Truncated Skinning Derivative w.r.t. gamma")
	{
		Matrix3x4f S1 = EulerTruncatedSkinning(alpha, beta, gamma, t(0), t(1), t(2));
		Matrix3x4f S2 = EulerTruncatedSkinning(alpha, beta, gamma + delta, t(0), t(1), t(2));

		Matrix3x4f dS1 = EulerTruncatedSkinningDerivativeToGamma(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE(S2.isApprox(S1 + delta * dS1, delta));
	}

	TEST_CASE("Skinning Derivative w.r.t. alpha and beta")
	{
		Eigen::Matrix4f S1 = EulerSkinning(alpha, beta, gamma, t(0), t(1), t(2));
		Eigen::Matrix4f S2 = EulerSkinning(alpha + delta, beta + delta, gamma, t(0), t(1), t(2));

		Eigen::Matrix4f daS1 = EulerSkinningDerivativeToAlpha(alpha, beta, gamma, t(0), t(1), t(2));
		Eigen::Matrix4f dbS1 = EulerSkinningDerivativeToBeta(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE(S2.isApprox(S1 + delta * daS1 + delta * dbS1, delta));
	}

	TEST_CASE("Skinning Derivative w.r.t. alpha and beta and gamma")
	{
		Eigen::Matrix4f S1 = EulerSkinning(alpha, beta, gamma, t(0), t(1), t(2));
		Eigen::Matrix4f S2 = EulerSkinning(alpha + delta, beta - delta, gamma + delta, t(0), t(1), t(2));

		Eigen::Matrix4f daS1 = EulerSkinningDerivativeToAlpha(alpha, beta, gamma, t(0), t(1), t(2));
		Eigen::Matrix4f dbS1 = EulerSkinningDerivativeToBeta(alpha, beta, gamma, t(0), t(1), t(2));
		Eigen::Matrix4f dgS1 = EulerSkinningDerivativeToGamma(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE(S2.isApprox(S1 + delta * daS1 - delta * dbS1 + delta * dgS1, delta));
	}
}