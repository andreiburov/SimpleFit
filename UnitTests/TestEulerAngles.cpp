#include <catch.hpp>
#include <EulerAngles.h>

namespace euler_angles
{
	float alpha = 0.7f;
	float beta = -0.3f;
	float gamma = 0.125f;
	float delta = 0.01f;

	Eigen::Vector3f t(1.1f, -0.5f, 2.1f);

	TEST_CASE("Euler Rotation ZYX")
	{
		Eigen::AngleAxisf aX(alpha, Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf aY(beta, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf aZ(gamma, Eigen::Vector3f::UnitZ());

		Eigen::Matrix3f M = (aZ * aY * aX).matrix();
		Eigen::Matrix3f R = EulerRotationZYX(alpha, beta, gamma);
		REQUIRE(M.isApprox(R));
	}

	TEST_CASE("Euler Rotation XYZ")
	{
		Eigen::AngleAxisf aX(2.22719f, Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf aY(2.99099f, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf aZ(-2.61493f, Eigen::Vector3f::UnitZ());

		Eigen::Matrix3f M = (aX * aY * aZ).matrix();
		Eigen::Matrix3f R = EulerRotationXYZ(2.22719f, 2.99099f, -2.61493f);
		REQUIRE(M.isApprox(R));
	}

	TEST_CASE("AngleAxis to Euler")
	{
		// 2.22719 2.99099 -2.61493 euler
		// -0.851371 0.375605 0.419657 aa
		Eigen::Vector3f aa(-0.851371f, 0.375605f, 0.419657f);
		Eigen::AngleAxisf axis_angle(aa.norm(), aa.normalized());

		Eigen::Vector3f euler = axis_angle.toRotationMatrix().eulerAngles(0, 1, 2);
		REQUIRE(euler.isApprox(Eigen::Vector3f(2.22719f, 2.99099f, -2.61493f)));
	}

	TEST_CASE("Euler to AxisAngle")
	{
		// 2.22719 2.99099 -2.61493 euler
		// -0.851371 0.375605 0.419657 aa
		Eigen::Vector3f euler(2.22719f, 2.99099f, -2.61493f);

		Eigen::AngleAxisf aX(euler(0), Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf aY(euler(1), Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf aZ(euler(2), Eigen::Vector3f::UnitZ());

		Eigen::AngleAxisf aa(aX * aY * aZ);
		Eigen::Vector3f axis_angle(aa.axis() * aa.angle());
		REQUIRE(axis_angle.isApprox(Eigen::Vector3f(-0.851371f, 0.375605f, 0.419657f)));
	}

	TEST_CASE("Euler Skinning ZYX")
	{
		Eigen::AngleAxisf aX(alpha, Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf aY(beta, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf aZ(gamma, Eigen::Vector3f::UnitZ());

		Eigen::Matrix4f M = (Eigen::Translation3f(t) * aZ * aY * aX * Eigen::Translation3f(-t)).matrix();
		Eigen::Matrix4f R = EulerSkinningZYX(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE(M.isApprox(R));
	}

	TEST_CASE("Euler Skinning XYZ")
	{
		Eigen::AngleAxisf aX(alpha, Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf aY(beta, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf aZ(gamma, Eigen::Vector3f::UnitZ());

		Eigen::Matrix4f M = (Eigen::Translation3f(t) * aX * aY * aZ * Eigen::Translation3f(-t)).matrix();
		Eigen::Matrix4f R = EulerSkinningXYZ(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE(M.isApprox(R));
	}

	TEST_CASE("Euler Truncated Skinning ZYX")
	{
		Eigen::Vector4f x(3.1f, 2.0f, -1.1f, 1.0f);

		Eigen::Matrix4f S = EulerSkinningZYX(alpha, beta, gamma, t(0), t(1), t(2));
		Matrix3x4f T = EulerTruncatedSkinningZYX(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE((S*x).head<3>().isApprox(T*x));
	}

	// Skinning ZYX Derivatives

	TEST_CASE("Skinning ZYX Derivative w.r.t. alpha")
	{
		Eigen::Matrix4f S1 = EulerSkinningZYX(alpha, beta, gamma, t(0), t(1), t(2));
		Eigen::Matrix4f S2 = EulerSkinningZYX(alpha + delta, beta, gamma, t(0), t(1), t(2));

		Eigen::Matrix4f dS1 = EulerSkinningZYXDerivativeToAlpha(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE(S2.isApprox(S1 + delta * dS1, delta));
	}

	TEST_CASE("Truncated Skinning ZYX Derivative w.r.t. alpha")
	{
		Matrix3x4f S1 = EulerTruncatedSkinningZYX(alpha, beta, gamma, t(0), t(1), t(2));
		Matrix3x4f S2 = EulerTruncatedSkinningZYX(alpha + delta, beta, gamma, t(0), t(1), t(2));

		Matrix3x4f dS1 = EulerTruncatedSkinningZYXDerivativeToAlpha(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE(S2.isApprox(S1 + delta * dS1, delta));
	}

	TEST_CASE("Skinning ZYX Derivative w.r.t. beta")
	{
		Eigen::Matrix4f S1 = EulerSkinningZYX(alpha, beta, gamma, t(0), t(1), t(2));
		Eigen::Matrix4f S2 = EulerSkinningZYX(alpha, beta + delta, gamma, t(0), t(1), t(2));

		Eigen::Matrix4f dS1 = EulerSkinningZYXDerivativeToBeta(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE(S2.isApprox(S1 + delta * dS1, delta));
	}

	TEST_CASE("Truncated Skinning ZYX Derivative w.r.t. beta")
	{
		Matrix3x4f S1 = EulerTruncatedSkinningZYX(alpha, beta, gamma, t(0), t(1), t(2));
		Matrix3x4f S2 = EulerTruncatedSkinningZYX(alpha, beta + delta, gamma, t(0), t(1), t(2));

		Matrix3x4f dS1 = EulerTruncatedSkinningZYXDerivativeToBeta(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE(S2.isApprox(S1 + delta * dS1, delta));
	}

	TEST_CASE("Skinning ZYX Derivative w.r.t. gamma")
	{
		Eigen::Matrix4f S1 = EulerSkinningZYX(alpha, beta, gamma, t(0), t(1), t(2));
		Eigen::Matrix4f S2 = EulerSkinningZYX(alpha, beta, gamma + delta, t(0), t(1), t(2));

		Eigen::Matrix4f dS1 = EulerSkinningZYXDerivativeToGamma(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE(S2.isApprox(S1 + delta * dS1, delta));
	}

	TEST_CASE("Truncated Skinning ZYX Derivative w.r.t. gamma")
	{
		Matrix3x4f S1 = EulerTruncatedSkinningZYX(alpha, beta, gamma, t(0), t(1), t(2));
		Matrix3x4f S2 = EulerTruncatedSkinningZYX(alpha, beta, gamma + delta, t(0), t(1), t(2));

		Matrix3x4f dS1 = EulerTruncatedSkinningZYXDerivativeToGamma(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE(S2.isApprox(S1 + delta * dS1, delta));
	}

	TEST_CASE("Skinning ZYX Derivative w.r.t. alpha and beta")
	{
		Eigen::Matrix4f S1 = EulerSkinningZYX(alpha, beta, gamma, t(0), t(1), t(2));
		Eigen::Matrix4f S2 = EulerSkinningZYX(alpha + delta, beta + delta, gamma, t(0), t(1), t(2));

		Eigen::Matrix4f daS1 = EulerSkinningZYXDerivativeToAlpha(alpha, beta, gamma, t(0), t(1), t(2));
		Eigen::Matrix4f dbS1 = EulerSkinningZYXDerivativeToBeta(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE(S2.isApprox(S1 + delta * daS1 + delta * dbS1, delta));
	}

	TEST_CASE("Skinning ZYX Derivative w.r.t. alpha and beta and gamma")
	{
		Eigen::Matrix4f S1 = EulerSkinningZYX(alpha, beta, gamma, t(0), t(1), t(2));
		Eigen::Matrix4f S2 = EulerSkinningZYX(alpha + delta, beta - delta, gamma + delta, t(0), t(1), t(2));

		Eigen::Matrix4f daS1 = EulerSkinningZYXDerivativeToAlpha(alpha, beta, gamma, t(0), t(1), t(2));
		Eigen::Matrix4f dbS1 = EulerSkinningZYXDerivativeToBeta(alpha, beta, gamma, t(0), t(1), t(2));
		Eigen::Matrix4f dgS1 = EulerSkinningZYXDerivativeToGamma(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE(S2.isApprox(S1 + delta * daS1 - delta * dbS1 + delta * dgS1, delta));
	}

	// Skinning XYZ Derivatives

	TEST_CASE("Skinning XYZ Derivative w.r.t. alpha")
	{
		Eigen::Matrix4f S1 = EulerSkinningXYZ(alpha, beta, gamma, t(0), t(1), t(2));
		Eigen::Matrix4f S2 = EulerSkinningXYZ(alpha + delta, beta, gamma, t(0), t(1), t(2));

		Eigen::Matrix4f dS1 = EulerSkinningXYZDerivativeToAlpha(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE(S2.isApprox(S1 + delta * dS1, delta));
	}

	TEST_CASE("Skinning XYZ Derivative w.r.t. beta")
	{
		Eigen::Matrix4f S1 = EulerSkinningXYZ(alpha, beta, gamma, t(0), t(1), t(2));
		Eigen::Matrix4f S2 = EulerSkinningXYZ(alpha, beta + delta, gamma, t(0), t(1), t(2));

		Eigen::Matrix4f dS1 = EulerSkinningXYZDerivativeToBeta(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE(S2.isApprox(S1 + delta * dS1, delta));
	}

	TEST_CASE("Skinning XYZ Derivative w.r.t. gamma")
	{
		Eigen::Matrix4f S1 = EulerSkinningXYZ(alpha, beta, gamma, t(0), t(1), t(2));
		Eigen::Matrix4f S2 = EulerSkinningXYZ(alpha, beta, gamma + delta, t(0), t(1), t(2));

		Eigen::Matrix4f dS1 = EulerSkinningXYZDerivativeToGamma(alpha, beta, gamma, t(0), t(1), t(2));
		REQUIRE(S2.isApprox(S1 + delta * dS1, delta));
	}
}