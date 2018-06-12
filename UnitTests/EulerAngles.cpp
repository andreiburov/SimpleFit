#include <catch.hpp>
#include <Eigen/Eigen>

float alpha = 0.7f;
float beta = -0.3f;
float gamma = 0.125f;
float delta = 0.01f;

Eigen::Vector3f t(1.1f, -0.5f, 2.1f);

Eigen::Matrix4f EulerRotation(float a, float b, float c)
{
	float r[] = {
		cos(b)*cos(c), -cos(a)*sin(c) + sin(a)*sin(b)*cos(c), sin(a)*sin(c) + cos(a)*sin(b)*cos(c), 0,
		cos(b)*sin(c), cos(a)*cos(c) + sin(a)*sin(b)*sin(c), -sin(a)*cos(c) + cos(a)*sin(b)*sin(c), 0,
		-sin(b), sin(a)*cos(b), cos(a)*cos(b), 0,
		0, 0, 0, 1 
	};

	Eigen::Matrix4f R = Eigen::Map<Eigen::Matrix4f, Eigen::RowMajor>(r, 4, 4).transpose();
	return R;
}

Eigen::Matrix4f EulerSkinning(float a, float b, float c, float x, float y, float z)
{
	float r[] = {
		cos(b)*cos(c), -cos(a)*sin(c) + sin(a)*sin(b)*cos(c), sin(a)*sin(c) + cos(a)*sin(b)*cos(c), -x*cos(b)*cos(c) + x - y*(sin(a)*sin(b)*cos(c)-cos(a)*sin(c)) - z*(sin(a)*sin(c)+cos(a)*sin(b)*cos(c)),
		cos(b)*sin(c), cos(a)*cos(c) + sin(a)*sin(b)*sin(c), -sin(a)*cos(c) + cos(a)*sin(b)*sin(c), -x*cos(b)*sin(c) - y*(sin(a)*sin(b)*sin(c)+cos(a)*cos(c)) + y - z*(-sin(a)*cos(c)+cos(a)*sin(b)*sin(c)),
		-sin(b), sin(a)*cos(b), cos(a)*cos(b), x*sin(b) - y*sin(a)*cos(b) - z*cos(a)*cos(b) + z,
		0, 0, 0, 1
	};

	Eigen::Matrix4f R = Eigen::Map<Eigen::Matrix4f, Eigen::RowMajor>(r, 4, 4).transpose();
	return R;
}

TEST_CASE("Euler Rotation")
{
	Eigen::AngleAxisf aX(alpha, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf aY(beta, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf aZ(gamma, Eigen::Vector3f::UnitZ());

	Eigen::Matrix3f M = (aZ * aY * aX).matrix();
	Eigen::Matrix3f R = EulerRotation(alpha, beta, gamma).block<3, 3>(0, 0);
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

Eigen::Matrix4f EulerSkinningDerivativeToAlpha(float a, float b, float c, float x, float y, float z)
{
	float r[] = {
		0, sin(a)*sin(c)+cos(a)*sin(b)*cos(c), -sin(a)*sin(b)*cos(c)+cos(a)*sin(c), -y*(sin(a)*sin(c)+cos(a)*sin(b)*cos(c)) - z*(-sin(a)*sin(b)*cos(c)+cos(a)*sin(c)),
		0, -sin(a)*cos(c)+cos(a)*sin(b)*sin(c), -sin(a)*sin(b)*sin(c)-cos(a)*cos(c), -y*(-sin(a)*cos(c)+cos(a)*sin(b)*sin(c)) - z*(-sin(a)*sin(b)*sin(c)-cos(a)*cos(c)),
		0, cos(a)*cos(b), -sin(a)*cos(b), -y*cos(a)*cos(b) + z*sin(a),cos(b),
		0, 0, 0, 0
	};

	Eigen::Matrix4f R = Eigen::Map<Eigen::Matrix4f, Eigen::RowMajor>(r, 4, 4).transpose();
	return R;
}

TEST_CASE("Skinning Derivative w.r.t. alpha")
{
	Eigen::Matrix4f S1 = EulerSkinning(alpha, beta, gamma, t(0), t(1), t(2));
	Eigen::Matrix4f S2 = EulerSkinning(alpha + delta, beta, gamma, t(0), t(1), t(2));

	Eigen::Matrix4f dS1 = EulerSkinningDerivativeToAlpha(alpha, beta, gamma, t(0), t(1), t(2));
	REQUIRE(S2.isApprox(S1 + delta * dS1, delta));
}

Eigen::Matrix4f EulerSkinningDerivativeToBeta(float a, float b, float c, float x, float y, float z)
{
	float r[] = {
		-sin(b)*cos(c), sin(a)*cos(b)*cos(c), cos(a)*cos(b)*cos(c), x*sin(b)*cos(c) - y*sin(a)*cos(b)*cos(c) - z*cos(a)*cos(b)*cos(c),
		-sin(b)*sin(c), sin(a)*cos(b)*sin(c), cos(a)*cos(b)*sin(c), x*sin(b)*sin(c) - y*sin(a)*cos(b)*sin(c) - z*cos(a)*cos(b)*sin(c),
		-cos(b), -sin(a)*sin(b), -cos(a)*sin(b), x*cos(b) + y*sin(a)*sin(b) + z*sin(b)*cos(a),
		0, 0, 0, 0
	};

	Eigen::Matrix4f R = Eigen::Map<Eigen::Matrix4f, Eigen::RowMajor>(r, 4, 4).transpose();
	return R;
}

TEST_CASE("Skinning Derivative w.r.t. beta")
{
	Eigen::Matrix4f S1 = EulerSkinning(alpha, beta, gamma, t(0), t(1), t(2));
	Eigen::Matrix4f S2 = EulerSkinning(alpha, beta + delta, gamma, t(0), t(1), t(2));

	Eigen::Matrix4f dS1 = EulerSkinningDerivativeToBeta(alpha, beta, gamma, t(0), t(1), t(2));
	REQUIRE(S2.isApprox(S1 + delta * dS1, delta));
}

Eigen::Matrix4f EulerSkinningDerivativeToGamma(float a, float b, float c, float x, float y, float z)
{
	float r[] = {
		-cos(b)*sin(c), -sin(a)*sin(b)*sin(c)-cos(a)*cos(c), sin(a)*cos(c)-cos(a)*sin(b)*sin(c), x*cos(b)*sin(c) - y*(-sin(a)*sin(b)*sin(c) - cos(a)*cos(c)) - z*(sin(a)*cos(c) - cos(a)*sin(b)*sin(c)),
		cos(b)*cos(c), sin(a)*sin(b)*cos(c)-cos(a)*sin(c), sin(a)*sin(c)+cos(a)*sin(b)*cos(c), -x*(cos(b)*cos(c)) - y*(sin(a)*sin(b)*cos(c) - cos(a)*sin(c)) - z*(sin(a)*sin(c) + cos(a)*sin(b)*cos(c)),
		0, 0, 0, 0,
		0, 0, 0, 0
	};

	Eigen::Matrix4f R = Eigen::Map<Eigen::Matrix4f, Eigen::RowMajor>(r, 4, 4).transpose();
	return R;
}

TEST_CASE("Skinning Derivative w.r.t. gamma")
{
	Eigen::Matrix4f S1 = EulerSkinning(alpha, beta, gamma, t(0), t(1), t(2));
	Eigen::Matrix4f S2 = EulerSkinning(alpha, beta, gamma + delta, t(0), t(1), t(2));

	Eigen::Matrix4f dS1 = EulerSkinningDerivativeToGamma(alpha, beta, gamma, t(0), t(1), t(2));
	REQUIRE(S2.isApprox(S1 + delta * dS1, delta));
}