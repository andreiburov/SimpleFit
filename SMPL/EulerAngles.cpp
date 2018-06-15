#include "EulerAngles.h"

Eigen::Matrix3f EulerRotation(float a, float b, float c)
{
	float r[] = {
		cos(b)*cos(c), -cos(a)*sin(c) + sin(a)*sin(b)*cos(c), sin(a)*sin(c) + cos(a)*sin(b)*cos(c), 
		cos(b)*sin(c), cos(a)*cos(c) + sin(a)*sin(b)*sin(c), -sin(a)*cos(c) + cos(a)*sin(b)*sin(c), 
		-sin(b), sin(a)*cos(b), cos(a)*cos(b)
	};

	Eigen::Matrix3f R = Eigen::Map<Eigen::Matrix3f, Eigen::RowMajor>(r, 3, 3).transpose();
	return R;
}

Eigen::Matrix4f EulerSkinning(float a, float b, float c, float x, float y, float z)
{
	float r[] = {
		cos(b)*cos(c), -cos(a)*sin(c) + sin(a)*sin(b)*cos(c), sin(a)*sin(c) + cos(a)*sin(b)*cos(c), -x * cos(b)*cos(c) + x - y * (sin(a)*sin(b)*cos(c) - cos(a)*sin(c)) - z * (sin(a)*sin(c) + cos(a)*sin(b)*cos(c)),
		cos(b)*sin(c), cos(a)*cos(c) + sin(a)*sin(b)*sin(c), -sin(a)*cos(c) + cos(a)*sin(b)*sin(c), -x * cos(b)*sin(c) - y * (sin(a)*sin(b)*sin(c) + cos(a)*cos(c)) + y - z * (-sin(a)*cos(c) + cos(a)*sin(b)*sin(c)),
		-sin(b), sin(a)*cos(b), cos(a)*cos(b), x*sin(b) - y * sin(a)*cos(b) - z * cos(a)*cos(b) + z,
		0, 0, 0, 1
	};

	Eigen::Matrix4f R = Eigen::Map<Eigen::Matrix4f, Eigen::RowMajor>(r, 4, 4).transpose();
	return R;
}

Matrix3x4f EulerTruncatedSkinning(float a, float b, float c, float x, float y, float z)
{
	float r[] = {
		cos(b)*cos(c), -cos(a)*sin(c) + sin(a)*sin(b)*cos(c), sin(a)*sin(c) + cos(a)*sin(b)*cos(c), -x * cos(b)*cos(c) + x - y * (sin(a)*sin(b)*cos(c) - cos(a)*sin(c)) - z * (sin(a)*sin(c) + cos(a)*sin(b)*cos(c)),
		cos(b)*sin(c), cos(a)*cos(c) + sin(a)*sin(b)*sin(c), -sin(a)*cos(c) + cos(a)*sin(b)*sin(c), -x * cos(b)*sin(c) - y * (sin(a)*sin(b)*sin(c) + cos(a)*cos(c)) + y - z * (-sin(a)*cos(c) + cos(a)*sin(b)*sin(c)),
		-sin(b), sin(a)*cos(b), cos(a)*cos(b), x*sin(b) - y * sin(a)*cos(b) - z * cos(a)*cos(b) + z
	};

	Matrix3x4f R = Eigen::Map<Eigen::Matrix<float, 4, 3>, Eigen::RowMajor>(r, 4, 3).transpose();
	return R;
}

Eigen::Matrix4f EulerSkinningDerivativeToAlpha(float a, float b, float c, float x, float y, float z)
{
	float r[] = {
		0, sin(a)*sin(c) + cos(a)*sin(b)*cos(c), -sin(a)*sin(b)*cos(c) + cos(a)*sin(c), -y * (sin(a)*sin(c) + cos(a)*sin(b)*cos(c)) - z * (-sin(a)*sin(b)*cos(c) + cos(a)*sin(c)),
		0, -sin(a)*cos(c) + cos(a)*sin(b)*sin(c), -sin(a)*sin(b)*sin(c) - cos(a)*cos(c), -y * (-sin(a)*cos(c) + cos(a)*sin(b)*sin(c)) - z * (-sin(a)*sin(b)*sin(c) - cos(a)*cos(c)),
		0, cos(a)*cos(b), -sin(a)*cos(b), -y * cos(a)*cos(b) + z * sin(a),cos(b),
		0, 0, 0, 0
	};

	Eigen::Matrix4f R = Eigen::Map<Eigen::Matrix4f, Eigen::RowMajor>(r, 4, 4).transpose();
	return R;
}

Matrix3x4f EulerTruncatedSkinningDerivativeToAlpha(float a, float b, float c, float x, float y, float z)
{
	float r[] = {
		0, sin(a)*sin(c) + cos(a)*sin(b)*cos(c), -sin(a)*sin(b)*cos(c) + cos(a)*sin(c), -y * (sin(a)*sin(c) + cos(a)*sin(b)*cos(c)) - z * (-sin(a)*sin(b)*cos(c) + cos(a)*sin(c)),
		0, -sin(a)*cos(c) + cos(a)*sin(b)*sin(c), -sin(a)*sin(b)*sin(c) - cos(a)*cos(c), -y * (-sin(a)*cos(c) + cos(a)*sin(b)*sin(c)) - z * (-sin(a)*sin(b)*sin(c) - cos(a)*cos(c)),
		0, cos(a)*cos(b), -sin(a)*cos(b), -y * cos(a)*cos(b) + z * sin(a),cos(b)
	};

	Matrix3x4f R = Eigen::Map<Eigen::Matrix<float, 4, 3>, Eigen::RowMajor>(r, 4, 3).transpose();
	return R;
}

Eigen::Matrix4f EulerSkinningDerivativeToBeta(float a, float b, float c, float x, float y, float z)
{
	float r[] = {
		-sin(b)*cos(c), sin(a)*cos(b)*cos(c), cos(a)*cos(b)*cos(c), x*sin(b)*cos(c) - y * sin(a)*cos(b)*cos(c) - z * cos(a)*cos(b)*cos(c),
		-sin(b)*sin(c), sin(a)*cos(b)*sin(c), cos(a)*cos(b)*sin(c), x*sin(b)*sin(c) - y * sin(a)*cos(b)*sin(c) - z * cos(a)*cos(b)*sin(c),
		-cos(b), -sin(a)*sin(b), -cos(a)*sin(b), x*cos(b) + y * sin(a)*sin(b) + z * sin(b)*cos(a),
		0, 0, 0, 0
	};

	Eigen::Matrix4f R = Eigen::Map<Eigen::Matrix4f, Eigen::RowMajor>(r, 4, 4).transpose();
	return R;
}

Matrix3x4f EulerTruncatedSkinningDerivativeToBeta(float a, float b, float c, float x, float y, float z)
{
	float r[] = {
		-sin(b)*cos(c), sin(a)*cos(b)*cos(c), cos(a)*cos(b)*cos(c), x*sin(b)*cos(c) - y * sin(a)*cos(b)*cos(c) - z * cos(a)*cos(b)*cos(c),
		-sin(b)*sin(c), sin(a)*cos(b)*sin(c), cos(a)*cos(b)*sin(c), x*sin(b)*sin(c) - y * sin(a)*cos(b)*sin(c) - z * cos(a)*cos(b)*sin(c),
		-cos(b), -sin(a)*sin(b), -cos(a)*sin(b), x*cos(b) + y * sin(a)*sin(b) + z * sin(b)*cos(a)
	};

	Matrix3x4f R = Eigen::Map<Eigen::Matrix<float, 4, 3>, Eigen::RowMajor>(r, 4, 3).transpose();
	return R;
}

Eigen::Matrix4f EulerSkinningDerivativeToGamma(float a, float b, float c, float x, float y, float z)
{
	float r[] = {
		-cos(b)*sin(c), -sin(a)*sin(b)*sin(c) - cos(a)*cos(c), sin(a)*cos(c) - cos(a)*sin(b)*sin(c), x*cos(b)*sin(c) - y * (-sin(a)*sin(b)*sin(c) - cos(a)*cos(c)) - z * (sin(a)*cos(c) - cos(a)*sin(b)*sin(c)),
		cos(b)*cos(c), sin(a)*sin(b)*cos(c) - cos(a)*sin(c), sin(a)*sin(c) + cos(a)*sin(b)*cos(c), -x * (cos(b)*cos(c)) - y * (sin(a)*sin(b)*cos(c) - cos(a)*sin(c)) - z * (sin(a)*sin(c) + cos(a)*sin(b)*cos(c)),
		0, 0, 0, 0,
		0, 0, 0, 0
	};

	Eigen::Matrix4f R = Eigen::Map<Eigen::Matrix4f, Eigen::RowMajor>(r, 4, 4).transpose();
	return R;
}

Matrix3x4f EulerTruncatedSkinningDerivativeToGamma(float a, float b, float c, float x, float y, float z)
{
	float r[] = {
		-cos(b)*sin(c), -sin(a)*sin(b)*sin(c) - cos(a)*cos(c), sin(a)*cos(c) - cos(a)*sin(b)*sin(c), x*cos(b)*sin(c) - y * (-sin(a)*sin(b)*sin(c) - cos(a)*cos(c)) - z * (sin(a)*cos(c) - cos(a)*sin(b)*sin(c)),
		cos(b)*cos(c), sin(a)*sin(b)*cos(c) - cos(a)*sin(c), sin(a)*sin(c) + cos(a)*sin(b)*cos(c), -x * (cos(b)*cos(c)) - y * (sin(a)*sin(b)*cos(c) - cos(a)*sin(c)) - z * (sin(a)*sin(c) + cos(a)*sin(b)*cos(c)),
		0, 0, 0, 0
	};

	Matrix3x4f R = Eigen::Map<Eigen::Matrix<float, 4, 3>, Eigen::RowMajor>(r, 4, 3).transpose();
	return R;
}