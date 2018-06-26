#include "EulerAngles.h"

// ZYX

Eigen::Matrix3f EulerRotationZYX(float a, float b, float c)
{
	float r[] = {
		cos(b)*cos(c), -cos(a)*sin(c) + sin(a)*sin(b)*cos(c), sin(a)*sin(c) + cos(a)*sin(b)*cos(c), 
		cos(b)*sin(c), cos(a)*cos(c) + sin(a)*sin(b)*sin(c), -sin(a)*cos(c) + cos(a)*sin(b)*sin(c), 
		-sin(b), sin(a)*cos(b), cos(a)*cos(b)
	};

	Eigen::Matrix3f R = Eigen::Map<Eigen::Matrix3f, Eigen::RowMajor>(r, 3, 3).transpose();
	return R;
}

Eigen::Matrix4f EulerSkinningZYX(float a, float b, float c, float x, float y, float z)
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

Matrix3x4f EulerTruncatedSkinningZYX(float a, float b, float c, float x, float y, float z)
{
	float r[] = {
		cos(b)*cos(c), -cos(a)*sin(c) + sin(a)*sin(b)*cos(c), sin(a)*sin(c) + cos(a)*sin(b)*cos(c), -x * cos(b)*cos(c) + x - y * (sin(a)*sin(b)*cos(c) - cos(a)*sin(c)) - z * (sin(a)*sin(c) + cos(a)*sin(b)*cos(c)),
		cos(b)*sin(c), cos(a)*cos(c) + sin(a)*sin(b)*sin(c), -sin(a)*cos(c) + cos(a)*sin(b)*sin(c), -x * cos(b)*sin(c) - y * (sin(a)*sin(b)*sin(c) + cos(a)*cos(c)) + y - z * (-sin(a)*cos(c) + cos(a)*sin(b)*sin(c)),
		-sin(b), sin(a)*cos(b), cos(a)*cos(b), x*sin(b) - y * sin(a)*cos(b) - z * cos(a)*cos(b) + z
	};

	Matrix3x4f R = Eigen::Map<Eigen::Matrix<float, 4, 3>, Eigen::RowMajor>(r, 4, 3).transpose();
	return R;
}

Eigen::Matrix4f EulerSkinningZYXDerivativeToAlpha(float a, float b, float c, float x, float y, float z)
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

Matrix3x4f EulerTruncatedSkinningZYXDerivativeToAlpha(float a, float b, float c, float x, float y, float z)
{
	float r[] = {
		0, sin(a)*sin(c) + cos(a)*sin(b)*cos(c), -sin(a)*sin(b)*cos(c) + cos(a)*sin(c), -y * (sin(a)*sin(c) + cos(a)*sin(b)*cos(c)) - z * (-sin(a)*sin(b)*cos(c) + cos(a)*sin(c)),
		0, -sin(a)*cos(c) + cos(a)*sin(b)*sin(c), -sin(a)*sin(b)*sin(c) - cos(a)*cos(c), -y * (-sin(a)*cos(c) + cos(a)*sin(b)*sin(c)) - z * (-sin(a)*sin(b)*sin(c) - cos(a)*cos(c)),
		0, cos(a)*cos(b), -sin(a)*cos(b), -y * cos(a)*cos(b) + z * sin(a),cos(b)
	};

	Matrix3x4f R = Eigen::Map<Eigen::Matrix<float, 4, 3>, Eigen::RowMajor>(r, 4, 3).transpose();
	return R;
}

Eigen::Matrix4f EulerSkinningZYXDerivativeToBeta(float a, float b, float c, float x, float y, float z)
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

Matrix3x4f EulerTruncatedSkinningZYXDerivativeToBeta(float a, float b, float c, float x, float y, float z)
{
	float r[] = {
		-sin(b)*cos(c), sin(a)*cos(b)*cos(c), cos(a)*cos(b)*cos(c), x*sin(b)*cos(c) - y * sin(a)*cos(b)*cos(c) - z * cos(a)*cos(b)*cos(c),
		-sin(b)*sin(c), sin(a)*cos(b)*sin(c), cos(a)*cos(b)*sin(c), x*sin(b)*sin(c) - y * sin(a)*cos(b)*sin(c) - z * cos(a)*cos(b)*sin(c),
		-cos(b), -sin(a)*sin(b), -cos(a)*sin(b), x*cos(b) + y * sin(a)*sin(b) + z * sin(b)*cos(a)
	};

	Matrix3x4f R = Eigen::Map<Eigen::Matrix<float, 4, 3>, Eigen::RowMajor>(r, 4, 3).transpose();
	return R;
}

Eigen::Matrix4f EulerSkinningZYXDerivativeToGamma(float a, float b, float c, float x, float y, float z)
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

Matrix3x4f EulerTruncatedSkinningZYXDerivativeToGamma(float a, float b, float c, float x, float y, float z)
{
	float r[] = {
		-cos(b)*sin(c), -sin(a)*sin(b)*sin(c) - cos(a)*cos(c), sin(a)*cos(c) - cos(a)*sin(b)*sin(c), x*cos(b)*sin(c) - y * (-sin(a)*sin(b)*sin(c) - cos(a)*cos(c)) - z * (sin(a)*cos(c) - cos(a)*sin(b)*sin(c)),
		cos(b)*cos(c), sin(a)*sin(b)*cos(c) - cos(a)*sin(c), sin(a)*sin(c) + cos(a)*sin(b)*cos(c), -x * (cos(b)*cos(c)) - y * (sin(a)*sin(b)*cos(c) - cos(a)*sin(c)) - z * (sin(a)*sin(c) + cos(a)*sin(b)*cos(c)),
		0, 0, 0, 0
	};

	Matrix3x4f R = Eigen::Map<Eigen::Matrix<float, 4, 3>, Eigen::RowMajor>(r, 4, 3).transpose();
	return R;
}

// XZY

Eigen::Matrix3f EulerRotationXYZ(float a, float b, float c)
{
	const float s1 = sin(a);
	const float s2 = sin(b);
	const float s3 = sin(c);
	const float c1 = cos(a);
	const float c2 = cos(b);
	const float c3 = cos(c);

	Eigen::Matrix3f result;
	result << c2*c3, -c2*s3, s2,
		c1*s3 + c3*s1*s2, c1*c3 - s1*s2*s3, -c2*s1,
		s1*s3 - c1*c3*s2, c3*s1 + c1*s2*s3, c1*c2;

	return result;
}

Eigen::Matrix4f EulerSkinningXYZ(float a, float b, float c, float x, float y, float z)
{
	const float s1 = sin(a);
	const float s2 = sin(b);
	const float s3 = sin(c);
	const float c1 = cos(a);
	const float c2 = cos(b);
	const float c3 = cos(c);

	Eigen::Matrix4f result;
	result << c2*c3, -c2*s3, s2, -x * c2*c3 + x + y * s3*c2 - z * s2,
		c1*s3 + c3*s1*s2, c1*c3 - s1*s2*s3, -c2*s1, -x*(s1*s2*c3 + s3*c1) - y*(-s1*s2*s3 + c1*c3) + y + z*s1*c2,
		s1*s3 - c1*c3*s2, c3*s1 + c1*s2*s3, c1*c2, -x*(s1*s3 - s2*c1*c3) - y*(s1*c3 + s2*s3*c1) - z*c1*c2 + z,
		0, 0, 0, 1;

	return result;
}

Eigen::Matrix4f EulerSkinningXYZDerivativeToAlpha(float a, float b, float c, float x, float y, float z)
{
	const float s1 = sin(a);
	const float s2 = sin(b);
	const float s3 = sin(c);
	const float c1 = cos(a);
	const float c2 = cos(b);
	const float c3 = cos(c);

	Eigen::Matrix4f result;
	result << 0, 0, 0, 0,
		-s1*s3+s2*c1*c3, -s1*c3-s2*s3*c1, -c1*c2, -x * (-s1 * s3 + s2 * c1*c3) - y * (-s1 * c3 - s2 * s3*c1) + z * c1*c2,
		s1*s2*c3 + s3 * c1, -s1 * s2*s3 + c1 * c3, -s1 * c2, -x * (s1*s2*c3 + s3 * c1) - y * (-s1 * s2*s3 + c1 * c3) + z * s1*c2,
		0, 0, 0, 0;

	return result;
}

Eigen::Matrix4f EulerSkinningXYZDerivativeToBeta(float a, float b, float c, float x, float y, float z)
{
	const float s1 = sin(a);
	const float s2 = sin(b);
	const float s3 = sin(c);
	const float c1 = cos(a);
	const float c2 = cos(b);
	const float c3 = cos(c);

	Eigen::Matrix4f result;
	result << -s2 * c3, s2*s3, c2, x*s2*c3 - y * s2*s3 - z * c2,
		s1*c2*c3, -s1 * s3*c2, s1*s2, -x * s1*c2*c3 + y * s1*s3*c2 - z * s1*s2,
		-c1 * c2*c3, s3*c1*c2, -s2 * c1, x*c1*c2*c3 - y * s3*c1*c2 + z * s2*c1,
		0, 0, 0, 0;

	return result;
}

Eigen::Matrix4f EulerSkinningXYZDerivativeToGamma(float a, float b, float c, float x, float y, float z)
{
	const float s1 = sin(a);
	const float s2 = sin(b);
	const float s3 = sin(c);
	const float c1 = cos(a);
	const float c2 = cos(b);
	const float c3 = cos(c);

	Eigen::Matrix4f result;
	result << -s3 * c2, -c2 * c3, 0, x*s3*c2 + y * c2*c3,
		-s1 * s2*s3 + c1 * c3, -s1 * s2*c3 - s3 * c1, 0, -x * (-s1 * s2*s3 + c1 * c3) - y * (-s1 * s2*c3 - s3 * c1),
		s1*c3 + s2 * s3*c1, -s1 * s3 + s2 * c1*c3, 0, -x * (s1*c3 + s2 * s3*c1) - y * (-s1 * s3 + s2 * c1*c3),
		0, 0, 0, 0;

	return result;
}