#pragma once
#include <Eigen/Eigen>

typedef Eigen::Matrix<float, 3, 4> Matrix3x4f;

Eigen::Matrix3f EulerRotationZYX(float a, float b, float c);
Eigen::Matrix4f EulerSkinningZYX(float a, float b, float c, float x, float y, float z);
Matrix3x4f EulerTruncatedSkinningZYX(float a, float b, float c, float x, float y, float z);

Eigen::Matrix4f EulerSkinningZYXDerivativeToAlpha(float a, float b, float c, float x, float y, float z);
Matrix3x4f EulerTruncatedSkinningZYXDerivativeToAlpha(float a, float b, float c, float x, float y, float z);

Eigen::Matrix4f EulerSkinningZYXDerivativeToBeta(float a, float b, float c, float x, float y, float z);
Matrix3x4f EulerTruncatedSkinningZYXDerivativeToBeta(float a, float b, float c, float x, float y, float z);

Eigen::Matrix4f EulerSkinningZYXDerivativeToGamma(float a, float b, float c, float x, float y, float z);
Matrix3x4f EulerTruncatedSkinningZYXDerivativeToGamma(float a, float b, float c, float x, float y, float z);

//

Eigen::Matrix3f EulerRotationXYZ(float a, float b, float c);
Eigen::Matrix4f EulerSkinningXYZ(float a, float b, float c, float x, float y, float z);

Eigen::Matrix4f EulerSkinningXYZDerivativeToAlpha(float a, float b, float c, float x, float y, float z);
Eigen::Matrix4f EulerSkinningXYZDerivativeToBeta(float a, float b, float c, float x, float y, float z);
Eigen::Matrix4f EulerSkinningXYZDerivativeToGamma(float a, float b, float c, float x, float y, float z);