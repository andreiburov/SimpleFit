#pragma once
#include <Eigen/Eigen>

typedef Eigen::Matrix<float, 3, 4> Matrix3x4f;

Eigen::Matrix3f EulerRotation(float a, float b, float c);
Eigen::Matrix4f EulerSkinning(float a, float b, float c, float x, float y, float z);
Matrix3x4f EulerTruncatedSkinning(float a, float b, float c, float x, float y, float z);

Eigen::Matrix4f EulerSkinningDerivativeToAlpha(float a, float b, float c, float x, float y, float z);
Matrix3x4f EulerTruncatedSkinningDerivativeToAlpha(float a, float b, float c, float x, float y, float z);

Eigen::Matrix4f EulerSkinningDerivativeToBeta(float a, float b, float c, float x, float y, float z);
Matrix3x4f EulerTruncatedSkinningDerivativeToBeta(float a, float b, float c, float x, float y, float z);

Eigen::Matrix4f EulerSkinningDerivativeToGamma(float a, float b, float c, float x, float y, float z);
Matrix3x4f EulerTruncatedSkinningDerivativeToGamma(float a, float b, float c, float x, float y, float z);