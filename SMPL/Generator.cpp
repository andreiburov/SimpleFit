#include "Generator.h"

namespace smpl
{
	void IdentityMorph::operator()(const ShapeCoefficients& betas, std::vector<float3>& vertices) const
	{
#pragma omp parallel for
		for (uint i = 0; i < VERTEX_COUNT; i++)
		{
			Eigen::Vector3f v = vertices[i].ToEigen();
			for (uint j = 0; j < BETA_COUNT; j++)
			{
				v += betas[j] * shapedirs_[i*BETA_COUNT + j].ToEigen();
			}
			vertices[i] = float3(v);
		}
	}

	void PoseMorph::operator()(const PoseAxisAngleCoefficients& thetas, std::vector<float3>& vertices) const
	{
		// invariant to rotation of the parent joint
		Eigen::Matrix3f rotation[THETA_COUNT_WITHOUT_PARENT];

		for (uint i = 0; i < THETA_COUNT_WITHOUT_PARENT; i++)
		{
			Eigen::AngleAxisf angle_axis(thetas[i + 1].ToEigen().norm(), thetas[i + 1].ToEigen().normalized());
			// rotations will be flattened row-wise
			rotation[i] = (angle_axis.toRotationMatrix() - Eigen::Matrix3f::Identity()).transpose();
		}

#pragma omp parallel for
		for (uint i = 0; i < VERTEX_COUNT; i++)
		{
			Eigen::Vector3f v = vertices[i].ToEigen();
			for (uint j = 0; j < THETA_MATRIX_COMPONENT_COUNT; j++)
			{
				v += rotation[j / 9].data()[j % 9] * posedirs_[i*THETA_MATRIX_COMPONENT_COUNT + j].ToEigen();
			}
			vertices[i] = float3(v);
		}
	}

	void PoseMorph::operator()(const PoseEulerCoefficients& thetas, std::vector<float3>& vertices) const
	{
		// invariant to rotation of the parent joint
		Eigen::Matrix3f rotation[THETA_COUNT_WITHOUT_PARENT];

		for (uint i = 0; i < THETA_COUNT_WITHOUT_PARENT; i++)
		{
			// rotations will be flattened row-wise
			rotation[i] = (EulerRotationZYX(thetas[i].x, thetas[i].y, thetas[i].z) - Eigen::Matrix3f::Identity()).transpose();
		}

#pragma omp parallel for
		for (uint i = 0; i < VERTEX_COUNT; i++)
		{
			Eigen::Vector3f v = vertices[i].ToEigen();
			for (uint j = 0; j < THETA_MATRIX_COMPONENT_COUNT; j++)
			{
				v += rotation[j / 9].data()[j % 9] * posedirs_[i*THETA_MATRIX_COMPONENT_COUNT + j].ToEigen();
			}
			vertices[i] = float3(v);
		}
	}

	void SkinMorph::operator()(const PoseAxisAngleCoefficients& thetas, const Joints& joints, std::vector<float3>& vertices) const
	{
		Eigen::Matrix4f palette[JOINT_COUNT];

		// parent initialization
		{
			Eigen::AngleAxisf angle_axis(thetas[0].ToEigen().norm(), thetas[0].ToEigen().normalized());
			palette[0] = (Eigen::Translation3f(joints.col(0)) * angle_axis * Eigen::Translation3f(-joints.col(0))).matrix();
		}

		for (uint i = 1; i < JOINT_COUNT; i++)
		{
			Eigen::AngleAxisf angle_axis(thetas[i].ToEigen().norm(), thetas[i].ToEigen().normalized());
			palette[i] = palette[PARENT_INDEX[i]]
				* (Eigen::Translation3f(joints.col(i)) * angle_axis * Eigen::Translation3f(-joints.col(i))).matrix();
		}

#pragma omp parallel for
		for (uint i = 0; i < VERTEX_COUNT; i++)
		{
			Eigen::Matrix4f skin
				= skins_[i].weight.x * palette[skins_[i].joint_index.x]
				+ skins_[i].weight.y * palette[skins_[i].joint_index.y]
				+ skins_[i].weight.z * palette[skins_[i].joint_index.z]
				+ skins_[i].weight.w * palette[skins_[i].joint_index.w];

			vertices[i] = float3((skin * vertices[i].ToEigen().homogeneous()).head(3));
		}
	}

	void SkinMorph::operator()(const PoseEulerCoefficients& thetas, const Joints& joints, std::vector<float3>& vertices) const
	{
		Eigen::Matrix4f palette[JOINT_COUNT];

		// parent initialization
		{
			palette[0] = EulerSkinningZYX(thetas[0].x, thetas[0].y, thetas[0].z, joints.col(0)(0), joints.col(0)(1), joints.col(0)(2));
		}

		for (uint i = 1; i < JOINT_COUNT; i++)
		{
			palette[i] = palette[PARENT_INDEX[i]]
				* EulerSkinningZYX(thetas[i].x, thetas[i].y, thetas[i].z, joints.col(i)(0), joints.col(i)(1), joints.col(i)(2));
		}

#pragma omp parallel for
		for (uint i = 0; i < VERTEX_COUNT; i++)
		{
			Eigen::Matrix4f skin
				= skins_[i].weight.x * palette[skins_[i].joint_index.x]
				+ skins_[i].weight.y * palette[skins_[i].joint_index.y]
				+ skins_[i].weight.z * palette[skins_[i].joint_index.z]
				+ skins_[i].weight.w * palette[skins_[i].joint_index.w];

			vertices[i] = float3((skin * vertices[i].ToEigen().homogeneous()).head(3));
		}
	}

	Body Generator::operator()(const ShapeCoefficients& betas, const PoseAxisAngleCoefficients& thetas) const
	{
		Body body;
		body.vertices = vertices_;
		body.indices = indices_;

		identity_morph_(betas, body.vertices);
		Joints joints = joint_regressor_(body.vertices);
		pose_morph_(thetas, body.vertices);

		// for interspection
		body.deformed_template = body.vertices;

		skin_morph_(thetas, joints, body.vertices);

		return body;
	}

	Body Generator::operator()(const ShapeCoefficients& betas, const PoseEulerCoefficients& thetas) const
	{
		Body body;
		body.vertices = vertices_;
		body.indices = indices_;

		identity_morph_(betas, body.vertices);
		Joints joints = joint_regressor_(body.vertices);
		pose_morph_(thetas, body.vertices);

		// for interspection
		body.deformed_template = body.vertices;

		skin_morph_(thetas, joints, body.vertices);

		return body;
	}
}