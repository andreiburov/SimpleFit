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
			rotation[i] = (EulerRotationXYZ(thetas[i].x, thetas[i].y, thetas[i].z) - Eigen::Matrix3f::Identity()).transpose();
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

	void SkinMorph::operator()(const PoseEulerCoefficients& thetas, 
		const Joints& joints, std::vector<float3>& vertices) const
	{
		Eigen::Matrix4f palette[JOINT_COUNT];
		// parent initialization
		{
			palette[0] = EulerSkinningXYZ(thetas[0].x, thetas[0].y, thetas[0].z, joints.col(0)(0), joints.col(0)(1), joints.col(0)(2));
		}

		for (uint i = 1; i < JOINT_COUNT; i++)
		{
			palette[i] = palette[PARENT_INDEX[i]]
				* EulerSkinningXYZ(thetas[i].x, thetas[i].y, thetas[i].z, joints.col(i)(0), joints.col(i)(1), joints.col(i)(2));
		}

		uint stride = static_cast<uint>(vertices.size() / VERTEX_COUNT);

#pragma omp parallel for
		for (uint i = 0; i < VERTEX_COUNT; i++)
		{
			Eigen::Matrix4f skin
				= skins_[i].weight.x * palette[skins_[i].joint_index.x]
				+ skins_[i].weight.y * palette[skins_[i].joint_index.y]
				+ skins_[i].weight.z * palette[skins_[i].joint_index.z]
				+ skins_[i].weight.w * palette[skins_[i].joint_index.w];

			for (uint j = 0; j < stride; j++)
			{
				vertices[i*stride + j] = 
					float3((skin * vertices[i*stride + j].ToEigen().homogeneous()).head(3));
			}
		}
	}

	Body Generator::operator()(const ShapeCoefficients& betas, const PoseAxisAngleCoefficients& thetas) const
	{
		Body body;
		body.vertices = vertices_;
		body.indices = indices_;

		identity_morph_(betas, body.vertices);
		body.joints = joint_regressor_(body.vertices);
		pose_morph_(thetas, body.vertices);

		// for interspection
		body.deformed_template = body.vertices;

		skin_morph_(thetas, body.joints, body.vertices);

		return body;
	}

	Body Generator::operator()() const
	{
		ShapeCoefficients betas;
		PoseEulerCoefficients thetas;
		
		return operator()(betas, thetas, false);
	}

	Body Generator::operator()(bool with_normals) const
	{
		ShapeCoefficients betas;
		PoseEulerCoefficients thetas;

		return operator()(betas, thetas, with_normals);
	}

	Body Generator::operator()(const ShapeCoefficients& betas,
		const PoseEulerCoefficients& thetas) const
	{
		return operator()(betas, thetas, false);
	}

	Body Generator::operator()(const ShapeCoefficients& betas, 
		const PoseEulerCoefficients& thetas, bool with_normals) const
	{
		Body body;
		body.vertices = vertices_;
		body.indices = indices_;

		identity_morph_(betas, body.vertices);
		body.joints = joint_regressor_(body.vertices);
		pose_morph_(thetas, body.vertices);

		// for interspection
		body.deformed_template = body.vertices;
		skin_morph_(thetas, body.joints, body.vertices);

		if (with_normals)
		{
			CalculateNormals(body);
		}

		return body;
	}

	void Generator::CalculateNormals(Body& body) const
	{
		body.vertices_normals.reserve(VERTEX_COUNT);
		std::vector<Eigen::Vector3f> normals(VERTEX_COUNT, Eigen::Vector3f(0.f, 0.f, 0.f));

		for (uint i = 0; i < FACE_COUNT * 3; i += 3)
		{
			uint id0 = indices_[i];
			uint id1 = indices_[i+1];
			uint id2 = indices_[i+2];

			// not normalized, the value is proportional to spanned triangle
			Eigen::Vector3f face_normal = (vertices_[id1].ToEigen() - vertices_[id0].ToEigen())
				.cross(vertices_[id2].ToEigen() - vertices_[id0].ToEigen());

			normals[id0] += face_normal;
			normals[id1] += face_normal;
			normals[id2] += face_normal;
		}

		for (uint i = 0; i < VERTEX_COUNT; i++)
		{
			body.vertices_normals.push_back(float6(body.vertices[i], float3(normals[i].normalized())));
		}
	}
}