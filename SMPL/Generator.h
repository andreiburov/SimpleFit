#pragma once

#include "Definitions.h"
#include "Utils.h"
#include "JointRegressor.h"
#include "EulerAngles.h"

namespace smpl {

	class IdentityMorph
	{
	public:
		IdentityMorph(std::vector<float3>& shapedirs)
			:shapedirs_(std::move(shapedirs))
		{
		}

		void operator()(const ShapeCoefficients& betas, std::vector<float3>& vertices) const
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

		const std::vector<float3>& GetShapeDirs() const
		{
			return shapedirs_;
		}

	private:
		const std::vector<float3> shapedirs_;
	};

	class PoseMorph
	{
	public:
		PoseMorph(std::vector<float3>& posedirs) :
			posedirs_(std::move(posedirs))
		{
		}

		void operator()(const PoseAxisAngleCoefficients& thetas, std::vector<float3>& vertices) const
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

		void operator()(const PoseEulerCoefficients& thetas, std::vector<float3>& vertices) const
		{
			// invariant to rotation of the parent joint
			Eigen::Matrix3f rotation[THETA_COUNT_WITHOUT_PARENT];

			for (uint i = 0; i < THETA_COUNT_WITHOUT_PARENT; i++)
			{
				// rotations will be flattened row-wise
				rotation[i] = (EulerRotation(thetas[i].x, thetas[i].y, thetas[i].z) - Eigen::Matrix3f::Identity()).transpose();
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

	private:
		const std::vector<float3> posedirs_;
	};

	class SkinMorph
	{
	public:
		SkinMorph(std::vector<Skin>& skins) :
			skins_(std::move(skins))
		{
		}

		void operator()(const PoseAxisAngleCoefficients& thetas, const Joints& joints, std::vector<float3>& vertices) const
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

		void operator()(const PoseEulerCoefficients& thetas, const Joints& joints, std::vector<float3>& vertices) const
		{
			Eigen::Matrix4f palette[JOINT_COUNT];

			// parent initialization
			{
				palette[0] = EulerSkinning(thetas[0].x, thetas[0].y, thetas[0].z, joints.col(0)(0), joints.col(0)(1), joints.col(0)(2));
			}

			for (uint i = 1; i < JOINT_COUNT; i++)
			{
				palette[i] = palette[PARENT_INDEX[i]]
					* EulerSkinning(thetas[i].x, thetas[i].y, thetas[i].z, joints.col(i)(0), joints.col(i)(1), joints.col(i)(2));
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

		const std::vector<Skin>& GetSkins() const
		{
			return skins_;
		}

	private:
		const std::vector<Skin> skins_;
	};

	class Generator
	{
	public:
		struct Configuration
		{
			Configuration(const std::string& configuration_path)
			{
				ReadObjFile(configuration_path + std::string("/smpl_template.obj"), vertices, indices, skins);
				ReadFloat3FromBinaryFile(configuration_path + std::string("/smpl_posedirs.bin"), posedirs, POSEDIRS_PER_VERTEX);
				ReadFloat3FromBinaryFile(configuration_path + std::string("/smpl_shapedirs.bin"), shapedirs, SHAPEDIRS_PER_VERTEX);
				ReadSparseMatrixFile(configuration_path + std::string("/smpl_regressor.txt"), joint_regressor);
			}

			std::vector<float3> vertices;
			std::vector<uint> indices;
			std::vector<Skin> skins;
			std::vector<float3> posedirs;
			std::vector<float3> shapedirs;
			SparseMatrix joint_regressor;
		};

		Generator(Configuration configuration) :
			vertices_(configuration.vertices), indices_(configuration.indices),
			identity_morph_(configuration.shapedirs), joint_regressor_(configuration.joint_regressor, JOINT_COUNT),
			pose_morph_(configuration.posedirs), skin_morph_(configuration.skins)
		{
		}
		
		Body operator()(const ShapeCoefficients& betas, const PoseAxisAngleCoefficients& thetas) const
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

		Body operator()(const ShapeCoefficients& betas, const PoseEulerCoefficients& thetas) const
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

		const std::vector<float3>& GetShapeDirs() const
		{
			return identity_morph_.GetShapeDirs();
		}

		const std::vector<Skin>& GetSkins() const
		{
			return skin_morph_.GetSkins();
		}

	private:
		const std::vector<float3> vertices_;
		const std::vector<uint> indices_;
		const IdentityMorph identity_morph_;
		const JointRegressor joint_regressor_;
		const PoseMorph pose_morph_;
		const SkinMorph skin_morph_;
	};
}