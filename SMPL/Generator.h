#pragma once

#include "Definitions.h"
#include "Body.h"
#include "Utils.h"
#include "JointsRegressor.h"
#include "EulerAngles.h"

namespace smpl {

	class IdentityMorph
	{
	public:
		IdentityMorph(std::vector<float3>& shapedirs)
			:shapedirs_(std::move(shapedirs))
		{
		}

		void operator()(const ShapeCoefficients& betas, std::vector<float3>& vertices) const;

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

		void operator()(const PoseAxisAngleCoefficients& thetas, std::vector<float3>& vertices) const;

		void operator()(const PoseEulerCoefficients& thetas, std::vector<float3>& vertices) const;

	private:
		const std::vector<float3> posedirs_;
	};

	class SkinMorph
	{
	public:
		SkinMorph(std::vector<Skin>& skins) :
			skins_(std::move(skins))
		{
			for (uint i = 0; i < JOINT_COUNT; i++)
			{
				part_transformations_[i] = Eigen::Matrix4f::Identity();
				palette_[i] = Eigen::Matrix4f::Identity();
			}
		}

		void operator()(const PoseAxisAngleCoefficients& thetas, 
			const RegressedJoints& joints, std::vector<float3>& vertices) const;

		/*
			Applies skinning to the vertices of the smpl body.
			When vertices.size() is bigger than VERTEX_COUNT, the same skinning
			is applied to stride = vertices.size() / VERTEX_COUNT number of vertices.
		*/
		void operator()(const PoseEulerCoefficients& thetas, 
			const RegressedJoints& joints, std::vector<float3>& vertices) const;

		/*
			Applies skinning to the vertices of the smpl body.
			The matrix palette from previous invocation of 
			operator()(const PoseEulerCoefficients&, const RegressedJoints&, std::vector<float3>&)
			is used.
		*/
		void operator()(std::vector<float3>& vertices) const;

		/*
			Computes skinning Jacobian.
			
			float4x4 dskinning[THETA_COMPONENT_COUNT * JOINT_COUNT]

				body_parts 
				__________
			a | theta0.x
			n | theta0.y
			g | theta0.z
			l | theta1.x
			e | theta1.y 
			s | ...
			
			dskinning[angle_component * joint_count + body_part] - derivative of skinning in body part w.r.t. angle component
		*/
		void ComputeSkinningJacobian(const Body& body, Eigen::Matrix4f* dskinning) const;

		/*
			Computes Jacobian in Body from Pose.

			float3 dpose[VERTEX_COUNT * THETA_COMPONENT_COUNT]
		*/
		void ComputeBodyFromPoseJacobian(const Body& body, std::vector<float3>& dpose) const;

		const std::vector<Skin>& GetSkins() const
		{
			return skins_;
		}

		const Eigen::Matrix4f (&GetPalette() const) [JOINT_COUNT]
		{
			return palette_;
		}

		const Eigen::Matrix4f(&GetPartTransformations() const) [JOINT_COUNT]
		{
			return part_transformations_;
		}

	private:
		const std::vector<Skin> skins_;
		mutable Eigen::Matrix4f part_transformations_[JOINT_COUNT]; // blended unhierarchical per part transformations
		mutable Eigen::Matrix4f palette_[JOINT_COUNT]; // hierarchical skinning palette used for rendering
	};

	class Generator
	{
	public:
		struct Configuration
		{
			Configuration(const std::string& configuration_path)
			{
				ReadObjFile(configuration_path + std::string("/smpl_template.obj"), vertices, indices, skins);
				ReadFloat3FromBinary(configuration_path + std::string("/smpl_posedirs.bin"), posedirs, POSEDIRS_PER_VERTEX);
				ReadFloat3FromBinary(configuration_path + std::string("/smpl_shapedirs.bin"), shapedirs, SHAPEDIRS_PER_VERTEX);
				ReadSparseMatrixFile(configuration_path + std::string("/smpl_regressor.txt"), joint_regressor);
			}

			std::vector<float3> vertices;
			std::vector<uint> indices;
			std::vector<Skin> skins;
			std::vector<float3> posedirs;
			std::vector<float3> shapedirs;
			SparseMatrix joint_regressor;
		};

		Generator(Configuration&& configuration) :
			vertices_(configuration.vertices), indices_(configuration.indices),
			identity_morph_(configuration.shapedirs), joint_regressor_(configuration.joint_regressor, JointsRegressor::SMPL, JOINT_COUNT),
			pose_morph_(configuration.posedirs), skin_morph_(configuration.skins)
		{
		}
		
		Body operator()(const ShapeCoefficients& betas, const PoseAxisAngleCoefficients& thetas) const;

		/*
			Generate the smpl body.
		*/
		Body operator()() const;
		Body operator()(bool with_normals) const;
		Body operator()(const ShapeCoefficients& betas,
			const PoseEulerCoefficients& thetas) const;
		Body operator()(const ShapeCoefficients& betas, 
			const PoseEulerCoefficients& thetas, bool with_normals) const;

		void CalculateNormals(Body& body) const;

		void ComputeBodyFromShapeJacobian(std::vector<float3>& dshape) const;

		void ComputeBodyFromPoseJacobian(const Body& body, std::vector<float3>& dpose) const;

		// Accessors

		const std::vector<float3>& GetShapeDirs() const
		{
			return identity_morph_.GetShapeDirs();
		}

		const std::vector<Skin>& GetSkins() const
		{
			return skin_morph_.GetSkins();
		}

		const Eigen::Matrix4f(&GetPalette() const) [JOINT_COUNT]
		{
			return skin_morph_.GetPalette();
		}

		const Eigen::Matrix4f(&GetPartTransformations() const) [JOINT_COUNT]
		{
			return skin_morph_.GetPartTransformations();
		}

		const JointsRegressor& GetJointRegressor() const
		{
			return joint_regressor_;
		}

		const SkinMorph& GetSkinMorph() const
		{
			return skin_morph_;
		}

	private:
		const std::vector<float3> vertices_;
		const std::vector<uint> indices_;
		const IdentityMorph identity_morph_;
		const JointsRegressor joint_regressor_;
		const PoseMorph pose_morph_;
		const SkinMorph skin_morph_;
	};
}