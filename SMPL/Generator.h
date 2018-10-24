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
		}

		void operator()(const PoseAxisAngleCoefficients& thetas, 
			const Joints& joints, std::vector<float3>& vertices) const;

		/*
			Applies skinning to the vertices of an smpl body.
			When vertices.size() is bigger than VERTEX_COUNT, the same skinning
			is applied to stride = vertices.size() / VERTEX_COUNT number of vertices.
		*/
		void operator()(const PoseEulerCoefficients& thetas, 
			const Joints& joints, std::vector<float3>& vertices) const;

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

		Generator(Configuration configuration) :
			vertices_(configuration.vertices), indices_(configuration.indices),
			identity_morph_(configuration.shapedirs), joint_regressor_(configuration.joint_regressor, JOINT_COUNT),
			pose_morph_(configuration.posedirs), skin_morph_(configuration.skins)
		{
		}
		
		Body operator()(const ShapeCoefficients& betas, const PoseAxisAngleCoefficients& thetas) const;

		Body operator()() const;

		Body operator()(bool with_normals) const;

		Body operator()(const ShapeCoefficients& betas,
			const PoseEulerCoefficients& thetas) const;

		Body operator()(const ShapeCoefficients& betas, 
			const PoseEulerCoefficients& thetas, bool with_normals) const;

		void CalculateNormals(Body& body) const;

		const std::vector<float3>& GetShapeDirs() const
		{
			return identity_morph_.GetShapeDirs();
		}

		const std::vector<Skin>& GetSkins() const
		{
			return skin_morph_.GetSkins();
		}

		const JointRegressor& GetJointRegressor() const
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
		const JointRegressor joint_regressor_;
		const PoseMorph pose_morph_;
		const SkinMorph skin_morph_;
	};
}