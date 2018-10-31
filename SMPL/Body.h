#pragma once
#include "Definitions.h"

namespace smpl
{
	struct Body
	{
		// main components of the smpl body
		std::vector<float3> vertices;			// mesh vertices
		std::vector<uint> indices;				// mesh indices zero-based

		// produced by smpl body generator
		std::vector<float3> deformed_template;	// shaped and posed mesh before the skinning
		std::vector<float6> vertices_normals;	// mesh vertices and normals interleaved
		Joints joints;							// mesh joints for skinning

		// parameters used by generator
		ShapeCoefficients betas;				// shape parameters
		PoseEulerCoefficients thetas;			// pose parameters

		Body() = default;

		Body(const std::vector<float3>& v, const std::vector<uint>& i);

		Body(const ShapeCoefficients& betas, const PoseEulerCoefficients& thetas,
			const std::vector<float3>& v, const std::vector<uint>& i);

		Body(const Body& other);

		Body(Body&& other);

		Body& operator=(const Body& other);

		Body& operator=(Body&& other);

		bool operator==(const Body& other) const;

		bool IsEqual(const Body& other, float eps) const;

		void Dump(const std::string& filename) const;

		/*void Draw(Image& image, const Eigen::Matrix3f& intrinsics, const Eigen::Vector3f& scaling, const Eigen::Vector3f& translation) const;*/
	};
} // smpl