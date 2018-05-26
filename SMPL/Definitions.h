#pragma once

#include <fstream>
#include "DXUtils.h"
#include "Image.h"

namespace smpl
{

	const int BETA_COUNT = 10;
	const int THETA_COUNT = 24;
	const int THETA_COUNT_WITHOUT_PARENT = THETA_COUNT - 1;
	const int THETA_MATRIX_COMPONENT_COUNT = 207;
	const int VERTEX_COUNT = 6890;
	const int VERTEX_BUFFER_SIZE = VERTEX_COUNT * sizeof(float3);
	const int FACE_COUNT = 13776;
	const int MAX_TRIANGLES_PER_VERTEX = 9;
	const int POSEDIRS_PER_VERTEX = 1426230;	// (6890 * 207) * 3
	const int SHAPEDIRS_PER_VERTEX = 68900;	// (6890 * 10) * 3

	const int HIP_CENTER = 0;
	const int HIP_RIGHT = 1;
	const int HIP_LEFT = 2;
	const int STOMACH = 3;
	const int KNEE_RIGHT = 4;
	const int KNEE_LEFT = 5;
	const int BACKBONE = 6;
	const int ANKLE_RIGHT = 7;
	const int ANKLE_LEFT = 8;
	const int CHEST = 9;
	const int FOOT_RIGHT = 10;
	const int FOOT_LEFT = 11;
	const int SHOULDER_CENTER = 12;
	const int PECK_RIGHT = 13;
	const int PECK_LEFT = 14;
	const int CHIN = 15;
	const int SHOULDER_RIGHT = 16;
	const int SHOULDER_LEFT = 17;
	const int ELBOW_RIGHT = 18;
	const int ELBOW_LEFT = 19;
	const int WRIST_RIGHT = 20;
	const int WRIST_LEFT = 21;
	const int HAND_RIGHT = 22;
	const int HAND_LEFT = 23;
	const int JOINT_COUNT = 24;

	const int COCO_JOINT_COUNT = 18;

	const int PARENT_INDEX[JOINT_COUNT] = {
		-1,
		HIP_CENTER,
		HIP_CENTER,
		HIP_CENTER,
		HIP_RIGHT,
		HIP_LEFT,
		STOMACH,
		KNEE_RIGHT,
		KNEE_LEFT,
		BACKBONE,
		ANKLE_RIGHT,
		ANKLE_LEFT,
		CHEST,
		CHEST,
		CHEST,
		SHOULDER_CENTER,
		PECK_RIGHT,
		PECK_LEFT,
		SHOULDER_RIGHT,
		SHOULDER_LEFT,
		ELBOW_RIGHT,
		ELBOW_LEFT,
		WRIST_RIGHT,
		WRIST_LEFT
	};

	struct ShapeCoefficients
	{
		float betas[BETA_COUNT];
		float operator [](int i) const { return betas[i]; }
		float& operator [](int i) { return betas[i]; }
	};

	struct PoseCoefficients
	{
		float3 thetas[THETA_COUNT];
		float3 operator [](int i) const { return thetas[i]; }
		float3& operator [](int i) { return thetas[i]; }
		
		// access componentwise
		float operator ()(int i) const { return thetas[i/3].data[i%3]; }
		float& operator ()(int i) { return thetas[i/3].data[i%3]; }
	};

	struct Skin
	{
		float4 weight;
		int4 joint_index;
	};

	typedef Eigen::MatrixXf Joints;

	struct Body
	{
		std::vector<float3> vertices;
		std::vector<uint> indices;

		Body()
		{
		}

		Body(const std::vector<float3>& v, const std::vector<uint>& i) :
			vertices(v), indices(i)
		{
		}

		Body(const Body& other)
		{
			vertices = other.vertices;
			indices = other.indices;
		}

		Body(Body&& other)
		{
			vertices = std::move(other.vertices);
			indices = std::move(other.indices);
		}

		/*Body& operator=(Body other)
		{
			vertices.swap(other.vertices);
			indices.swap(other.indices);
			return *this;
		}*/

		void Draw(Image& image, const Eigen::Matrix3f& intrinsics, const float3& scaling, const float3& translation) const
		{
			int w = image.GetWidth();
			int h = image.GetHeight();

			RGBTRIPLE white;
			white.rgbtRed = 255;
			white.rgbtGreen = 255;
			white.rgbtBlue = 255;

			for (uint i = 0; i < VERTEX_COUNT; i++)
			{
				Eigen::Vector3f p = intrinsics * (Eigen::Scaling(scaling.ToEigen()) * vertices[i].ToEigen() + translation.ToEigen());
				p /= p(2);
				if ((p(0) >= 0) && (p(0) < w) && (p(1) >= 0) && (p(1) < h))
				{
					image[int(p(1))][int(p(0))] = white;
				}
			}
		}

		void Dump(const std::string& filename) const
		{
			std::ofstream file(filename, std::ios::out);
			for (auto& v : vertices)
			{
				file << "v " << v.x << " " << v.y << " " << v.z << "\n";
			}
			for (int i = 0; i < indices.size(); i++)
			{
				if (i % 3 == 0)
				{
					file << "f " << indices[i] + 1 << " ";
				}
				else if (i % 3 == 1)
				{
					file << indices[i] + 1 << " ";
				}
				else
				{
					file << indices[i] + 1 << "\n";
				}
			}
		}
	};
}