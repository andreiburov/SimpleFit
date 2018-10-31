#include "Body.h"

namespace smpl
{
	Body::Body(const std::vector<float3>& v, const std::vector<uint>& i) :
		vertices(v), indices(i)
	{
	}

	Body::Body(const ShapeCoefficients& betas, const PoseEulerCoefficients& thetas,
		const std::vector<float3>& v, const std::vector<uint>& i) :
		betas(betas), thetas(thetas), vertices(v), indices(i)
	{
	}

	Body::Body(const Body& other)
	{
		vertices = other.vertices;
		indices = other.indices;
		deformed_template = other.deformed_template;
		vertices_normals = other.vertices_normals;
		joints = other.joints;
		betas = other.betas;
		thetas = other.thetas;
	}

	Body::Body(Body&& other)
	{
		vertices = std::move(other.vertices);
		indices = std::move(other.indices);
		deformed_template = std::move(other.deformed_template);
		vertices_normals = std::move(other.vertices_normals);
		joints = std::move(other.joints);
		betas = std::move(other.betas);
		thetas = std::move(other.thetas);
	}

	Body& Body::operator=(const Body& other)
	{
		Body temp(other);
		vertices = std::move(temp.vertices);
		indices = std::move(temp.indices);
		deformed_template = std::move(temp.deformed_template);
		vertices_normals = std::move(temp.vertices_normals);
		joints = std::move(temp.joints);
		betas = std::move(temp.betas);
		thetas = std::move(temp.thetas);
		return *this;
	}

	Body& Body::operator=(Body&& other)
	{
		vertices = std::move(other.vertices);
		indices = std::move(other.indices);
		deformed_template = std::move(other.deformed_template);
		vertices_normals = std::move(other.vertices_normals);
		joints = std::move(other.joints);
		betas = std::move(other.betas);
		thetas = std::move(other.thetas);
		return *this;
	}

	bool Body::operator==(const Body& other) const
	{
		if (
			vertices == other.vertices &&
			indices == other.indices &&
			deformed_template == other.deformed_template &&
			vertices_normals == other.vertices_normals &&
			joints == other.joints
			)
			return true;
		else
			return false;
	}

	bool Body::IsEqual(const Body& other, float eps) const
	{
		for (uint i = 0; i < VERTEX_COUNT; i++)
		{
			if (
				(vertices[i].ToEigen() - other.vertices[i].ToEigen()).norm() > eps
				)
				return false;
		}

		return true;
	}

	void Body::Dump(const std::string& filename) const
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

	/*void Body::Draw(Image& image, const Eigen::Matrix3f& intrinsics, const Eigen::Vector3f& scaling, const Eigen::Vector3f& translation) const
	{
		int w = image.GetWidth();
		int h = image.GetHeight();

		RGBTRIPLE white;
		white.rgbtRed = 255;
		white.rgbtGreen = 255;
		white.rgbtBlue = 255;

		for (uint i = 0; i < VERTEX_COUNT; i++)
		{
			Eigen::Vector3f p = intrinsics * (Eigen::Scaling(scaling) * vertices[i].ToEigen() + translation);
			p /= p(2);
			if ((p(0) >= 0) && (p(0) < w) && (p(1) >= 0) && (p(1) < h))
			{
				image[int(p(1))][int(p(0))] = white;
			}
		}
	}*/
}