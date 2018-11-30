#pragma once

#include "Definitions.h"
#include "Utils.h"
#include "Generator.h"
#include "Image.h"

namespace smpl
{
	class Silhouette
	{
	public:
		struct Loader
		{
			Loader(const std::string& pattern) :
				image((pattern + std::string(".png")).c_str())
			{
				ReadFloatVectorFromBinary(pattern + ".normals", normals);
				ReadFloatVectorFromBinary(pattern + ".vertex_indices", vertex_indices);
				ReadFloatVectorFromBinary(pattern + ".barycentrics", barycentrics);
			}

			Image image;
			std::vector<float4> normals, vertex_indices, barycentrics;
		};

		Silhouette(Loader loader) :
			image_(loader.image), 
			normals_(loader.normals),
			vertex_indices_(loader.vertex_indices),
			barycentrics_(loader.barycentrics)
		{
		}

		Silhouette(const Image& image, const std::vector<float4>& normals, 
			const std::vector<float4>& vertex_indices, const std::vector<float4>& barycentrics) :
			image_(image),
			normals_(normals),
			vertex_indices_(vertex_indices),
			barycentrics_(barycentrics)
		{
		}

		Silhouette(const Silhouette& other) :
			image_(other.image_),
			normals_(other.normals_),
			vertex_indices_(other.vertex_indices_),
			barycentrics_(other.barycentrics_)
		{
		}

		Silhouette(Silhouette&& other)
		{
			image_ = std::move(other.image_);
			normals_ = std::move(other.normals_);
			vertex_indices_ = std::move(other.vertex_indices_);
			barycentrics_ = std::move(other.barycentrics_);
		}

		Silhouette& operator=(const Silhouette& other)
		{
			Silhouette temp(other);
			std::swap(image_, temp.image_);
			std::swap(normals_, temp.normals_);
			std::swap(normals_, temp.normals_);
			std::swap(normals_, temp.normals_);
			return *this;
		}

		Silhouette& operator=(Silhouette&& other)
		{
			std::swap(image_, other.image_);
			std::swap(normals_, other.normals_);
			std::swap(normals_, other.normals_);
			std::swap(normals_, other.normals_);
			return *this;
		}

		bool operator==(const Silhouette& other) const
		{
			if (
				image_ == other.image_ /*&&
				normals_ == other.normals_ &&
				vertex_indices_ == other.vertex_indices_ &&
				barycentrics_ == other.barycentrics_*/)
				return true;
			return false;
		}

		void Dump(const std::string& pattern)
		{
			image_.SavePNG(pattern + std::string(".png"));
			DumpFloatVectorToBinary(pattern + ".normals", normals_);
			DumpFloatVectorToBinary(pattern + ".vertex_indices", vertex_indices_);
			DumpFloatVectorToBinary(pattern + ".barycentrics", barycentrics_);
		}

		const Image& GetImage() const { return image_; }
		const std::vector<float4>& GetNormals() const { return normals_; }
		const std::vector<float4>& GetVertexIndices() const { return vertex_indices_; }
		const std::vector<float4>& GetBarycentrics() const { return barycentrics_; }

	private:
		Image image_;
		std::vector<float4> normals_, vertex_indices_, barycentrics_;
	};

	class SilhouetteRenderer
	{
	public:
		struct Matrices
		{
			Matrices() :
				view(Eigen::Matrix4f::Identity()),
				view_it(Eigen::Matrix4f::Identity()),
				projection(Eigen::Matrix4f::Identity())
			{
			}

			Eigen::Matrix4f view;
			Eigen::Matrix4f view_it;
			Eigen::Matrix4f projection;
		};

		// pass body to allocate appropriate amounts of memory
		SilhouetteRenderer(const Body& body); 
		
		// pass body to create a silhouette
		Silhouette operator()(const Body& body, const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection) const;
	
	private:

		void SetMatrices(const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection) const;

		// silhouette, normals, vertex indices, barycentric coordinates
		static const int		render_targets_number_ = 4;

		mutable ID3D11Device*			device_ = nullptr;
		mutable ID3D11DeviceContext*	device_context_ = nullptr;

		mutable ID3D11VertexShader*		vertex_shader_ = nullptr;
		mutable ID3D11GeometryShader*	geometry_shader_ = nullptr;
		mutable ID3D11PixelShader*		pixel_shader_ = nullptr;

		mutable ID3D11InputLayout *		input_layout_ = nullptr;
		mutable ID3D11Buffer*			vertex_buffer_ = nullptr;
		mutable ID3D11Buffer*			index_buffer_ = nullptr;
		mutable unsigned int			indices_count_ = 0;

		mutable ID3D11Buffer*			camera_constant_buffer_ = nullptr;
		mutable Matrices				matrices_;

		mutable ID3D11Texture2D*		render_target_textures_[render_targets_number_];
		mutable ID3D11Texture2D*		render_target_staging_textures_[render_targets_number_];
		mutable ID3D11Texture2D*		depth_stencil_texture_;
		mutable ID3D11RenderTargetView* render_target_views_[render_targets_number_];
		mutable ID3D11DepthStencilView* depth_stencil_view_;
		mutable ID3D11DepthStencilState*
										depth_stencil_state_;

#ifdef _DEBUG
		mutable RENDERDOC_API_1_1_2*	rdoc_api_ = nullptr;
#endif
	};
}