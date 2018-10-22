#pragma once

#include "Definitions.h"
#include "Utils.h"
#include "Generator.h"
#include "Image.h"

namespace smpl
{
	class SilhouetteMaker
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

		SilhouetteMaker(const Body& body); // pass body to allocate appropriate amounts of memory
		Image operator()(const Body& body, const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection); // pass body to create a silhouette

	private:

		void SetMatrices(const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection);

		// silhouette, normals, vertex indices, barycentric coordinates
		static const int		render_targets_number_ = 4;

		ID3D11Device*			device_ = nullptr;
		ID3D11DeviceContext*	device_context_ = nullptr;

		ID3D11VertexShader*		vertex_shader_ = nullptr;
		ID3D11GeometryShader*	geometry_shader_ = nullptr;
		ID3D11PixelShader*		pixel_shader_ = nullptr;

		ID3D11InputLayout *		input_layout_ = nullptr;
		ID3D11Buffer*			vertex_buffer_ = nullptr;
		ID3D11Buffer*			index_buffer_ = nullptr;
		unsigned int			indices_count_ = 0;

		ID3D11Buffer*			camera_constant_buffer_ = nullptr;
		Matrices				matrices_;

		ID3D11Texture2D*		render_target_textures_[render_targets_number_];
		ID3D11Texture2D*		render_target_staging_textures_[render_targets_number_];
		ID3D11RenderTargetView* render_target_views_[render_targets_number_];

#ifdef _DEBUG
		RENDERDOC_API_1_1_2*	rdoc_api_ = nullptr;
#endif
	};
}