#include "SilhouetteRenderer.h"

namespace smpl
{
	SilhouetteRenderer::SilhouetteRenderer(const Body& body) : device_(d3d_context.device), device_context_(d3d_context.device_context)
	{
#ifdef _DEBUG
		{
			if (HMODULE mod = LoadLibrary(L"renderdoc.dll"))
			{
				pRENDERDOC_GetAPI RENDERDOC_GetAPI = (pRENDERDOC_GetAPI)GetProcAddress(mod, "RENDERDOC_GetAPI");
				int ret = RENDERDOC_GetAPI(eRENDERDOC_API_Version_1_1_2, (void **)&rdoc_api_);
				assert(ret == 1);
			}

			if (rdoc_api_)
			{
				rdoc_api_->SetCaptureFilePathTemplate("Captures/Example");
				rdoc_api_->StartFrameCapture(device_, nullptr);
				std::cout << "Start Frame Capture" << std::endl;
			}
		}
#endif

		const D3D11_INPUT_ELEMENT_DESC vertex_layout_desc[] =
		{
			{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0,  0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
			{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0,  12, D3D11_INPUT_PER_VERTEX_DATA, 0 }
		};

		{
			std::vector<byte> vertex_shader = ReadShaderFromCSO("SilhouetteVS.cso");
			VALIDATE(device_->CreateVertexShader(vertex_shader.data(), vertex_shader.size(),
				nullptr, &vertex_shader_), "Could not create VertexShader");

			VALIDATE(device_->CreateInputLayout(vertex_layout_desc, ARRAYSIZE(vertex_layout_desc),
				vertex_shader.data(), vertex_shader.size(), &input_layout_), "Could not create InputLayout");

			std::vector<byte> geometry_shader = ReadShaderFromCSO("SilhouetteGS.cso");
			VALIDATE(device_->CreateGeometryShader(geometry_shader.data(), geometry_shader.size(),
				nullptr, &geometry_shader_), "Could not create GeometryShader");

			std::vector<byte> pixel_shader = ReadShaderFromCSO("SilhouettePS.cso");
			VALIDATE(device_->CreatePixelShader(pixel_shader.data(), pixel_shader.size(),
				nullptr, &pixel_shader_), "Could not create PixelShader");
		}

		{
			D3D11_BUFFER_DESC vertex_buffer_desc = { 0 };
			vertex_buffer_desc.ByteWidth = sizeof(float6) * (unsigned int)smpl::VERTEX_COUNT;
			vertex_buffer_desc.Usage = D3D11_USAGE_DYNAMIC;
			vertex_buffer_desc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
			vertex_buffer_desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
			vertex_buffer_desc.MiscFlags = 0;
			vertex_buffer_desc.StructureByteStride = 0;

			D3D11_SUBRESOURCE_DATA vertex_buffer_data;
			if (body.vertices_normals.size() == 0)
				MessageBoxA(nullptr, "Body normals are not calculated", "Error", MB_OK);
			vertex_buffer_data.pSysMem = body.vertices_normals.data();
			vertex_buffer_data.SysMemPitch = 0;
			vertex_buffer_data.SysMemSlicePitch = 0;

			VALIDATE(device_->CreateBuffer(&vertex_buffer_desc, &vertex_buffer_data,
				&vertex_buffer_), "Could not create VertexBuffer");
		}

		indices_count_ = (unsigned int)body.indices.size();

		{
			D3D11_BUFFER_DESC index_buffer_desc;
			index_buffer_desc.ByteWidth = sizeof(smpl::uint) * (unsigned int)indices_count_;
			index_buffer_desc.Usage = D3D11_USAGE_DEFAULT;
			index_buffer_desc.BindFlags = D3D11_BIND_INDEX_BUFFER;
			index_buffer_desc.CPUAccessFlags = 0;
			index_buffer_desc.MiscFlags = 0;
			index_buffer_desc.StructureByteStride = 0;

			D3D11_SUBRESOURCE_DATA index_buffer_data;
			index_buffer_data.pSysMem = body.indices.data();
			index_buffer_data.SysMemPitch = 0;
			index_buffer_data.SysMemSlicePitch = 0;

			VALIDATE(device_->CreateBuffer(&index_buffer_desc, &index_buffer_data, &index_buffer_),
				"Could not create IndexBuffer");
		}

		// Create ConstantBuffer for camera matrices
		{
			D3D11_BUFFER_DESC constantBufferDesc = { 0 };
			constantBufferDesc.ByteWidth = sizeof(matrices_);
			constantBufferDesc.Usage = D3D11_USAGE_DEFAULT;
			constantBufferDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
			constantBufferDesc.CPUAccessFlags = 0;
			constantBufferDesc.MiscFlags = 0;
			constantBufferDesc.StructureByteStride = 0;

			VALIDATE(device_->CreateBuffer(&constantBufferDesc, nullptr, &camera_constant_buffer_),
				"Could not create CameraConstantBuffer");
		}

		// Create a texture and the render target attached to it
		{
			D3D11_TEXTURE2D_DESC textureDesc;

			ZeroMemory(&textureDesc, sizeof(textureDesc));

			textureDesc.Width = IMAGE_WIDTH;
			textureDesc.Height = IMAGE_HEIGHT;
			textureDesc.MipLevels = 1;
			textureDesc.ArraySize = 1;
			textureDesc.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;//DXGI_FORMAT_R32_UINT;//DXGI_FORMAT_B8G8R8A8_UNORM;//DXGI_FORMAT_R16G16B16A16_UNORM;
			textureDesc.SampleDesc.Count = 1;
			textureDesc.MiscFlags = 0;
			textureDesc.Usage = D3D11_USAGE_DEFAULT;
			textureDesc.BindFlags = D3D11_BIND_RENDER_TARGET;
			textureDesc.CPUAccessFlags = 0;

			for (int i = 0; i < render_targets_number_; i++)
			{
				device_->CreateTexture2D(&textureDesc, NULL, &render_target_textures_[i]);
			}

			textureDesc.Usage = D3D11_USAGE_STAGING;
			textureDesc.BindFlags = 0;
			textureDesc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;

			for (int i = 0; i < render_targets_number_; i++)
			{
				device_->CreateTexture2D(&textureDesc, NULL, &render_target_staging_textures_[i]);
			}

			D3D11_RENDER_TARGET_VIEW_DESC renderTargetViewDesc;
			renderTargetViewDesc.Format = textureDesc.Format;
			renderTargetViewDesc.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE2D;
			renderTargetViewDesc.Texture2D.MipSlice = 0;

			// Create the render target view.
			for (int i = 0; i < render_targets_number_; i++) 
			{
				device_->CreateRenderTargetView(render_target_textures_[i], &renderTargetViewDesc, &render_target_views_[i]);
			}
		}

		// set viewport
		{
			D3D11_VIEWPORT vp;
			vp.Width = (FLOAT)IMAGE_WIDTH;
			vp.Height = (FLOAT)IMAGE_HEIGHT;
			vp.MinDepth = 0.0f;
			vp.MaxDepth = 1.0f;
			vp.TopLeftX = 0;
			vp.TopLeftY = 0;
			device_context_->RSSetViewports(1, &vp);
		}

		// set rasterizer state
		{
			ID3D11RasterizerState* rasterizerState;

			D3D11_RASTERIZER_DESC rasterizerStateDesc;
			rasterizerStateDesc.FillMode = D3D11_FILL_SOLID;
			rasterizerStateDesc.CullMode = D3D11_CULL_BACK;
			rasterizerStateDesc.FrontCounterClockwise = false;
			rasterizerStateDesc.DepthBias = false;
			rasterizerStateDesc.DepthBiasClamp = 0;
			rasterizerStateDesc.SlopeScaledDepthBias = 0;
			rasterizerStateDesc.DepthClipEnable = false;
			rasterizerStateDesc.ScissorEnable = false;
			rasterizerStateDesc.MultisampleEnable = false;
			rasterizerStateDesc.AntialiasedLineEnable = false;
			device_->CreateRasterizerState(&rasterizerStateDesc, &rasterizerState);

			device_context_->RSSetState(rasterizerState);
		}
	}

	Silhouette SilhouetteRenderer::operator()(const Body& body, const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection)
	{
		SetMatrices(view, projection);

		const float clear_color[] = { 0.0f, 0.0f, 0.0f, 1.0f };
		for (int i = 0; i < render_targets_number_; i++)
		{
			device_context_->ClearRenderTargetView(render_target_views_[i], clear_color);
		}

		// Update the vertex buffer
		{
			D3D11_MAPPED_SUBRESOURCE mapped_resource;
			if (SUCCEEDED(device_context_->Map(vertex_buffer_, 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped_resource)))
			{
				memcpy(mapped_resource.pData, body.vertices_normals.data(), body.vertices_normals.size()*sizeof(float6));
				device_context_->Unmap(vertex_buffer_, 0);
			}
		}

		device_context_->UpdateSubresource(camera_constant_buffer_, 0, nullptr, &matrices_, 0, 0);
		device_context_->IASetInputLayout(input_layout_);

		// Set the vertex and index buffers, and specify the way they define geometry
		UINT stride = sizeof(float6);
		UINT offset = 0;
		device_context_->IASetVertexBuffers(0, 1, &vertex_buffer_, &stride, &offset);
		device_context_->IASetIndexBuffer(index_buffer_, DXGI_FORMAT_R32_UINT, 0);
		device_context_->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

		// Set the vertex and pixel shader stage state
		device_context_->VSSetShader(vertex_shader_, nullptr, 0);
		device_context_->GSSetShader(geometry_shader_, nullptr, 0);
		device_context_->GSSetConstantBuffers(0, 1, &camera_constant_buffer_);
		device_context_->PSSetShader(pixel_shader_, nullptr, 0);
		device_context_->OMSetRenderTargets(render_targets_number_, render_target_views_, nullptr);

		device_context_->DrawIndexed(indices_count_, 0, 0);

		std::vector<float4> copied_textures[render_targets_number_];
		int texture_size = IMAGE_HEIGHT * IMAGE_WIDTH;
		for (int i = 0; i < render_targets_number_; i++)
		{
			device_context_->CopyResource(render_target_staging_textures_[i], render_target_textures_[i]);
			device_context_->Flush();

			copied_textures[i].resize(texture_size);

			D3D11_MAPPED_SUBRESOURCE mapped_resource;
			if (SUCCEEDED(device_context_->Map(render_target_staging_textures_[i], 0, D3D11_MAP_READ, 0, &mapped_resource)))
			{
				memcpy(copied_textures[i].data(), mapped_resource.pData, texture_size*sizeof(float4));
				device_context_->Unmap(render_target_staging_textures_[i], 0);
			}
		}

#ifdef _DEBUG
		{	
			if (rdoc_api_)
			{
				rdoc_api_->EndFrameCapture(device_, nullptr);
				std::cout << "End Frame Capture" << std::endl;
			}
		}
#endif

		Image silhouette_image;

#ifdef USE_24_BITS_PER_PIXEL
		for (int j = 0; j < IMAGE_HEIGHT; j++)
		{
			for (int i = 0; i < IMAGE_WIDTH; i++)
			{
				float4 pixel(copied_textures[0][j*IMAGE_WIDTH + i]);
				silhouette_image[j][i].r() = (pixel[0] < 1e-8f) ? 0U : static_cast<BYTE>(pixel[0] * 255 + 0.5f);
				silhouette_image[j][i].g() = (pixel[1] < 1e-8f) ? 0U : static_cast<BYTE>(pixel[1] * 255 + 0.5f);
				silhouette_image[j][i].b() = (pixel[2] < 1e-8f) ? 0U : static_cast<BYTE>(pixel[2] * 255 + 0.5f);

				/*float4 normal(copied_textures[1][j*IMAGE_WIDTH + i]);
				float4 vertex_indices(copied_textures[2][j*IMAGE_WIDTH + i]);
				float4 barycentric(copied_textures[3][j*IMAGE_WIDTH + i]);*/

				/*if (pixel[0] > 0.00001f)
				{
					std::cout << "x " << i << ", y " << j << std::endl;
				}*/
			}
		}
#endif
#ifdef USE_32_BITS_PER_PIXEL
		//memcpy(&silhouette_image[0][0], copied_texture, IMAGE_WIDTH*IMAGE_HEIGHT*4*sizeof(BYTE));
		silhouette_image = FreeImage_ConvertFromRawBits((BYTE*)copied_textures[0].data(), IMAGE_WIDTH, IMAGE_HEIGHT,
			IMAGE_WIDTH*4, 32, FI_RGBA_RED_MASK, FI_RGBA_GREEN_MASK, FI_RGBA_BLUE_MASK, TRUE);
#endif

		Silhouette silhouette(silhouette_image, copied_textures[1], copied_textures[2], copied_textures[3]);
 		return silhouette;
	}

	void SilhouetteRenderer::SetMatrices(const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection)
	{
		matrices_.view = view;
		matrices_.view_it = view.inverse().transpose();
		matrices_.projection = projection;
	}
}