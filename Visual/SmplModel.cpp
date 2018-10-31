#include "SmplModel.h"
#include "Utils.h"

void SmplModel::Initialize(ID3D11Device* device, ID3D11DeviceContext* device_context)
{
	device_context_ = device_context;

	const D3D11_INPUT_ELEMENT_DESC vertex_layout_desc[] =
	{
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0,  0, D3D11_INPUT_PER_VERTEX_DATA, 0 }
	};

	{
		std::vector<byte> vertex_shader = readShaderFromCSO("CubeVS.cso");
		VALIDATE(device->CreateVertexShader(vertex_shader.data(), vertex_shader.size(),
			nullptr, &vertex_shader_), L"Could not create VertexShader");

		VALIDATE(device->CreateInputLayout(vertex_layout_desc, ARRAYSIZE(vertex_layout_desc),
			vertex_shader.data(), vertex_shader.size(), &input_layout_), L"Could not create InputLayout");

		std::vector<byte> geometry_shader = readShaderFromCSO("CubeGS.cso");
		VALIDATE(device->CreateGeometryShader(geometry_shader.data(), geometry_shader.size(),
			nullptr, &geometry_shader_), L"Could not create GeometryShader");

		std::vector<byte> pixel_shader = readShaderFromCSO("CubePS.cso");
		VALIDATE(device->CreatePixelShader(pixel_shader.data(), pixel_shader.size(),
			nullptr, &pixel_shader_), L"Could not create PixelShader");
	}

	// Generate Template SMPL Body
	Generate();
	{
		D3D11_BUFFER_DESC vertex_buffer_desc = { 0 };
		vertex_buffer_desc.ByteWidth = sizeof(smpl::float3) * (unsigned int)smpl::VERTEX_COUNT;
		vertex_buffer_desc.Usage = D3D11_USAGE_DYNAMIC;
		vertex_buffer_desc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
		vertex_buffer_desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
		vertex_buffer_desc.MiscFlags = 0;
		vertex_buffer_desc.StructureByteStride = 0;

		D3D11_SUBRESOURCE_DATA vertex_buffer_data;
		vertex_buffer_data.pSysMem = body_.vertices.data();
		vertex_buffer_data.SysMemPitch = 0;
		vertex_buffer_data.SysMemSlicePitch = 0;

		VALIDATE(device->CreateBuffer(&vertex_buffer_desc, &vertex_buffer_data,
			&vertex_buffer_), L"Could not create VertexBuffer");
	}

	indices_count_ = (unsigned int)body_.indices.size();

	{
		D3D11_BUFFER_DESC index_buffer_desc;
		index_buffer_desc.ByteWidth = sizeof(smpl::uint) * (unsigned int)indices_count_;
		index_buffer_desc.Usage = D3D11_USAGE_DEFAULT;
		index_buffer_desc.BindFlags = D3D11_BIND_INDEX_BUFFER;
		index_buffer_desc.CPUAccessFlags = 0;
		index_buffer_desc.MiscFlags = 0;
		index_buffer_desc.StructureByteStride = 0;

		D3D11_SUBRESOURCE_DATA index_buffer_data;
		index_buffer_data.pSysMem = body_.indices.data();
		index_buffer_data.SysMemPitch = 0;
		index_buffer_data.SysMemSlicePitch = 0;

		VALIDATE(device->CreateBuffer(&index_buffer_desc, &index_buffer_data, &index_buffer_),
			L"Could not create IndexBuffer");
	}

	// Create ConstantBuffer for camera matrices
	{
		D3D11_BUFFER_DESC constantBufferDesc = { 0 };
		constantBufferDesc.ByteWidth = camera_.GetDataSize();
		constantBufferDesc.Usage = D3D11_USAGE_DEFAULT;
		constantBufferDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
		constantBufferDesc.CPUAccessFlags = 0;
		constantBufferDesc.MiscFlags = 0;
		constantBufferDesc.StructureByteStride = 0;

		VALIDATE(device->CreateBuffer(&constantBufferDesc, nullptr, &camera_constant_buffer_),
			L"Could not create CameraConstantBuffer");
	}
}

void SmplModel::Draw()
{
	device_context_->UpdateSubresource(camera_constant_buffer_, 0, nullptr, camera_.GetDataPointer(), 0, 0);
	device_context_->IASetInputLayout(input_layout_);

	// Set the vertex and index buffers, and specify the way they define geometry
	UINT stride = sizeof(smpl::float3);
	UINT offset = 0;
	device_context_->IASetVertexBuffers(0, 1, &vertex_buffer_, &stride, &offset);
	device_context_->IASetIndexBuffer(index_buffer_, DXGI_FORMAT_R32_UINT, 0);
	device_context_->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

	// Set the vertex and pixel shader stage state
	device_context_->VSSetShader(vertex_shader_, nullptr, 0);
	device_context_->GSSetShader(geometry_shader_, nullptr, 0);
	device_context_->GSSetConstantBuffers(0, 1, &camera_constant_buffer_);
	device_context_->PSSetShader(pixel_shader_, nullptr, 0);

	device_context_->DrawIndexed(indices_count_, 0, 0);
}

void SmplModel::Clear()
{
	SAFE_RELEASE(vertex_shader_);
	SAFE_RELEASE(pixel_shader_);
	SAFE_RELEASE(geometry_shader_);
	SAFE_RELEASE(input_layout_);
	SAFE_RELEASE(vertex_buffer_);
	SAFE_RELEASE(index_buffer_);
	SAFE_RELEASE(camera_constant_buffer_);
}

void SmplModel::Dump(const std::string& filename)
{
	body_.Dump(filename);
}

void SmplModel::Generate(smpl::ShapeCoefficients& shape, smpl::PoseAxisAngleCoefficients& pose)
{
	body_ = generator_(shape, pose);
	UpdateBodyOnGPU();
}

void SmplModel::Generate(smpl::ShapeCoefficients& shape, smpl::PoseEulerCoefficients& pose)
{
	body_ = generator_(shape, pose);
	UpdateBodyOnGPU();
}

void SmplModel::Generate()
{
	smpl::ShapeCoefficients shape;
	smpl::PoseAxisAngleCoefficients pose;
	
	body_ = generator_(shape, pose);
}

void SmplModel::UpdateBodyOnGPU()
{
	D3D11_MAPPED_SUBRESOURCE resource;
	device_context_->Map(vertex_buffer_, 0, D3D11_MAP_WRITE_DISCARD, 0, &resource);
	memcpy(resource.pData, body_.vertices.data(), sizeof(smpl::float3) * (unsigned int)smpl::VERTEX_COUNT);
	device_context_->Unmap(vertex_buffer_, 0);
}