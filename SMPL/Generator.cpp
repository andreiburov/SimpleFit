#include <iostream>
#include <iterator>
#include <fstream>
#include <ole2.h>
#include <wrl/client.h>
#include <DirectXMath.h>
#include "Generator.h"

namespace smpl {

	Generator::Generator(std::string objFilename, std::string posedirsFilename) {
		m_posedirs = new float[POSEDIRS_ELEMENTS_COUNT];
		readConfigs(objFilename, posedirsFilename);
		initD3D("VertexShader.cso");
	}

	Generator::~Generator() {
		delete[] m_posedirs;
		cleanupD3D();
	}

	void Generator::initD3D(const std::string& vertexShaderFilename)
	{
		const D3D11_INPUT_ELEMENT_DESC vertexLayoutDesc[] =
		{
			{ "WEIGHTS", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
			{ "JOINT_INDICES", 0, DXGI_FORMAT_R32G32B32A32_UINT, 0, 16, D3D11_INPUT_PER_VERTEX_DATA, 0 },
			{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0,  32, D3D11_INPUT_PER_VERTEX_DATA, 0 },
			{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 44, D3D11_INPUT_PER_VERTEX_DATA, 0 }
		};

		ID3D11Device* pd3dDevice = m_renderer.getDevice();

		std::vector<byte> vertexShader = readShaderFromCSO(vertexShaderFilename);
		VALIDATE(pd3dDevice->CreateVertexShader(vertexShader.data(), vertexShader.size(),
			nullptr, &m_pVertexShader), L"Could not create VertexShader");

		VALIDATE(pd3dDevice->CreateInputLayout(vertexLayoutDesc, ARRAYSIZE(vertexLayoutDesc),
			vertexShader.data(), vertexShader.size(), &m_pInputLayout), L"Could not create InputLayout");

		D3D11_BUFFER_DESC vertexBufferDesc = { 0 };
		vertexBufferDesc.ByteWidth = sizeof(Vertex) * (unsigned int)m_vertices.size();
		vertexBufferDesc.Usage = D3D11_USAGE_DEFAULT;
		vertexBufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
		vertexBufferDesc.CPUAccessFlags = 0;
		vertexBufferDesc.MiscFlags = 0;
		vertexBufferDesc.StructureByteStride = 0;

		D3D11_SUBRESOURCE_DATA vertexBufferData;
		vertexBufferData.pSysMem = m_vertices.data();
		vertexBufferData.SysMemPitch = 0;
		vertexBufferData.SysMemSlicePitch = 0;

		VALIDATE(pd3dDevice->CreateBuffer(&vertexBufferDesc, &vertexBufferData,
			&m_pVertexBuffer), L"Could not create VertexBuffer");

		// create StreamOutput structures
		// https://fgiesen.wordpress.com/2011/08/14/a-trip-through-the-graphics-pipeline-2011-part-11/
		{
			const D3D11_SO_DECLARATION_ENTRY streamOutputLayout[] =
			{
				{ 0, "SV_POSITION", 0, 0, 3, 0 }
			};
			//streamOutputLayout[0].Stream = 

			UINT stride = sizeof(streamOutputLayout[0]);

			VALIDATE(pd3dDevice->CreateGeometryShaderWithStreamOutput(vertexShader.data(), vertexShader.size(), streamOutputLayout, ARRAYSIZE(streamOutputLayout),
				NULL, 0, D3D11_SO_NO_RASTERIZED_STREAM, NULL, &m_pGeometryShader), L"Can not create GS with SO");
				
			D3D11_BUFFER_DESC soDesc;
			soDesc.BindFlags = D3D11_BIND_STREAM_OUTPUT;
			soDesc.ByteWidth = m_vertices.size() * sizeof(DirectX::XMFLOAT3);
			soDesc.CPUAccessFlags = 0;
			soDesc.Usage = D3D11_USAGE_DEFAULT;
			soDesc.MiscFlags = 0;
			soDesc.StructureByteStride = 0;

			VALIDATE(pd3dDevice->CreateBuffer(&soDesc, NULL, &m_pStreamOutBuffer), L"Can not create StreamOutputBuffer");

			// One can not map the D3D11_USAGE_DEFAULT & D3D11_CPU_ACCESS_READ can not be bound to SO

			soDesc.BindFlags = 0;
			soDesc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
			soDesc.Usage = D3D11_USAGE_STAGING;

			VALIDATE(pd3dDevice->CreateBuffer(&soDesc, NULL, &m_pStagingStreamOutBuffer), L"Can not create StagingStreamOutputBuffer");
		}

		/*// create ConstantBuffer for linear blend skinning
		{
			D3D11_BUFFER_DESC constantBufferDesc = { 0 };
			constantBufferDesc.ByteWidth = m_LBS.getByteWidth();
			constantBufferDesc.Usage = D3D11_USAGE_DEFAULT;
			constantBufferDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
			constantBufferDesc.CPUAccessFlags = 0;
			constantBufferDesc.MiscFlags = 0;
			constantBufferDesc.StructureByteStride = 0;

			VALIDATE(pd3dDevice->CreateBuffer(&constantBufferDesc, nullptr, &m_pLBSConstantBuffer),
				L"Could not create HierarchyConstantBuffer");
		}

		// Create Pose SRV for SMPL
		{
			D3D11_BUFFER_DESC constantBufferDesc = { 0 };
			constantBufferDesc.ByteWidth = m_Pose.getByteWidth();
			constantBufferDesc.Usage = D3D11_USAGE_DYNAMIC;
			constantBufferDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
			constantBufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
			constantBufferDesc.MiscFlags = 0;
			constantBufferDesc.StructureByteStride = 0;

			VALIDATE(pd3dDevice->CreateBuffer(&constantBufferDesc, nullptr, &m_pPoseBuffer),
				L"Could not create Pose Buffer");

			D3D11_SHADER_RESOURCE_VIEW_DESC srvDesc;
			srvDesc.Format = DXGI_FORMAT_R32_FLOAT;
			srvDesc.ViewDimension = D3D_SRV_DIMENSION_BUFFER;
			srvDesc.Buffer.FirstElement = 0;
			srvDesc.Buffer.NumElements = POSEDIRS_COUNT;

			VALIDATE(pd3dDevice->CreateShaderResourceView(m_pPoseBuffer, &srvDesc, &m_pPoseSRV),
				L"Could not create Pose SRV");
		}

		// Create Posedirs SRV for SMPL
		{
			D3D11_BUFFER_DESC posedirsBufferDesc = { 0 };
			posedirsBufferDesc.ByteWidth = sizeof(float)* VERTEX_COUNT * POSEDIRS_COUNT * 3;
			posedirsBufferDesc.Usage = D3D11_USAGE_IMMUTABLE;
			posedirsBufferDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
			posedirsBufferDesc.CPUAccessFlags = 0;
			posedirsBufferDesc.MiscFlags = 0;
			posedirsBufferDesc.StructureByteStride = 0;

			D3D11_SUBRESOURCE_DATA posedirsBufferData;
			posedirsBufferData.pSysMem = m_posedirs;
			posedirsBufferData.SysMemPitch = 0;
			posedirsBufferData.SysMemSlicePitch = 0;

			Microsoft::WRL::ComPtr<ID3D11Buffer> posedirsBuffer;

			VALIDATE(pd3dDevice->CreateBuffer(&posedirsBufferDesc, &posedirsBufferData, posedirsBuffer.GetAddressOf()),
				L"Could not create Posedirs Buffer");

			D3D11_SHADER_RESOURCE_VIEW_DESC srvDesc;
			srvDesc.Format = DXGI_FORMAT_R32G32B32_FLOAT;
			srvDesc.ViewDimension = D3D_SRV_DIMENSION_BUFFER;
			srvDesc.Buffer.FirstElement = 0;
			srvDesc.Buffer.NumElements = VERTEX_COUNT * POSEDIRS_COUNT;

			VALIDATE(pd3dDevice->CreateShaderResourceView(posedirsBuffer.Get(), &srvDesc, &m_pPosedirsSRV),
				L"Could not create Posedirs SRV");
		}*/
	}

	void Generator::cleanupD3D() 
	{
		SAFE_RELEASE(m_pVertexShader);
		SAFE_RELEASE(m_pInputLayout);
		SAFE_RELEASE(m_pVertexBuffer);
		SAFE_RELEASE(m_pGeometryShader);
		SAFE_RELEASE(m_pStreamOutBuffer);
		SAFE_RELEASE(m_pStagingStreamOutBuffer);
		SAFE_RELEASE(m_pLBSConstantBuffer);
		SAFE_RELEASE(m_pPoseBuffer);
		SAFE_RELEASE(m_pPoseSRV);
		SAFE_RELEASE(m_pPosedirsSRV);
	}

	void Generator::run(ShapeCoefficients& shape, PoseCoefficients& pose, Body& body) 
	{
		std::copy(m_indices.begin(), m_indices.end(), back_inserter(body.getIndices()));

		ID3D11DeviceContext* pd3dDeviceContext = m_renderer.getDeviceContext();
		
		// update LBS
		// udpate POSEDIRS

		pd3dDeviceContext->IASetInputLayout(m_pInputLayout);
		UINT stride = sizeof(Vertex);
		UINT offset = 0;
		pd3dDeviceContext->IASetVertexBuffers(0, 1, &m_pVertexBuffer, &stride, &offset);
		pd3dDeviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST);
		pd3dDeviceContext->VSSetShader(m_pVertexShader, nullptr, 0);
		pd3dDeviceContext->GSSetShader(m_pGeometryShader, nullptr, 0);
		pd3dDeviceContext->SOSetTargets(1, &m_pStreamOutBuffer, &offset);

		pd3dDeviceContext->Draw(m_vertices.size(), 0);
		pd3dDeviceContext->CopyResource(m_pStagingStreamOutBuffer, m_pStreamOutBuffer);
		pd3dDeviceContext->Flush();

		D3D11_MAPPED_SUBRESOURCE data;
		if (SUCCEEDED(pd3dDeviceContext->Map(m_pStagingStreamOutBuffer, 0, D3D11_MAP_READ, 0, &data)))
		{
			// SO data is unpacked
			std::vector<DirectX::XMFLOAT3>& vertices = body.getVertices();
			vertices.resize(m_vertices.size());
			memcpy(vertices.data(), data.pData, m_vertices.size() * sizeof(DirectX::XMFLOAT3));
			pd3dDeviceContext->Unmap(m_pStagingStreamOutBuffer, 0);
		}
	}

	void Generator::readConfigs(std::string& objFilename, std::string& posedirsFilename)
	{
		std::ifstream objFile(objFilename, std::ios::in);
		if (!objFile)
		{
			std::cerr << "[ERROR] Can not open the obj file \"" << objFilename << "\"\n";
		}

		m_vertices.reserve(VERTEX_COUNT);
		m_indices.reserve(FACE_COUNT * 3);

		int i = 0;

		while (objFile)
		{
			std::string type;
			objFile >> type;
			if (type.compare("v") == 0)
			{
				float x, y, z;
				objFile >> x >> y >> z;
				m_vertices.push_back(Vertex(x, y, z));
			}
			else if (type.compare("f") == 0)
			{
				unsigned short x, y, z;
				objFile >> x >> y >> z;
				m_indices.push_back(x - 1);
				m_indices.push_back(y - 1);
				m_indices.push_back(z - 1);
			}
			else if (type.compare("#w") == 0)
			{
				for (int j = 0, k = 0; j < SKELETON_POSITION_COUNT; j++)
				{
					float weight;
					objFile >> weight;
					if (weight > 0.00001f) // small epsilon value
					{
						m_vertices[i].joint_idx[k] = j;
						m_vertices[i].weight[k] = weight;
						k++;
					}
				}
				i++; // next vertex
			}
		}

		FILE* posedirsFile = nullptr;
		fopen_s(&posedirsFile, posedirsFilename.c_str(), "rb");
		if (!posedirsFile)
		{
			std::cerr << "[ERROR] Can not open the posedirs file \"" << posedirsFilename << "\"\n";
		}

		fread(m_posedirs, sizeof(float), POSEDIRS_ELEMENTS_COUNT, posedirsFile);
		fclose(posedirsFile);
	}
}