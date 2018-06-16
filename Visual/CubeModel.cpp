#include "Utils.h"
#include "CubeModel.h"

void CubeModel::Initialize(ID3D11Device* pd3dDevice, ID3D11DeviceContext* pImmediateContext)
{
	m_pDeviceContext = pImmediateContext;

	const D3D11_INPUT_ELEMENT_DESC vertexLayoutDesc[] =
	{
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0,  0, D3D11_INPUT_PER_VERTEX_DATA, 0 }
	};

	{
		std::vector<byte> vertexShader = readShaderFromCSO("CubeVS.cso");
		VALIDATE(pd3dDevice->CreateVertexShader(vertexShader.data(), vertexShader.size(),
			nullptr, &m_pVertexShader), L"Could not create VertexShader");

		VALIDATE(pd3dDevice->CreateInputLayout(vertexLayoutDesc, ARRAYSIZE(vertexLayoutDesc),
			vertexShader.data(), vertexShader.size(), &m_pInputLayout), L"Could not create InputLayout");

		std::vector<byte> geometryShader = readShaderFromCSO("CubeGS.cso");
		VALIDATE(pd3dDevice->CreateGeometryShader(geometryShader.data(), geometryShader.size(),
			nullptr, &m_pGeometryShader), L"Could not create GeometryShader");

		std::vector<byte> pixelShader = readShaderFromCSO("CubePS.cso");
		VALIDATE(pd3dDevice->CreatePixelShader(pixelShader.data(), pixelShader.size(),
			nullptr, &m_pPixelShader), L"Could not create PixelShader");
	}

	std::vector<Eigen::Vector3f> vertices;
	vertices.reserve(8);

	vertices.push_back(Eigen::Vector3f(-1.0, -1.0, 1.0));
	vertices.push_back(Eigen::Vector3f(-1.0, 1.0, 1.0));
	vertices.push_back(Eigen::Vector3f(1.0, 1.0, 1.0));
	vertices.push_back(Eigen::Vector3f(1.0, -1.0, 1.0));
	vertices.push_back(Eigen::Vector3f(-1.0, -1.0, -1.0));
	vertices.push_back(Eigen::Vector3f(1.0, -1.0, -1.0));
	vertices.push_back(Eigen::Vector3f(1.0, 1.0, -1.0));
	vertices.push_back(Eigen::Vector3f(-1.0, 1.0, -1.0));

	// World Matrix
	/*for (auto& v : vertices)
	{
	v = Eigen::Translation3f(Eigen::Vector3f(0.0, 0.0, -5.0)) * v;
	}*/

	unsigned short indices[] = {
		// front
		0, 1, 2,
		0, 2, 3,
		// right
		3, 2, 6,
		3, 6, 5,
		// back
		5, 6, 7,
		5, 7, 4,
		// left
		4, 7, 1,
		4, 1, 0,
		// bottom
		4, 0, 3,
		4, 3, 5,
		// top
		1, 7, 6,
		1, 6, 2,
	};

	{
		D3D11_BUFFER_DESC vertexBufferDesc = { 0 };
		vertexBufferDesc.ByteWidth = sizeof(CubeVertex) * (unsigned int)vertices.size();
		vertexBufferDesc.Usage = D3D11_USAGE_DEFAULT;
		vertexBufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
		vertexBufferDesc.CPUAccessFlags = 0;
		vertexBufferDesc.MiscFlags = 0;
		vertexBufferDesc.StructureByteStride = 0;

		D3D11_SUBRESOURCE_DATA vertexBufferData;
		vertexBufferData.pSysMem = vertices.data();
		vertexBufferData.SysMemPitch = 0;
		vertexBufferData.SysMemSlicePitch = 0;

		VALIDATE(pd3dDevice->CreateBuffer(&vertexBufferDesc, &vertexBufferData,
			&m_pVertexBuffer), L"Could not create VertexBuffer");
	}

	m_IndicesCount = (unsigned int)ARRAYSIZE(indices);

	{
		D3D11_BUFFER_DESC indexBufferDesc;
		indexBufferDesc.ByteWidth = sizeof(unsigned short) * (unsigned int)m_IndicesCount;
		indexBufferDesc.Usage = D3D11_USAGE_DEFAULT;
		indexBufferDesc.BindFlags = D3D11_BIND_INDEX_BUFFER;
		indexBufferDesc.CPUAccessFlags = 0;
		indexBufferDesc.MiscFlags = 0;
		indexBufferDesc.StructureByteStride = 0;

		D3D11_SUBRESOURCE_DATA indexBufferData;
		indexBufferData.pSysMem = indices;
		indexBufferData.SysMemPitch = 0;
		indexBufferData.SysMemSlicePitch = 0;

		VALIDATE(pd3dDevice->CreateBuffer(&indexBufferDesc, &indexBufferData, &m_pIndexBuffer),
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

		VALIDATE(pd3dDevice->CreateBuffer(&constantBufferDesc, nullptr, &m_pCameraConstantBuffer),
			L"Could not create CameraConstantBuffer");
	}
}

void CubeModel::Draw()
{
	m_pDeviceContext->UpdateSubresource(m_pCameraConstantBuffer, 0, nullptr, camera_.GetDataPointer(), 0, 0);
	m_pDeviceContext->IASetInputLayout(m_pInputLayout);

	// Set the vertex and index buffers, and specify the way they define geometry
	UINT stride = sizeof(CubeVertex);
	UINT offset = 0;
	m_pDeviceContext->IASetVertexBuffers(0, 1, &m_pVertexBuffer, &stride, &offset);
	m_pDeviceContext->IASetIndexBuffer(m_pIndexBuffer, DXGI_FORMAT_R16_UINT, 0);
	m_pDeviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

	// Set the vertex and pixel shader stage state
	m_pDeviceContext->VSSetShader(m_pVertexShader, nullptr, 0);
	m_pDeviceContext->GSSetShader(m_pGeometryShader, nullptr, 0);
	m_pDeviceContext->GSSetConstantBuffers(0, 1, &m_pCameraConstantBuffer);
	m_pDeviceContext->PSSetShader(m_pPixelShader, nullptr, 0);

	m_pDeviceContext->DrawIndexed(m_IndicesCount, 0, 0);
}

void CubeModel::Clear()
{
	SAFE_RELEASE(m_pVertexShader);
	SAFE_RELEASE(m_pPixelShader);
	SAFE_RELEASE(m_pGeometryShader);
	SAFE_RELEASE(m_pInputLayout);
	SAFE_RELEASE(m_pVertexBuffer);
	SAFE_RELEASE(m_pIndexBuffer);
	SAFE_RELEASE(m_pCameraConstantBuffer);
}