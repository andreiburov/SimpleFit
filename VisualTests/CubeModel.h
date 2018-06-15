#pragma once
#include "Utils.h"
#include "Camera.h"

class CubeModel
{
public:

	CubeModel(Camera& camera) : camera_(camera) {}

	void Initialize(ID3D11Device* pd3dDevice, ID3D11DeviceContext* pImmediateContext);
	void Render();
	void Clear();

private:

	typedef Eigen::Vector3f CubeVertex;

	ID3D11DeviceContext*	m_pDeviceContext;
	ID3D11VertexShader*		m_pVertexShader = nullptr;
	ID3D11GeometryShader*	m_pGeometryShader = nullptr;
	ID3D11PixelShader*		m_pPixelShader = nullptr;

	ID3D11InputLayout *		m_pInputLayout = nullptr;
	ID3D11Buffer*			m_pVertexBuffer = nullptr;
	ID3D11Buffer*			m_pIndexBuffer = nullptr;
	unsigned int			m_IndicesCount = 0;

	ID3D11Buffer*			m_pCameraConstantBuffer = nullptr;
	
	Camera&					camera_;
};