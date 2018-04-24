#pragma once
#include <d3d11_1.h>
#include "Basic.h"

class OffscreenRenderer
{
public:
	OffscreenRenderer() { VALIDATE(initDevice(), L"[ERROR] Device not initialized"); }
	~OffscreenRenderer() { cleanupDevice(); }
	ID3D11Device* getDevice() { return m_pd3dDevice; }
	ID3D11DeviceContext* getDeviceContext() { return m_pImmediateContext; }

private:
	HRESULT initDevice();
	void cleanupDevice();

	D3D_FEATURE_LEVEL        m_featureLevel = D3D_FEATURE_LEVEL_11_0;
	ID3D11Device*            m_pd3dDevice = nullptr;
	ID3D11DeviceContext*     m_pImmediateContext = nullptr;
};