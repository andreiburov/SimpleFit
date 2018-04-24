#pragma once

#include <iostream>
#include <vector>
#include <Windows.h>

#define SAFE_RELEASE(p) do { if (p) { (p)->Release(); (p) = nullptr; } } while(0)

#define VALIDATE(x, wstr) \
do { HRESULT hr = (x); if( !SUCCEEDED(hr) ) \
{ std::wcerr << "[ERROR] " << wstr << "\n"; } } while(0)

#define V_RETURN(x, wstr) \
do { HRESULT hr = (x); if( !SUCCEEDED(hr) ) \
{ std::wcerr << "[ERROR] " << wstr << "\n"; return hr; } } while(0)

std::vector<byte> readShaderFromCSO(const std::string& filename);

namespace smpl {

	int printHello();
}