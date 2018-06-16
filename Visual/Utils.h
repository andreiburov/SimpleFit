#pragma once
#include <windows.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <sstream>
#include <cstdio>
#include <iterator>

#define VALIDATE(x, wstr) \
do { HRESULT hr = (x); if( FAILED(hr) ) \
{ MessageBoxW(NULL, wstr, L"File error", MB_ICONERROR | MB_OK); } } while(0)
#define V_RETURN(x, wstr) \
do { HRESULT hr = (x); if( FAILED(hr) ) \
{ MessageBoxW(NULL, wstr, L"File error", MB_ICONERROR | MB_OK); return hr; } } while(0)
#define SAFE_RELEASE(p) do { if (p) { (p)->Release(); (p) = nullptr; } } while(0)

std::vector<byte> readShaderFromCSO(const std::string& filename);