#include <d3d11.h>
#include <Eigen/Eigen>
#include <AntTweakBar.h>
#include <windows.h>

#include <iostream>
#include <fstream>
#include <sstream>

#include "Camera.h"
#include "CubeModel.h"
#include "SmplModel.h"

// D3D objects
ID3D11Device*				g_D3DDev = NULL;
ID3D11DeviceContext*		g_D3DDevCtx = NULL;
IDXGISwapChain*				g_SwapChain = NULL;
DXGI_SWAP_CHAIN_DESC		g_SwapChainDesc;
ID3D11RenderTargetView*		g_RenderTargetView = NULL;
ID3D11DepthStencilView*		g_DepthStencilView = NULL;
D3D11_TEXTURE2D_DESC		g_DepthStencilDesc;
ID3D11BlendState*			g_BlendState = NULL;
ID3D11DepthStencilState*	g_DepthStencilState = NULL;
ID3D11RasterizerState*		g_RasterState = NULL;
TwBar*						g_bar = NULL;
Eigen::Vector4f				g_BackgroundColor(0.0f, 0.6f, 0.6f, 1.0f);

Camera						g_Camera;
CubeModel					g_Cube(g_Camera);
SmplModel					g_SmplModel(g_Camera);
std::string					g_betas_string;
std::string					g_thetas_string;
std::string					g_thetas_euler_string;
Eigen::Quaternionf			g_thetas[smpl::THETA_COUNT];

smpl::ShapeCoefficients		g_shape;
smpl::PoseAxisAngleCoefficients g_pose;
smpl::PoseEulerCoefficients g_pose_euler;
bool						g_use_euler = false;

// Forward declarations
HRESULT InitDevice(HWND wnd);
HRESULT InitScene();
void InitSceneObjects();
void DestroySceneObjects();
void Cleanup();
LRESULT CALLBACK MessageProc(HWND, UINT, WPARAM, LPARAM);
void Anim();
void Render();
std::vector<byte> readShaderFromCSO(const std::string& filename);

void TW_CALL CopyStdStringToClient(std::string& destinationClientString, const std::string& sourceLibraryString)
{
	// Copy the content of souceString handled by the AntTweakBar library to destinationClientString handled by your application
	destinationClientString = sourceLibraryString;
}

void TW_CALL DumpBtnCB(void* /*clientData*/)
{
	g_SmplModel.Dump("mesh.obj");
}

void TW_CALL ResetBtnCB(void* /*clientData*/)
{
	ZeroMemory(&g_pose, sizeof(g_pose));
	//ZeroMemory(&g_pose_euler, sizeof(g_pose_euler));
	//ZeroMemory(&g_shape, sizeof(g_shape));
	g_shape.Reset();
	g_pose_euler.Reset();

	for (int i = 0; i < smpl::THETA_COUNT; i++)
	{
		g_thetas[i] = Eigen::Quaternionf::Identity();
	}

	g_betas_string = "";
	g_thetas_string = "";
	g_thetas_euler_string = "";

	g_SmplModel.Generate(g_shape, g_pose);
}

void TW_CALL GenerateBtnCB(void* /*clientData*/)
{
	g_BackgroundColor = Eigen::Vector4f(0.0f, (float)rand()/RAND_MAX, (float)rand()/RAND_MAX, 1.0f);

	g_shape << g_betas_string;
	g_pose << g_thetas_string;
	g_pose_euler << g_thetas_euler_string;

	if (g_use_euler)
	{
		for (int i = 0; i < smpl::THETA_COUNT; i++)
		{
			Eigen::Quaternionf q(EulerRotationZYX(g_pose_euler[i].x, g_pose_euler[i].y, g_pose_euler[i].z));
			g_thetas[i] = q;
		}
		g_SmplModel.Generate(g_shape, g_pose_euler);
	}
	else
	{
		for (int i = 0; i < smpl::THETA_COUNT; i++)
		{
			Eigen::Quaternionf q(Eigen::AngleAxisf(g_pose[i].ToEigen().norm(), g_pose[i].ToEigen().normalized()));
			g_thetas[i] = q;
		}
		g_SmplModel.Generate(g_shape, g_pose);
	}
}

void TW_CALL SetBeta(const void *value, void * clientData)
{
	int i = (int)clientData;
	g_shape[i] = *(float*)(value);
	
	g_SmplModel.Generate(g_shape, g_pose);

	std::stringstream ss;
	for (UINT i = 0; i < smpl::BETA_COUNT; i++)
	{
		ss << g_shape[i] << " ";
	}

	g_betas_string = ss.str();
}

void TW_CALL GetBeta(void *value, void * clientData)
{
	int i = (int)clientData;
	*(float*)(value) = g_shape[i];
}

void TW_CALL SetTheta(const void *value, void * clientData)
{
	int i = (int)clientData;
	g_thetas[i] = *(Eigen::Quaternionf*)(value);	

	{
		Eigen::Vector3f theta = g_thetas[i].toRotationMatrix().eulerAngles(0, 1, 2);
		g_pose_euler[i] = smpl::float3(theta);
		std::stringstream ss;

		for (UINT j = 0; j < smpl::THETA_COUNT; j++)
		{
			ss << g_pose_euler[j] << " ";
		}

		g_thetas_euler_string = ss.str();
	}
	{
		Eigen::AngleAxisf theta(g_thetas[i]);
		g_pose[i] = smpl::float3(theta.axis() * theta.angle());
		std::stringstream ss;

		for (UINT j = 0; j < smpl::THETA_COUNT; j++)
		{
			ss << g_pose[j] << " ";
		}

		g_thetas_string = ss.str();
	}

	if (g_use_euler)
	{
		g_SmplModel.Generate(g_shape, g_pose_euler);
	}
	else
	{
		g_SmplModel.Generate(g_shape, g_pose);
	}
}

void TW_CALL GetTheta(void *value, void * clientData)
{
	int i = (int)clientData;
	*(Eigen::Quaternionf*)(value) = g_thetas[i];
}


// Main
int WINAPI WinMain(HINSTANCE instance, HINSTANCE, LPSTR, int cmdShow)
{
	AllocConsole();
	FILE *stream;
	freopen_s(&stream, "CON", "w", stdout);
	freopen_s(&stream, "CON", "w", stderr);
	freopen_s(&stream, "CON", "r", stdin);

	// Register our window class
	WNDCLASSEX wcex = { sizeof(WNDCLASSEX), CS_HREDRAW | CS_VREDRAW, MessageProc,
		0L, 0L, instance, NULL, NULL, NULL, NULL, L"Debugger", NULL };
	RegisterClassEx(&wcex);

	// Create a window
	RECT rc = { 0, 0, 640, 480 };
	AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW, FALSE);
	HWND wnd = CreateWindow(L"Debugger", L"Visualizations",
		WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT,
		rc.right - rc.left, rc.bottom - rc.top, NULL, NULL, instance, NULL);
	if (!wnd)
	{
		MessageBox(NULL, L"Cannot create window", L"Error", MB_OK | MB_ICONERROR);
		return 0;
	}
	ShowWindow(wnd, cmdShow);
	UpdateWindow(wnd);

	// Initialize D3D11
	if (FAILED(InitDevice(wnd)))
	{
		MessageBox(wnd, L"Cannot create D3D11 device", L"Error", MB_OK | MB_ICONERROR);
		Cleanup();
		return 0;
	}

	// Initialize the 3D scene
	if (FAILED(InitScene()))
	{
		MessageBox(wnd, L"Scene initialization failed.", L"Error", MB_OK | MB_ICONERROR);
		Cleanup();
		return 0;
	}

	InitSceneObjects();

	// Initialize AntTweakBar
	if (!TwInit(TW_DIRECT3D11, g_D3DDev))
	{
		MessageBoxA(wnd, TwGetLastError(), "AntTweakBar initialization failed", MB_OK | MB_ICONERROR);
		Cleanup();
		return 0;
	}

	// Create a tweak bar
	g_bar = TwNewBar("TweakBar");
	TwDefine(" GLOBAL help='This example shows how to integrate AntTweakBar into a DirectX11 application.' "); // Message added to the help bar.
	int barSize[2] = { 224, 320 };
	TwSetParam(g_bar, NULL, "size", TW_PARAM_INT32, 2, barSize);
	TwCopyStdStringToClientFunc(CopyStdStringToClient);

	TwAddVarRW(g_bar, "Background", TW_TYPE_COLOR4F, &g_BackgroundColor, "colormode=hls");
	TwAddVarRW(g_bar, "Use Euler Angles", TW_TYPE_BOOLCPP, &g_use_euler, "");
	TwAddVarRW(g_bar, "Betas", TW_TYPE_STDSTRING, &g_betas_string, "");
	TwAddVarRW(g_bar, "Thetas", TW_TYPE_STDSTRING, &g_thetas_string, "");
	TwAddVarRW(g_bar, "Thetas Euler", TW_TYPE_STDSTRING, &g_thetas_euler_string, "");
	TwAddButton(g_bar, "Generate", GenerateBtnCB, nullptr, "");
	TwAddButton(g_bar, "Reset", ResetBtnCB, nullptr, "");
	TwAddButton(g_bar, "Dump", DumpBtnCB, nullptr, "");
	
	for (UINT i = 0; i < smpl::BETA_COUNT; i++)
	{
		TwAddVarCB(g_bar, std::string("B").append(std::to_string(i)).c_str(),
			TW_TYPE_FLOAT, SetBeta, GetBeta, (void*)i, "min=-10 max=10 step=0.1");
	}

	//ZeroMemory(&g_shape, sizeof(g_shape));
	g_shape.Reset();
	ZeroMemory(&g_pose, sizeof(g_pose));
	g_pose_euler.Reset();
	//ZeroMemory(&g_pose_euler, sizeof(g_pose_euler));
	for (UINT i = 0; i < smpl::THETA_COUNT; i++)
	{
		TwAddVarCB(g_bar, std::string("T").append(std::to_string(i)).append(" ").append(smpl::JOINT_FROM_INDEX[i]).c_str(),
			TW_TYPE_QUAT4F, SetTheta, GetTheta, (void*)i, "opened=true axisz=z");
	}

	// Main message loop
	MSG msg = { 0 };
	while (WM_QUIT != msg.message)
	{
		if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		else
		{
			Anim();
			Render();
		}
	}

	TwTerminate();
	DestroySceneObjects();
	Cleanup();

	return (int)msg.wParam;
}

// Create Direct3D device and swap chain
HRESULT InitDevice(HWND wnd)
{
	HRESULT hr = S_OK;

	// Get window size
	RECT rc;
	GetClientRect(wnd, &rc);
	UINT width = rc.right - rc.left;
	UINT height = rc.bottom - rc.top;

	// Create D3D11 device and swap chain
	UINT createDeviceFlags = 0;
#ifdef _DEBUG
	createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
#endif
	ZeroMemory(&g_SwapChainDesc, sizeof(g_SwapChainDesc));
	g_SwapChainDesc.BufferCount = 1;
	g_SwapChainDesc.BufferDesc.Width = width;
	g_SwapChainDesc.BufferDesc.Height = height;
	g_SwapChainDesc.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	g_SwapChainDesc.BufferDesc.RefreshRate.Numerator = 0;
	g_SwapChainDesc.BufferDesc.RefreshRate.Denominator = 0;
	g_SwapChainDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	g_SwapChainDesc.OutputWindow = wnd;
	g_SwapChainDesc.SampleDesc.Count = 4;
	g_SwapChainDesc.SampleDesc.Quality = 0;
	g_SwapChainDesc.Windowed = TRUE;
	g_SwapChainDesc.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
	// Try to create a hardware accelerated device with multisample antialiasing first
	hr = D3D11CreateDeviceAndSwapChain(NULL, D3D_DRIVER_TYPE_HARDWARE, NULL, createDeviceFlags,
		NULL, 0, D3D11_SDK_VERSION, &g_SwapChainDesc, &g_SwapChain,
		&g_D3DDev, NULL, &g_D3DDevCtx);
	if (FAILED(hr))
	{
		// If failed, try without antialiasing
		g_SwapChainDesc.SampleDesc.Count = 1;
		hr = D3D11CreateDeviceAndSwapChain(NULL, D3D_DRIVER_TYPE_HARDWARE, NULL, createDeviceFlags,
			NULL, 0, D3D11_SDK_VERSION, &g_SwapChainDesc, &g_SwapChain,
			&g_D3DDev, NULL, &g_D3DDevCtx);
		if (FAILED(hr))
		{
			// If failed, try to create a reference device
			hr = D3D11CreateDeviceAndSwapChain(NULL, D3D_DRIVER_TYPE_REFERENCE, NULL, createDeviceFlags,
				NULL, 0, D3D11_SDK_VERSION, &g_SwapChainDesc, &g_SwapChain,
				&g_D3DDev, NULL, &g_D3DDevCtx);
			if (SUCCEEDED(hr))
				MessageBox(wnd, L"No DX11 hardware acceleration found.\nSwitching to REFERENCE driver (very slow).",
					L"Warning", MB_OK | MB_ICONWARNING);
			else
				return hr;
		}
	}

	// Create a render target and depth-stencil view
	ID3D11Texture2D *backBuffer = NULL, *dsBuffer = NULL;
	hr = g_SwapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), (LPVOID*)&backBuffer);
	if (FAILED(hr))
		return hr;

	hr = g_D3DDev->CreateRenderTargetView(backBuffer, NULL, &g_RenderTargetView);
	backBuffer->Release();
	if (FAILED(hr))
		return hr;

	g_DepthStencilDesc.Width = width;
	g_DepthStencilDesc.Height = height;
	g_DepthStencilDesc.MipLevels = 1;
	g_DepthStencilDesc.ArraySize = 1;
	g_DepthStencilDesc.Format = DXGI_FORMAT_D16_UNORM;
	g_DepthStencilDesc.SampleDesc = g_SwapChainDesc.SampleDesc;
	g_DepthStencilDesc.Usage = D3D11_USAGE_DEFAULT;
	g_DepthStencilDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL;
	g_DepthStencilDesc.CPUAccessFlags = 0;
	g_DepthStencilDesc.MiscFlags = 0;
	hr = g_D3DDev->CreateTexture2D(&g_DepthStencilDesc, NULL, &dsBuffer);
	if (FAILED(hr))
		return hr;
	hr = g_D3DDev->CreateDepthStencilView(dsBuffer, NULL, &g_DepthStencilView);
	dsBuffer->Release();
	if (FAILED(hr))
		return hr;

	g_D3DDevCtx->OMSetRenderTargets(1, &g_RenderTargetView, g_DepthStencilView);

	// Setup the viewport
	D3D11_VIEWPORT vp;
	vp.Width = (float)width;
	vp.Height = (float)height;
	vp.MinDepth = 0.0f;
	vp.MaxDepth = 1.0f;
	vp.TopLeftX = 0;
	vp.TopLeftY = 0;
	g_D3DDevCtx->RSSetViewports(1, &vp);

	return S_OK;
}


// Initialize the 3D objects & shaders
HRESULT InitScene()
{
	// Blend state
	D3D11_BLEND_DESC bsd;
	bsd.AlphaToCoverageEnable = FALSE;
	bsd.IndependentBlendEnable = FALSE;
	for (int i = 0; i < 8; i++)
	{
		bsd.RenderTarget[i].BlendEnable = TRUE;
		bsd.RenderTarget[i].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
		bsd.RenderTarget[i].SrcBlend = D3D11_BLEND_SRC_ALPHA;
		bsd.RenderTarget[i].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
		bsd.RenderTarget[i].BlendOp = D3D11_BLEND_OP_ADD;
		bsd.RenderTarget[i].SrcBlendAlpha = D3D11_BLEND_SRC_ALPHA;
		bsd.RenderTarget[i].DestBlendAlpha = D3D11_BLEND_INV_SRC_ALPHA;
		bsd.RenderTarget[i].BlendOpAlpha = D3D11_BLEND_OP_ADD;
	}
	g_D3DDev->CreateBlendState(&bsd, &g_BlendState);
	float blendFactors[4] = { 1, 1, 1, 1 };
	g_D3DDevCtx->OMSetBlendState(g_BlendState, blendFactors, 0xffffffff);

	// Depth-stencil state
	D3D11_DEPTH_STENCILOP_DESC od;
	od.StencilFunc = D3D11_COMPARISON_ALWAYS;
	od.StencilFailOp = D3D11_STENCIL_OP_KEEP;
	od.StencilPassOp = D3D11_STENCIL_OP_KEEP;
	od.StencilDepthFailOp = D3D11_STENCIL_OP_KEEP;
	D3D11_DEPTH_STENCIL_DESC dsd;
	dsd.DepthEnable = TRUE;
	dsd.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
	dsd.DepthFunc = D3D11_COMPARISON_LESS_EQUAL;
	dsd.StencilEnable = FALSE;
	dsd.StencilReadMask = D3D11_DEFAULT_STENCIL_READ_MASK;
	dsd.StencilWriteMask = D3D11_DEFAULT_STENCIL_WRITE_MASK;
	dsd.FrontFace = od;
	dsd.BackFace = od;
	g_D3DDev->CreateDepthStencilState(&dsd, &g_DepthStencilState);
	g_D3DDevCtx->OMSetDepthStencilState(g_DepthStencilState, 0);

	// Rasterizer state
	D3D11_RASTERIZER_DESC rs;
	ZeroMemory(&rs, sizeof(rs));
	rs.FillMode = D3D11_FILL_SOLID;
	rs.CullMode = D3D11_CULL_NONE;
	rs.MultisampleEnable = (g_SwapChainDesc.SampleDesc.Count > 0);
	g_D3DDev->CreateRasterizerState(&rs, &g_RasterState);
	g_D3DDevCtx->RSSetState(g_RasterState);

	return S_OK;
}


// Clean up D3D objects
void Cleanup()
{
#define RELEASE_CHECK(p) if (p) { ULONG rc = (p)->Release(); assert(rc == 0); (void)rc; (p) = NULL; } 

	if (g_D3DDevCtx)
		g_D3DDevCtx->ClearState();

	RELEASE_CHECK(g_BlendState);
	RELEASE_CHECK(g_DepthStencilState);
	RELEASE_CHECK(g_RasterState);
	RELEASE_CHECK(g_RenderTargetView);
	RELEASE_CHECK(g_DepthStencilView);
	if (g_SwapChainDesc.Windowed)
		RELEASE_CHECK(g_SwapChain);
	RELEASE_CHECK(g_D3DDevCtx);
	RELEASE_CHECK(g_D3DDev);
}

// Called every time the application receives a message
LRESULT CALLBACK MessageProc(HWND wnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	// Send event message to AntTweakBar
	if (TwEventWin(wnd, message, wParam, lParam))
		return 0; // Event has been handled by AntTweakBar

	switch (message)
	{
	case WM_PAINT:
	{
		PAINTSTRUCT ps;
		BeginPaint(wnd, &ps);
		EndPaint(wnd, &ps);
		return 0;
	}
	case WM_SIZE: // Window size has been changed
		if (g_D3DDev) // Resize D3D render target
		{
			// Release render target and depth-stencil view
			ID3D11RenderTargetView *nullRTV = NULL;
			g_D3DDevCtx->OMSetRenderTargets(1, &nullRTV, NULL);
			if (g_RenderTargetView)
			{
				g_RenderTargetView->Release();
				g_RenderTargetView = NULL;
			}
			if (g_DepthStencilView)
			{
				g_DepthStencilView->Release();
				g_DepthStencilView = NULL;
			}

			if (g_SwapChain)
			{
				// Resize swap chain
				UINT width = LOWORD(lParam);
				UINT height = HIWORD(lParam);
				g_SwapChainDesc.BufferDesc.Width = width;
				g_SwapChainDesc.BufferDesc.Height = height;
				g_SwapChain->ResizeBuffers(g_SwapChainDesc.BufferCount, g_SwapChainDesc.BufferDesc.Width,
					g_SwapChainDesc.BufferDesc.Height, g_SwapChainDesc.BufferDesc.Format,
					g_SwapChainDesc.Flags);

				// Re-create a render target and depth-stencil view
				ID3D11Texture2D *backBuffer = NULL, *dsBuffer = NULL;
				g_SwapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), (LPVOID*)&backBuffer);
				g_D3DDev->CreateRenderTargetView(backBuffer, NULL, &g_RenderTargetView);
				backBuffer->Release();
				g_DepthStencilDesc.Width = width;
				g_DepthStencilDesc.Height = height;
				g_D3DDev->CreateTexture2D(&g_DepthStencilDesc, NULL, &dsBuffer);
				g_D3DDev->CreateDepthStencilView(dsBuffer, NULL, &g_DepthStencilView);
				dsBuffer->Release();
				g_D3DDevCtx->OMSetRenderTargets(1, &g_RenderTargetView, g_DepthStencilView);

				// Setup the viewport
				D3D11_VIEWPORT vp;
				vp.Width = (float)width;
				vp.Height = (float)height;
				vp.MinDepth = 0.0f;
				vp.MaxDepth = 1.0f;
				vp.TopLeftX = 0;
				vp.TopLeftY = 0;
				g_D3DDevCtx->RSSetViewports(1, &vp);

				g_Camera.SetPerspective((float)width / height);
			}

			// TwWindowSize has been called by TwEventWin, so it is not necessary to call it again here.
		}
		return 0;

	case WM_CHAR:
		if (wParam == VK_ESCAPE)
			PostQuitMessage(0);
		return 0;

	case WM_LBUTTONDOWN:
		POINT focusPoint;
		GetCursorPos(&focusPoint);
		ScreenToClient(GetActiveWindow(), &focusPoint);
		g_Camera.Activate(focusPoint);
		return 0;

	case WM_LBUTTONUP:
		g_Camera.Deactivate();
		return 0;

	case WM_MOUSEWHEEL:
		int delta;
		delta = GET_WHEEL_DELTA_WPARAM(wParam);
		g_Camera.Zoom(delta);
		return 0;

	case WM_DESTROY:
		PostQuitMessage(0);
		return 0;

	default:
		return DefWindowProc(wnd, message, wParam, lParam);
	}
}

void InitSceneObjects()
{
	g_Camera.SetPerspective((float)g_DepthStencilDesc.Width / g_DepthStencilDesc.Height);
	//g_Cube.Initialize(g_D3DDev, g_D3DDevCtx);
	g_SmplModel.Initialize(g_D3DDev, g_D3DDevCtx);
}

void DestroySceneObjects()
{
	//g_Cube.Clear();
	g_SmplModel.Clear();
}

// Render a frame
void Render()
{
	// Clear the back buffer 
	g_D3DDevCtx->ClearRenderTargetView(g_RenderTargetView, g_BackgroundColor.data());
	g_D3DDevCtx->ClearDepthStencilView(g_DepthStencilView, D3D11_CLEAR_DEPTH, 1, 0);

	//g_Cube.Draw();
	g_SmplModel.Draw();
	
	// Draw tweak bars
	TwDraw();

	// Present the information rendered in the back buffer to the front buffer (the screen)
	g_SwapChain->Present(0, 0);
}

// Rotating sponge
void Anim()
{
	HWND wnd = GetActiveWindow();

	POINT focusPoint;
	GetCursorPos(&focusPoint);
	ScreenToClient(wnd, &focusPoint);

	RECT rect;
	GetClientRect(wnd, &rect);
	g_Camera.Move(focusPoint, rect);

	//std::cout << focusPoint.x << " " << focusPoint.y << " " << rect.left << " " << rect.right << " " << rect.top << " " << rect.bottom << std::endl;
}