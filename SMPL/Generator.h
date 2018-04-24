#pragma once

#include <string>
#include <vector>
#include "Definitions.h"
#include "Body.h"
#include "OffscreenRenderer.h"

namespace smpl {

	class Generator
	{
	public:
		Generator(std::string objFilename, std::string posedirsFilename);
		Generator() : 
			Generator(std::string("Model/smpl.obj"), std::string("Model/smpl.posedirs")) {}
		~Generator();

		// generate 3D model based on coefficients
		void run(ShapeCoefficients& shape, PoseCoefficients& pose, Body& body);

	private:
		void readConfigs(std::string& objFilename, std::string& posedirsFilename);
		void initD3D(const std::string& vertexShaderFilename);
		void cleanupD3D();

		std::vector<Vertex> m_vertices;
		std::vector<unsigned short> m_indices;
		float* m_posedirs;
		OffscreenRenderer m_renderer;

		// D3D
		ID3D11VertexShader* m_pVertexShader = nullptr;
		ID3D11InputLayout * m_pInputLayout = nullptr;
		ID3D11Buffer* m_pVertexBuffer = nullptr;
		ID3D11GeometryShader* m_pGeometryShader = nullptr;
		ID3D11Buffer* m_pStreamOutBuffer = nullptr;
		ID3D11Buffer* m_pStagingStreamOutBuffer = nullptr;

		// Linear Blend Skinning Matrices
		ID3D11Buffer* m_pLBSConstantBuffer = nullptr;
		//SimpleLBS m_LBS;

		// Thetas for SMPL
		ID3D11Buffer* m_pPoseBuffer = nullptr;
		ID3D11ShaderResourceView*  m_pPoseSRV = nullptr;
		//SimplePose m_Pose;

		// per-vertex basis of position directions for SMPL
		ID3D11ShaderResourceView*  m_pPosedirsSRV = nullptr;
	};
}