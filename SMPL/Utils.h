#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <Windows.h>
#include <Eigen/Eigen>

#include "DXUtils.h"
#include "Definitions.h"

namespace smpl 
{
	extern D3D d3d_context;

	struct SparseMatrixGPU
	{
		std::vector<float> values;
		std::vector<uint> indices;
		std::vector<uint> pointers;
	};

	struct SparseMatrix
	{
		std::vector<Eigen::Triplet<float> > triplets;
		
		SparseMatrixGPU ToGPU();
		Eigen::SparseMatrix<float> ToEigen(const uint& joint_count) const;
	};

	std::vector<byte> ReadShaderFromCSO(const std::string& filename);
	void ReadObjFile(const std::string& filename, std::vector<float3>& vertices, std::vector<uint>& indices, std::vector<Skin>& skins);
	void ReadFloat3FromBinaryFile(const std::string& filename, std::vector<float3>& array, size_t arraySize);
	void ReadSparseMatrixFile(const std::string& filename, SparseMatrix& matrix);
};