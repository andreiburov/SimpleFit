#pragma once

#include <iostream>
#include <vector>
#include <Windows.h>
#include <Eigen/Eigen>

#include "DXUtils.h"
#include "Definitions.h"

namespace smpl
{
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
		Eigen::SparseMatrix<float> ToEigen();
	};

	std::vector<byte> ReadShaderFromCSO(const std::string& filename);
	void ReadObjFile(const std::string& filename, std::vector<float3>& vertices, std::vector<uint>& indices, std::vector<Skin>& skins);
	void ReadFloat3FromBinaryFile(const std::string& filename, std::vector<float3>& array, size_t arraySize);
	void ReadSparseMatrixFile(const std::string& filename, SparseMatrix& matrix);
};