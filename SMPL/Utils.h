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

	Eigen::Vector3f mul(const Eigen::Matrix4f&, const Eigen::Vector3f&);

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
	void ReadFloat3FromBinary(const std::string& filename, std::vector<float3>& array, size_t arraySize);
	void ReadSparseMatrixFile(const std::string& filename, SparseMatrix& matrix);

	template <typename T>
	void DumpFloatVectorToBinary(const std::string& filename, const std::vector<T>& vector)
	{
		FILE* file = nullptr;
		fopen_s(&file, filename.c_str(), "wb");
		if (!file)
		{
			MessageBoxA(nullptr, std::string("Could not open the file: ").append(filename).c_str(), "Error", MB_OK);
		}

		int info[1];
		info[0] = static_cast<int>(vector.size());

		fwrite(info, sizeof(int), 1, file);
		fwrite(vector.data(), sizeof(T), info[0], file);

		fclose(file);
	}

	template <typename T>
	void ReadFloatVectorFromBinary(const std::string& filename, std::vector<T>& vector)
	{
		FILE* file = nullptr;
		fopen_s(&file, filename.c_str(), "rb");
		if (!file)
		{
			MessageBoxA(nullptr, std::string("Could not open the file: ").append(filename).c_str(), "Error", MB_OK);
		}

		int info[1];

		fread(info, sizeof(int), 1, file);
		vector.resize(info[0]);

		fread(vector.data(), sizeof(T), info[0], file);

		fclose(file);
	}
};