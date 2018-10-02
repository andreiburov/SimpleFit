#include <iostream>
#include <fstream>
#include <sstream>
#include <iterator>
#include <algorithm>
#include <set>
#include <ctime>

#include "Utils.h"
#include "Definitions.h"

namespace smpl 
{
	D3D d3d_context;

	SparseMatrixGPU SparseMatrix::ToGPU()
	{
		Eigen::SparseMatrix<float, Eigen::RowMajor> matrix(JOINT_COUNT, VERTEX_COUNT);
		matrix.setFromTriplets(triplets.begin(), triplets.end());

		SparseMatrixGPU gpu_matrix;
		int i = 0; // counter
		int pp = -1; // previous pointer

		for (int k = 0; k < matrix.outerSize(); ++k)
		{
			for (Eigen::SparseMatrix<float, Eigen::RowMajor>::InnerIterator it(matrix, k); it; ++it)
			{
				while (pp < it.row())
				{
					gpu_matrix.pointers.push_back(i);
					pp++;
				}

				gpu_matrix.indices.push_back(it.col());
				gpu_matrix.values.push_back(it.value());
				i++;
			}
		}

		return gpu_matrix;
	}

	Eigen::SparseMatrix<float> SparseMatrix::ToEigen(const uint& joint_count) const
	{
		Eigen::SparseMatrix<float> matrix(joint_count, VERTEX_COUNT);
		matrix.setFromTriplets(triplets.begin(), triplets.end());
		return matrix;
	}

	std::vector<byte> ReadShaderFromCSO(const std::string& filename)
	{
		std::ifstream file(filename, std::ios::in | std::ios::binary);
		if (!file)
		{
			std::cerr << "[ERROR] Can not open shader file \"" + filename + "\"\n";
		}

		// Stop eating new lines in binary mode!!!
		file.unsetf(std::ios::skipws);

		// get its size:
		std::streampos filesize;

		file.seekg(0, std::ios::end);
		filesize = file.tellg();
		file.seekg(0, std::ios::beg);

		// reserve capacity
		std::vector<byte> v;
		v.reserve((unsigned int)filesize);

		// read the data:
		v.insert(v.begin(),
			std::istream_iterator<byte>(file),
			std::istream_iterator<byte>());

		return v;
	}

	void ReadObjFile(const std::string& filename, std::vector<float3>& vertices, 
		std::vector<uint>& indices,
		std::vector<Skin>& skins)
	{
		std::ifstream file(filename, std::ios::in);
		if (!file)
		{
			std::cerr << "[ERROR] Can not open the obj file \"" << filename << "\"\n";
		}

		vertices.reserve(VERTEX_COUNT);
		indices.reserve(FACE_COUNT * 3);
		skins.resize(VERTEX_COUNT);

		int i = 0;
		std::string line;
		while (std::getline(file, line))
		{
			std::istringstream iss(line);
			std::string type;
			iss >> type;
			if (type.compare("v") == 0)
			{
				float x, y, z;
				iss >> x >> y >> z;
				vertices.push_back(float3(x, y, z));
			}
			else if (type.compare("f") == 0)
			{
				unsigned short x, y, z;
				iss >> x >> y >> z;
				indices.push_back(x - 1);
				indices.push_back(y - 1);
				indices.push_back(z - 1);
			}
			else if (type.compare("#w") == 0)
			{
				for (int j = 0, k = 0; j < JOINT_COUNT; j++)
				{
					float weight;
					iss >> weight;
					if (weight > 0.00001f) // small epsilon value
					{
						skins[i].joint_index[k] = j;
						skins[i].weight[k] = weight;
						k++;
					}
				}
				i++; // next vertex
			}
		}
	}

	void ReadFloat3FromBinaryFile(const std::string& filename, std::vector<float3>& array, size_t arraySize)
	{
		FILE* binaryFile = nullptr;
		fopen_s(&binaryFile, filename.c_str(), "rb");
		if (!binaryFile)
		{
			std::cerr << "[ERROR] Can not open a binary file \"" << filename << "\"\n";
		}

		array.resize(arraySize);
		fread(array.data(), sizeof(float3), arraySize, binaryFile);
		fclose(binaryFile);
	}

	void ReadSparseMatrixFile(const std::string& filename, SparseMatrix& matrix)
	{
		std::ifstream file(filename, std::ios::in);
		if (!file)
		{
			MessageBoxA(NULL, std::string("File not found: ").append(filename).c_str(), "Error", MB_OK);
		}

		int i, j;
		float v;
		
		while (file >> i >> j >> v)
		{
			matrix.triplets.push_back(Eigen::Triplet<float>(i,j,v));
		}
	}
}; // smpl