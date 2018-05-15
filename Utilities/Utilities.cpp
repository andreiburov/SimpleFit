#define _CRT_SECURE_NO_WARNINGS 1 
#define _WINSOCK_DEPRECATED_NO_WARNINGS 1 

#include <cstdio>
#include <iostream>
#include <fstream>
#include <vector>
#include <DirectXMath.h>

//#define SHAPE

#define SMPL_POSEDIRS_ELEMENTS_COUNT 4278690 // 6890 * 3 * 207
#define SMPL_SHAPEDIRS_ELEMENTS_COUNT 206700 // 6890 * 3 * 10

#ifdef SHAPE
#define ELEMENTS_COUNT SMPL_SHAPEDIRS_ELEMENTS_COUNT
#endif
#ifndef SHAPE 
#define ELEMENTS_COUNT SMPL_POSEDIRS_ELEMENTS_COUNT
#endif

int Convert(int argc, char** argv)
{
	size_t a = sizeof(DirectX::XMFLOAT3);
	if (argc != 3)
	{
		std::cout << "./ConvertDirsToBinary input output\n" << std::endl;
		return 0;
	}

	std::cout << "Converting " << argv[1] << " to " << argv[2] << std::endl;

	float* posedirs = new float[ELEMENTS_COUNT];

	{
		std::ifstream in(argv[1], std::ifstream::in);

		for (unsigned int i = 0; i < ELEMENTS_COUNT; i++)
		{
			float f;
			in >> f;
			posedirs[i] = f;
		}

		in.close();
	}

	FILE* out = fopen(argv[2], "wb");
	fwrite(posedirs, sizeof(float), ELEMENTS_COUNT, out);
	fclose(out);

	delete[] posedirs;

	{
		float* p = new float[ELEMENTS_COUNT];
		FILE* in = fopen(argv[2], "rb");
		fread(p, sizeof(float), ELEMENTS_COUNT, in);
		fclose(in);
		delete[] p;
	}

	return 0;
}

void ConvertBack(int argc, char** argv)
{
	float* p = new float[ELEMENTS_COUNT];
	FILE* in = fopen(argv[1], "rb");
	fread(p, sizeof(float), ELEMENTS_COUNT, in);
	fclose(in);

	std::ofstream out(argv[2], std::ofstream::out);

	for (unsigned int i = 0; i < ELEMENTS_COUNT; i++)
	{
		out << p[i] << " ";
	}

	delete[] p;
}

#include <Eigen/Eigen>

typedef Eigen::Triplet<float> Tri;

void triplet()
{
	std::vector<Tri> triplets;
	triplets.push_back(Tri(4, 1, 0.23));
	triplets.push_back(Tri(2, 3, 0.32));
	triplets.push_back(Tri(1, 1, 0.74));
	triplets.push_back(Tri(2, 1, 0.55));
	triplets.push_back(Tri(1, 4, 0.98));

	Eigen::SparseMatrix<float, Eigen::RowMajor> matrix(5,5);
	matrix.setFromTriplets(triplets.begin(), triplets.end());

	for (int k = 0; k < matrix.outerSize(); ++k)
	{
		for (Eigen::SparseMatrix<float, Eigen::RowMajor>::InnerIterator it(matrix, k); it; ++it)
		{
			std::cout << it.row() << "\t";
			std::cout << it.col() << "\t";
			std::cout << it.value() << std::endl;
		}
	}
}

int main(int argc, char** argv)
{
	Eigen::AngleAxisf a(0, Eigen::Vector3f(0, 0, 0));
	Eigen::Matrix3f m(a.toRotationMatrix());
	Eigen::Translation3f t(1, 2, 3);
	//Eigen::Matrix4f m2 = t.
	return 0;
}
