#define _CRT_SECURE_NO_WARNINGS 1 
#define _WINSOCK_DEPRECATED_NO_WARNINGS 1 

#include <cstdio>
#include <iostream>
#include <fstream>
#include <DirectXMath.h>

#include "Utilities.h"

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