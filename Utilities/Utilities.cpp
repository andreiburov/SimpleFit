#define _CRT_SECURE_NO_WARNINGS 1 
#define _WINSOCK_DEPRECATED_NO_WARNINGS 1 

#include <cstdio>
#include <iostream>
#include <fstream>
#include <vector>
#include <DirectXMath.h>
#include <FreeImage.h>
#include "Image.h"

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
#include <unsupported/Eigen/EulerAngles>

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

void FreeImageExample()
{
	Image im(10, 2);

	for (int y = 0; y < 1; y++)
	{
		for (int x = 0; x < 5; x++)
		{
			im[y][x].rgbtBlue = 255;
			RGBTRIPLE a = im[y][x];
			a.rgbtBlue = a.rgbtBlue;
		}
	}

	for (int y = 0; y < 2; y++)
	{
		for (int x = 0; x < 10; x++)
		{
			unsigned r = (unsigned)im[y][x].rgbtRed;
			unsigned g = (unsigned)im[y][x].rgbtGreen;
			unsigned b = (unsigned)im[y][x].rgbtBlue;
			std::cout << " (" << r << "," << g << "," << b << "),";
		}
		std::cout << std::endl;
	}

	im.SavePNG("hello.png");
	system("pause");
}

Eigen::MatrixXf derivative(const Eigen::Vector3f& t)
{
	float z2 = t(2) * t(2);
	float r[6] = { 1.f / t(2), 0, 1.f * t(0) / z2, 0, 1.f / t(2), 1.f * t(1) / z2 };
	return Eigen::Map<Eigen::MatrixXf, Eigen::RowMajor>(r, 3, 2);
}

//Eigen::Vector3f Skinning(Eigen::Vector3f& x, float alpha, float beta, float gamma, Eigen::Vector3f t)
//{
//	Eigen::Vector3f r1 = Eigen::Translation3f(t) * Eigen::EulerAngles<float, Eigen::EulerSystemXYZ>(alpha, beta, gamma).toRotationMatrix() * Eigen::Translation3f(-t) * x;
//
//	float rX[] = { 1,0,0,0,cos(alpha),-sin(alpha),0,sin(alpha),cos(alpha) };
//	float rY[] = { cos(beta),0,sin(beta),0,1,0,-sin(beta),0,cos(beta) };
//	float rZ[] = { cos(gamma),-sin(gamma),0,sin(gamma),cos(gamma),0,0,0,1 };
//
//	Eigen::Matrix3f rotX = Eigen::Map<Eigen::Matrix3f, Eigen::RowMajor>(rX, 3, 3).transpose();
//	Eigen::Matrix3f rotY = Eigen::Map<Eigen::Matrix3f, Eigen::RowMajor>(rY, 3, 3).transpose();
//	Eigen::Matrix3f rotZ = Eigen::Map<Eigen::Matrix3f, Eigen::RowMajor>(rZ, 3, 3).transpose();
//
//	Eigen::Vector3f r2 = Eigen::Translation3f(t) * rotZ * rotY * rotX * Eigen::Translation3f(-t) * x;
//
//	return r1;
//}
//
//Eigen::Vector3f SkinningDerivative(Eigen::Vector3f& x, float alpha, float beta, float gamma, Eigen::Vector3f t)
//{
//	float rX[] = { 1,0,0,0,cos(alpha),-sin(alpha),0,sin(alpha),cos(alpha) };
//	float rY[] = { cos(beta),0,sin(beta),0,1,0,-sin(beta),0,cos(beta) };
//	float rZ[] = { cos(gamma),-sin(gamma),0,sin(gamma),cos(gamma),0,0,0,1 };
//
//	Eigen::Matrix3f rotX = Eigen::Map<Eigen::Matrix3f, Eigen::RowMajor>(rX, 3, 3).transpose();
//	Eigen::Matrix3f rotY = Eigen::Map<Eigen::Matrix3f, Eigen::RowMajor>(rY, 3, 3).transpose();
//	Eigen::Matrix3f rotZ = Eigen::Map<Eigen::Matrix3f, Eigen::RowMajor>(rZ, 3, 3).transpose();
//
//	float drX[] = { 1,0,0,0,-sin(alpha),-cos(alpha),0,cos(alpha),-sin(alpha) };
//	float drY[] = { -sin(beta),0,cos(beta),0,1,0,-cos(beta),0,-sin(beta) };
//	float drZ[] = { -sin(gamma),-cos(gamma),0,cos(gamma),-sin(gamma),0,0,0,1 };
//
//	Eigen::Matrix3f drotX = Eigen::Map<Eigen::Matrix3f, Eigen::RowMajor>(drX, 3, 3).transpose();
//	Eigen::Matrix3f drotY = Eigen::Map<Eigen::Matrix3f, Eigen::RowMajor>(drY, 3, 3).transpose();
//	Eigen::Matrix3f drotZ = Eigen::Map<Eigen::Matrix3f, Eigen::RowMajor>(drZ, 3, 3).transpose();
//
//	Eigen::Transform<float, 3, Eigen::Affine> t1 = Eigen::Translation3f(t) * Eigen::EulerAngles<float, Eigen::EulerSystemXYZ>(alpha, beta, gamma).toRotationMatrix() * Eigen::Translation3f(-t);
//	Eigen::Transform<float, 3, Eigen::Affine> t2 = Eigen::Translation3f(t) * rotZ * rotY * rotX * Eigen::Translation3f(-t);
//
//	return x;
//}

void RotationCheck()
{
	float alpha = -0.5f;
	float beta = 0.33f;
	float gamma = -0.7f;

	Eigen::Vector3f t(-1, 2, 3);
	Eigen::Matrix3f e = Eigen::EulerAngles<float, Eigen::EulerSystemZYX>(alpha, beta, gamma).toRotationMatrix();
	Eigen::Matrix3f aX = Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX()).matrix();
	Eigen::Matrix3f aY = Eigen::AngleAxisf(beta, Eigen::Vector3f::UnitY()).matrix();
	Eigen::Matrix3f aZ = Eigen::AngleAxisf(gamma, Eigen::Vector3f::UnitZ()).matrix();
	Eigen::Matrix3f a = aZ * aY * aX;

	Eigen::Transform<float, 3, Eigen::Affine> t1 = Eigen::Translation3f(t) * a * Eigen::Translation3f(-t);
	Eigen::Transform<float, 3, Eigen::Affine> t2 = Eigen::Translation3f(t) * aZ * Eigen::Translation3f(-t) * Eigen::Translation3f(t) * aY * Eigen::Translation3f(-t) * Eigen::Translation3f(t) * aX * Eigen::Translation3f(-t);

	Eigen::Matrix4f m1 = t1.matrix();
	Eigen::Matrix4f m2 = t2.matrix();

	std::cout << "!\n";
}

int main(int argc, char** argv)
{
	RotationCheck();
	
	return 0;
}
