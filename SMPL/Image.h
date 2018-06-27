#pragma once
#include <Windows.h>
#include <FreeImage.h>
#include <iostream>
#include <string>
#include <Eigen/Eigen>
#include <vector>
#include "Definitions.h"

namespace smpl
{
	const int IMAGE_WIDTH = 640;
	const int IMAGE_HEIGHT = 480;

	extern RGBTRIPLE WHITE;
	extern RGBTRIPLE RED;
	extern RGBTRIPLE GREEN;
	extern RGBTRIPLE BLUE;

	class Image
	{
	public:
		Image();

		Image(const int width, const int height);

		Image(const char* filename, const int width, const int height);

		Image(const char* filename);
	
		// first access y then access x!
		RGBTRIPLE* operator[](int i);

		static void Draw3D(Image& image, const Eigen::Matrix3f& intrinsics, 
			const Eigen::Vector3f& scaling,	const Eigen::Vector3f& translation, 
			const RGBTRIPLE& color,	const int brush_size, const std::vector<float3>& pointcloud);

		static void Draw3D(Image& image, const Eigen::Matrix3f& intrinsics,
			const Eigen::Vector3f& scaling, const Eigen::Vector3f& translation,
			const RGBTRIPLE& color, const std::vector<float3>& pointcloud)
		{
			Draw3D(image, intrinsics, scaling, translation, color, 0, pointcloud);
		}

		void SavePNG(const std::string& filename)
		{
			FreeImage_Save(FIF_PNG, bitmap_, filename.c_str(), 0);
		}

		int GetWidth() const { return width_; }

		int GetHeight() const { return height_; }

		~Image()
		{
			FreeImage_DeInitialise();
		}

	private:
		const int width_;
		const int height_;
		const int bpp_; // bits per pixel
		FIBITMAP* bitmap_;
	};
}