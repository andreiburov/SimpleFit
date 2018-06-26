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
	extern RGBTRIPLE WHITE;
	extern RGBTRIPLE RED;
	extern RGBTRIPLE GREEN;
	extern RGBTRIPLE BLUE;

	class Image
	{
	public:
		Image(const int width, const int height)
			: width_(width), height_(height), bpp_(24)
		{
			FreeImage_Initialise();
			bitmap_ = FreeImage_Allocate(width_, height_, bpp_);

			if (!bitmap_)
			{
				std::cerr << "Could not allocate memory for the image.\n";
				exit(1);
			}
		}

		Image(const char* filename, const int width, const int height)
			: width_(width), height_(height), bpp_(24)
		{
			FreeImage_Initialise();
			bitmap_ = FreeImage_Load(FIF_PNG, filename, PNG_DEFAULT);
			
			if (width != FreeImage_GetWidth(bitmap_))
			{
				std::cerr << "Overlay width does not match the image width.\n";
				exit(1);
			}

			if (height != FreeImage_GetHeight(bitmap_))
			{
				std::cerr << "Overlay height does not match the image height.\n";
				exit(1);
			}

			if (!bitmap_)
			{
				std::cerr << "Could not allocate memory for the image.\n";
				exit(1);
			}
		}

		// first access y then access x!
		RGBTRIPLE* operator[](int i)
		{
			if (i < 0)
			{
				std::cout << "Out of bounds!\n";
				i = 0;
			}
			else if (i >= height_)
			{
				std::cout << "Out of bounds!\n";
				i = height_ - 1;
			}

			RGBTRIPLE* scan_line = (RGBTRIPLE*)FreeImage_GetScanLine(bitmap_, height_ - i - 1);
			return scan_line;
		}

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