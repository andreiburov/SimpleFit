#pragma once
#include <Windows.h>
#include <FreeImage.h>
#include <iostream>
#include <string>
#include <Eigen/Eigen>
#include <vector>
#include "Definitions.h"

#define USE_24_BITS_PER_PIXEL
#ifndef USE_24_BITS_PER_PIXEL
#define USE_32_BITS_PER_PIXEL
#endif // !USE_24_BITS_PER_PIXEL

namespace smpl
{
	const int IMAGE_WIDTH = 640;
	const int IMAGE_HEIGHT = 480;

	struct FreeImageLibrary
	{
		FreeImageLibrary()
		{
			FreeImage_Initialise();
		}

		~FreeImageLibrary()
		{
			FreeImage_DeInitialise();
		}
	};

	extern FreeImageLibrary free_image_library;

#ifdef USE_24_BITS_PER_PIXEL
	struct PIXEL
	{
		BYTE& b() { return v.rgbtBlue; }
		BYTE& g() { return v.rgbtGreen; }
		BYTE& r() { return v.rgbtRed; }
		const BYTE& bc() const { return v.rgbtBlue; }
		const BYTE& gc() const { return v.rgbtGreen; }
		const BYTE& rc() const { return v.rgbtRed; }

		bool IsBlack() const
		{
			return (bc() == 0U && gc() == 0U && rc() == 0U);
		}

		bool IsWhite(const PIXEL& rgb) const
		{
			return (bc() == 255U && gc() == 255U && rc() == 255U);
		}

		bool operator==(const PIXEL& rgb) const
		{
			if (bc() == rgb.bc() && gc() == rgb.gc() && rc() == rgb.rc())
			{
				return true;
			}
			else
			{
				return false;
			}
		}

		RGBTRIPLE v;
	};
#endif
#ifdef USE_32_BITS_PER_PIXEL
	struct PIXEL
	{
		BYTE& b() { return v.rgbBlue; }
		BYTE& g() { return v.rgbGreen; }
		BYTE& r() { return v.rgbRed; }
		BYTE& a() { return v.rgbReserved; }
		const BYTE& bc() const { return v.rgbBlue; }
		const BYTE& gc() const { return v.rgbGreen; }
		const BYTE& rc() const { return v.rgbRed; }
		const BYTE& ac() const { return v.rgbReserved; }

		bool IsBlack() const
		{
			return (bc() == 0U && gc() == 0U && rc() == 0U);
		}

		bool IsWhite(const PIXEL& rgb) const
		{
			return (bc() == 255U && gc() == 255U && rc() == 255U);
		}

		RGBQUAD v;
	};
#endif

	extern PIXEL WHITE;
	extern PIXEL RED;
	extern PIXEL GREEN;
	extern PIXEL BLUE;
	extern PIXEL BLACK;
	extern PIXEL YELLOW;

	class Image
	{
	public:
		Image();

		Image(const int width, const int height);

		Image(const char* filename, const int width, const int height);

		Image(const char* filename);

		Image(const Image& other);

		~Image();

		Image& operator=(const Image& other);

		Image& operator=(FIBITMAP* other);

		bool operator==(const Image& other) const;
	
		// first access y then access x!
		PIXEL* operator[](int i);

		PIXEL operator()(int x, int y) const;

		PIXEL& operator()(int x, int y);

		static void Draw3D(Image& image, const Eigen::Matrix3f& intrinsics, 
			const Eigen::Vector3f& translation, 
			const PIXEL& color,	const int brush_size, const std::vector<float3>& pointcloud);

		static void Draw3D(Image& image, const Eigen::Matrix3f& intrinsics,
			const Eigen::Vector3f& translation,
			const PIXEL& color, const std::vector<float3>& pointcloud)
		{
			Draw3D(image, intrinsics, translation, color, 0, pointcloud);
		}

		static void Draw2D(Image& image, const PIXEL& color, const int brush_size, const std::vector<float>& points);

		void SavePNG(const std::string& filename) const
		{
			FreeImage_Save(FIF_PNG, bitmap_, filename.c_str(), 0);
		}

		FIBITMAP* GetBitmap() { return bitmap_; }

		int GetWidth() const { return width_; }

		int GetHeight() const { return height_; }

	private:
		int width_;
		int height_;
		int bpp_; // bits per pixel
		FIBITMAP* bitmap_;
	};
}