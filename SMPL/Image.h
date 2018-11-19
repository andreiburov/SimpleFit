#pragma once
#include <Windows.h>
#include <FreeImage.h>
#include <iostream>
#include <string>
#include <Eigen/Eigen>
#include <vector>
#include "Definitions.h"
#include "Projector.h"

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
	struct Pixel
	{
		BYTE& b() { return v.rgbtBlue; }
		BYTE& g() { return v.rgbtGreen; }
		BYTE& r() { return v.rgbtRed; }
		const BYTE& bc() const { return v.rgbtBlue; }
		const BYTE& gc() const { return v.rgbtGreen; }
		const BYTE& rc() const { return v.rgbtRed; }

		Pixel(BYTE red, BYTE green, BYTE blue)
		{
			v = { blue, green, red };
		}

		bool IsBlack() const
		{
			return (bc() == 0U && gc() == 0U && rc() == 0U);
		}

		bool IsWhite() const
		{
			return (bc() == 255U && gc() == 255U && rc() == 255U);
		}

		float GrayScale() const
		{
			float gray = (0.3f * static_cast<float>(rc() / 255) +
				(0.59f * static_cast<float>(gc() / 255)) +
				(0.11f * static_cast<float>(bc() / 255)));
			return std::max(0.f, std::min(1.f, gray));
		}

		bool operator==(const Pixel& rgb) const
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

		static Pixel White()
		{
			return Pixel(255, 255, 255);
		}

		static Pixel Black()
		{
			return Pixel(0, 0, 0);
		}

		static Pixel Red()
		{
			return Pixel(255, 0, 0);
		}

		static Pixel Green()
		{
			return Pixel(0, 255, 0);
		}

		static Pixel Blue()
		{
			return Pixel(0, 0, 255);
		}

		static Pixel Yellow()
		{
			return Pixel(255, 255, 0);
		}

		RGBTRIPLE v;
	};
#endif
#ifdef USE_32_BITS_PER_PIXEL
	struct Pixel
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

	extern Pixel WHITE;
	extern Pixel RED;
	extern Pixel GREEN;
	extern Pixel BLUE;
	extern Pixel BLACK;
	extern Pixel YELLOW;

	class Image
	{
	public:
		Image();

		Image(const int width, const int height);

		Image(const char* filename, const int width, const int height);

		Image(const char* filename);

		Image(const Image& other);

		Image(Image&& other);

		~Image();

		Image& operator=(const Image& other);

		Image& operator=(Image&& other);

		Image& operator=(FIBITMAP* other);

		bool operator==(const Image& other) const;
	
		// first access y then access x!
		Pixel* operator[](int i);

		Pixel operator()(int x, int y) const;

		Pixel& operator()(int x, int y);

		static inline Eigen::Vector2f& Coordinate(Eigen::Vector2f&& point)
		{
			point.y() = IMAGE_HEIGHT - point.y();
			return point;
		}

		static inline std::vector<float>& Coordinates(std::vector<float>&& points)
		{
			for (size_t i = 1; i < points.size(); i += 2)
			{
				points[i] = IMAGE_HEIGHT - points[i];
			}

			return points;
		}

		static inline Eigen::Vector2f& Jacobian(Eigen::Vector2f&& point)
		{
			point.y() = -point.y();
			return point;
		}

		static void Draw3D(Image& image, const Pixel& color, const int brush_size, 
			const Projector& projector, const Eigen::Vector3f& translation,
			const std::vector<float3>& pointcloud);

		static void Draw3D(Image& image, const Pixel& color, 
			const Projector& projector, const Eigen::Vector3f& translation,
			const std::vector<float3>& pointcloud)
		{
			Draw3D(image, color, 0, 
				projector, translation, pointcloud);
		}

		static void Draw2D(Image& image, const Pixel& color, const int brush_size, 
			const std::vector<float>& points);

		static void Draw2D(Image& image, const Pixel& color,
			const std::vector<float>& points)
		{
			Draw2D(image, color, 1, points);
		}

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