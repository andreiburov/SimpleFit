#include "Image.h"

namespace smpl
{
	RGBTRIPLE WHITE = { 255, 255, 255 };
	RGBTRIPLE BLUE = { 255, 0, 0 };
	RGBTRIPLE GREEN = { 0, 255, 0 };
	RGBTRIPLE RED = { 0, 0, 255 };

	Image::Image()
		: width_(IMAGE_WIDTH), height_(IMAGE_HEIGHT), bpp_(24)
	{
		FreeImage_Initialise();
		bitmap_ = FreeImage_Allocate(width_, height_, bpp_);

		if (!bitmap_)
		{
			std::cerr << "Could not allocate memory for the image.\n";
			exit(1);
		}
	}

	Image::Image(const int width, const int height)
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

	Image::Image(const char* filename, const int width, const int height)
		: width_(width), height_(height), bpp_(24)
	{
		FreeImage_Initialise();
		bitmap_ = FreeImage_Load(FIF_PNG, filename, PNG_DEFAULT);

		if (width_ != FreeImage_GetWidth(bitmap_))
		{
			std::cerr << "Overlay width does not match the image width.\n";
			exit(1);
		}

		if (height_ != FreeImage_GetHeight(bitmap_))
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

	Image::Image(const char* filename)
		: width_(IMAGE_WIDTH), height_(IMAGE_HEIGHT), bpp_(24)
	{
		FreeImage_Initialise();
		bitmap_ = FreeImage_Load(FIF_PNG, filename, PNG_DEFAULT);

		if (width_ != FreeImage_GetWidth(bitmap_))
		{
			std::cerr << "Overlay width does not match the image width.\n";
			exit(1);
		}

		if (height_ != FreeImage_GetHeight(bitmap_))
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

	RGBTRIPLE* Image::operator[](int i)
	{
		if (i < 0)
		{
			MessageBoxA(NULL, "Out of bounds!", "Error", MB_OK);
			i = 0;
		}
		else if (i >= height_)
		{
			MessageBoxA(NULL, "Out of bounds!", "Error", MB_OK);
			i = height_ - 1;
		}

		RGBTRIPLE* scan_line = (RGBTRIPLE*)FreeImage_GetScanLine(bitmap_, height_ - i - 1);
		return scan_line;
	}

	void Image::Draw3D(Image& image, const Eigen::Matrix3f& intrinsics,
		const Eigen::Vector3f& scaling, const Eigen::Vector3f& translation,
		const RGBTRIPLE& color, const int brush_size, const std::vector<float3>& pointcloud)
	{
		int w = image.GetWidth();
		int h = image.GetHeight();

		for (auto& point : pointcloud)
		{
			Eigen::Vector3f p = intrinsics * (Eigen::Scaling(scaling) * point.ToEigen() + translation);
			p /= p(2);
			int x_c = p(0);
			int y_c = p(1);

			for (int x = x_c - brush_size; x < x_c + brush_size + 1; x++)
			{
				for (int y = y_c - brush_size; y < y_c + brush_size + 1; y++)
				{
					if ((x >= 0) && (x < w) && (y >= 0) && (y < h))
					{
						image[int(y)][int(x)] = color;
					}
				}
			}
		}
	}
}

