#pragma once
#include <FreeImage.h>
#include <iostream>
#include <string>

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

	// first access y then access x!
	RGBTRIPLE* operator[](unsigned i)
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