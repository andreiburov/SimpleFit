#include "Image.h"

namespace smpl
{
	FreeImageLibrary free_image_library;

#ifdef USE_24_BITS_PER_PIXEL
	PIXEL WHITE = { 255, 255, 255 };
	PIXEL BLUE = { 255, 0, 0 };
	PIXEL GREEN = { 0, 255, 0 };
	PIXEL RED = { 0, 0, 255 };
	PIXEL BLACK = { 0, 0, 0 };
	PIXEL YELLOW = { 0, 255, 255 };

	const int BPP = 24;
#endif
#ifdef USE_32_BITS_PER_PIXEL
	PIXEL WHITE = { 255, 255, 255, 0 };
	PIXEL BLUE = { 255, 0, 0, 0 };
	PIXEL GREEN = { 0, 255, 0, 0 };
	PIXEL RED = { 0, 0, 255, 0 };
	PIXEL BLACK = { 0, 0, 0, 0 };
	PIXEL YELLOW = { 0, 255, 255, 0 };

	const int BPP = 32;
#endif

	Image::Image()
		: width_(IMAGE_WIDTH), height_(IMAGE_HEIGHT), bpp_(BPP)
	{
		
		bitmap_ = FreeImage_Allocate(width_, height_, bpp_);

		if (!bitmap_)
		{
			std::cerr << "Could not allocate memory for the image.\n";
			exit(1);
		}
	}

	Image::Image(const int width, const int height)
		: width_(width), height_(height), bpp_(BPP)
	{
		bitmap_ = FreeImage_Allocate(width_, height_, bpp_);

		if (!bitmap_)
		{
			std::cerr << "Could not allocate memory for the image.\n";
			exit(1);
		}
	}

	Image::Image(const char* filename, const int width, const int height)
		: width_(width), height_(height), bpp_(BPP)
	{
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
	{
		bitmap_ = FreeImage_Load(FIF_PNG, filename, PNG_DEFAULT);

		if (!bitmap_)
		{
			MessageBoxA(nullptr, "Not allocated memory for the image", "Error", MB_OK);
		}

		width_ = FreeImage_GetWidth(bitmap_);
		height_ = FreeImage_GetHeight(bitmap_);
		bpp_ = FreeImage_GetBPP(bitmap_);
	}

	Image::Image(const Image& other) : 
		width_(other.width_), height_(other.height_), bpp_(other.bpp_)
	{
		bitmap_ = FreeImage_Clone(other.bitmap_);
	}

	Image::~Image()
	{
		FreeImage_Unload(bitmap_);
	}

	Image& Image::operator=(const Image& other)
	{
		if (this == &other) return *this;

		FreeImage_Unload(bitmap_);
		bitmap_ = FreeImage_Clone(other.bitmap_);
		width_ = other.width_;
		height_ = other.height_;
		bpp_ = other.bpp_;

		return *this;
	}

	Image& Image::operator=(FIBITMAP* other)
	{
		FreeImage_Unload(bitmap_);
		bitmap_ = other;
		return *this;
	}

	bool Image::operator==(const Image& other) const
	{
		if (width_ != other.width_) return false;
		if (height_ != other.height_) return false;
		if (bpp_ != other.bpp_) return false;

		for (int y = 0; y < height_; y++)
		{
			for (int x = 0; x < width_; x++)
			{
				if (!(operator()(x,y) == other(x,y))) return false;
			}
		}

		return true;
	}

	PIXEL* Image::operator[](int i)
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

		PIXEL* scan_line = (PIXEL*)FreeImage_GetScanLine(bitmap_, height_ - i - 1);
		return scan_line;
	}

	PIXEL Image::operator()(int x, int y) const
	{
		if (x < 0 || x >= IMAGE_WIDTH || y < 0 || y >= IMAGE_HEIGHT)
		{
			MessageBoxA(NULL, "Out of bounds!", "Error", MB_OK);
			x = 0;
			y = 0;
		}

		PIXEL* scan_line = (PIXEL*)FreeImage_GetScanLine(bitmap_, height_ - y - 1);
		return scan_line[x];
	}

	PIXEL& Image::operator()(int x, int y)
	{
		if (x < 0 || x >= IMAGE_WIDTH || y < 0 || y >= IMAGE_HEIGHT)
		{
			//MessageBoxA(NULL, "Out of bounds!", "Error", MB_OK);
			std::cout << "Out of bounds\n";
			x = 0;
			y = 0;
		}

		PIXEL* scan_line = (PIXEL*)FreeImage_GetScanLine(bitmap_, height_ - y - 1);
		return scan_line[x];
	}

	void Image::Draw3D(Image& image, const Projector& projector, const Eigen::Vector3f& translation,
		const PIXEL& color, const int brush_size, const std::vector<float3>& pointcloud)
	{
		int w = image.GetWidth();
		int h = image.GetHeight();

		for (int i = 0; i < pointcloud.size(); i++)
		{
			Eigen::Vector2f p = projector(pointcloud[i].ToEigen(), translation);
			int x_c = (int)p(0);
			// flip the y, since DirectX has different image axes than FreeImage
			int y_c = IMAGE_HEIGHT-(int)p(1); 

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

	void Image::Draw2D(Image& image, const PIXEL& color, const int brush_size, const std::vector<float>& points)
	{
		int w = image.GetWidth();
		int h = image.GetHeight();

		size_t size = points.size() / 2;
		for (uint i = 0; i < size; i++)
		{
			int x_c = (int)points[2*i];
			int y_c = (int)points[2*i+1];

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