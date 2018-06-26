#include "Image.h"

namespace smpl
{
	RGBTRIPLE WHITE = { 255, 255, 255 };
	RGBTRIPLE BLUE = { 255, 0, 0 };
	RGBTRIPLE GREEN = { 0, 255, 0 };
	RGBTRIPLE RED = { 0, 0, 255 };

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

