#include "SilhouetteOptimizer.h"

#include <stdexcept>
#include <cmath>

namespace smpl
{
	const int MAX_DIST = 30;

	template <typename T> 
	struct Point
	{
		union
		{
			struct { T x; T y; };
			T m[2];
		};

		bool is_defined;

		Point(T x, T y) : x(x), y(y), is_defined(true) {}
		Point() : is_defined(false) {}

		const T& operator[](int i) const
		{
			if (i < 0 && i > 1) throw std::out_of_range("Point has only two components");
			return m[i];
		}

		T& operator[](int i)
		{
			if (i < 0 && i > 1) throw std::out_of_range("Point has only two components");
			return m[i];
		}

		Point normalized() const
		{
			Point point;
			float norm = sqrt(static_cast<float>(x * x + y * y));
			point.x = x / norm;
			point.y = y / norm;
			return point;
		}

		bool IsDefined() const { return is_defined; }
	};

	bool IsBorderingPixelBlack(Image& image, int i, int j)
	{
		if (image[j][max(0, i - 1)].IsBlack() ||
			image[j][min(IMAGE_WIDTH-1, i + 1)].IsBlack() ||
			image[max(0, j - 1)][i].IsBlack() ||
			image[min(IMAGE_HEIGHT-1, j + 1)][i].IsBlack())
			return true;
		else
			return false;
	}

	Point<int> Bresenham(Image& input_clean, Image& model_clean, const Point<int>& p0, 
		const Point<int>& p1, bool painted)
	{
		if (!p1.IsDefined()) return Point<int>();

		// ensure integer coordinates
		int x0 = p0[0];
		int y0 = p0[1];
		int x1 = p1[0];
		int y1 = p1[1];

		// compute deltas and update directions
		float dx = abs(static_cast<float>(x1 - x0));
		int sx = x0 < x1 ? 1 : -1;
		float dy = -abs(static_cast<float>(y1 - y0));
		int sy = y0 < y1 ? 1 : -1;
		float err = dx + dy;
		float e2 = 0; // error value e_xy

		// set initial coordinates
		int x = x0;
		int y = y0;

		// start loop to set nPixels
		int nPixels = static_cast<int>(max(dx, -dy));
		for (int i = 0; i < nPixels; ++i)
		{
			if (painted) model_clean[y][x] = BLUE;

			// if pixel is set break
			PIXEL& check = input_clean[y][x];
			//std::cout << check.r() << " " << check.g() << " " << check.b() << std::endl;
			if (input_clean[y][x].IsBlack()) return Point<int>(x, y);
			if (x < 0 || y < 0 || x >= IMAGE_WIDTH || y >= IMAGE_HEIGHT) return Point<int>();
			// update error
			e2 = 2 * err;
			// update coordinates depending on the error
			if (e2 > dy) { err += dy; x += sx; } /* e_xy+e_x > 0 */
			if (e2 < dx) { err += dx; y += sy; } /* e_xy+e_y < 0 */
		}

		return Point<int>();
	}

	void AddCorrespondence(Image& input_clean, Image& model_clean, const Point<int>& point, const Point<float>& normal,
		int max_dist, std::vector<Point<int> >& input_border, std::vector<int>& l2_distance_squared)
	{
		Point<int> x1(static_cast<int>(point[0] + max_dist * normal[0]), static_cast<int>(point[1] + max_dist * normal[1]));
		Point<int> x2(static_cast<int>(point[0] - max_dist * normal[0]), static_cast<int>(point[1] - max_dist * normal[1]));

		Point<int> c1 = Bresenham(input_clean, model_clean, point, x1, false);
		Point<int> c2 = Bresenham(input_clean, model_clean, point, x2, false);

		int l1_2 = -1;
		if (c1.IsDefined()) l1_2 = (point[0] - c1[0]) * (point[0] - c1[0]) + (point[1] - c1[1]) * (point[1] - c1[1]);
		int l2_2 = -1;
		if (c2.IsDefined()) l2_2 = (point[0] - c2[0]) * (point[0] - c2[0]) + (point[1] - c2[1]) * (point[1] - c2[1]);

		if (l1_2 > 0 && l2_2 > 0)
		{
			if (l1_2 > l2_2)
			{
				input_border.push_back(c2);
				l2_distance_squared.push_back(l2_2);
			}
			else
			{
				input_border.push_back(c1);
				l2_distance_squared.push_back(l1_2);
			}
		}
		else if (l1_2 > 0)
		{
			input_border.push_back(c1);
			l2_distance_squared.push_back(l1_2);
		}
		else if (l2_2 > 0)
		{
			input_border.push_back(c2);
			l2_distance_squared.push_back(l2_2);
		}
		else
		{
			input_border.push_back(Point<int>());
			l2_distance_squared.push_back(-1);
		}
	}

	void SilhouetteOptimizer::FindCorrespondences(Image& input, Image& model, std::vector<float4>& normals)
	{
		Image input_clean, model_clean;
		std::vector<Point<int> > model_border;
		std::vector<Point<int> > input_border;
		std::vector<int> l2_distance_squared;

		// Create the clean silhouette from the input
		for (int j = 0; j < IMAGE_HEIGHT; j++)
		{
			for (int i = 0; i < IMAGE_WIDTH; i++)
			{
				if (!input[j][i].IsBlack())
					input_clean[j][i] = WHITE;

				if (!model[j][i].IsBlack())
					if (IsBorderingPixelBlack(model, i, j))
					{
						model_clean[j][i] = WHITE;
						model_border.push_back(Point<int>(i, j));
					}
					else model_clean[j][i] = BLACK;
			}
		}

		input_clean.SavePNG("input_clean.png");
		model_clean.SavePNG("model_clean.png");

		for (auto& point : model_border)
		{
			float4 normal4 = normals[point.y*IMAGE_WIDTH + point.x];
			Point<float> normal = Point<float>(normal4.x, -normal4.y).normalized();
			AddCorrespondence(input_clean, model_clean, point, normal, MAX_DIST, input_border, l2_distance_squared);
		}

		for (int i = 0; i < model_border.size(); i++)
		{
			Bresenham(input_clean, model_clean, model_border[i], input_border[i], true);
		}

		model_clean.SavePNG("correspondences.png");
	}
}