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

		bool IsDefined() { return !is_defined; }
	};

	bool IsBlack(const RGBTRIPLE& rgb)
	{
		return (rgb.rgbtBlue == 0U && rgb.rgbtGreen == 0U && rgb.rgbtRed == 0U);
	}

	bool IsWhite(const RGBTRIPLE& rgb)
	{
		return (rgb.rgbtBlue == 255U && rgb.rgbtGreen == 255U && rgb.rgbtRed == 255U);
	}

	bool IsBorderingPixelBlack(Image& image, int i, int j)
	{
		if (IsBlack(image[j][max(0, i - 1)]) ||
			IsBlack(image[j][min(IMAGE_WIDTH-1, i + 1)]) ||
			IsBlack(image[max(0, j - 1)][i]) ||
			IsBlack(image[min(IMAGE_HEIGHT-1, j + 1)][i]))
			return true;
		else
			return false;
	}

	Point<int> Bresenham(Image& input_clean, Image& model_clean, const Point<int>& p0, const Point<int>& p1)
	{
		// ensure integer coordinates
		int x0 = p0[0];
		int y0 = p0[1];
		int x1 = p1[0];
		int y1 = p1[1];

		// compute deltas and update directions
		float dx = abs(x1 - x0);
		int sx = x0 < x1 ? 1 : -1;
		float dy = -abs(y1 - y0);
		int sy = y0 < y1 ? 1 : -1;
		float err = dx + dy;
		float e2 = 0; // error value e_xy

		// set initial coordinates
		int x = x0;
		int y = y0;

		// start loop to set nPixels
		int nPixels = max(dx, -dy);
		for (int i = 0; i < nPixels; ++i)
		{
			model_clean[y][x] = BLUE;

			// if pixel is set break
			RGBTRIPLE& check = input_clean[y][x];
			std::cout << check.rgbtRed << " " << check.rgbtGreen << " " << check.rgbtBlue << std::endl;
			if (IsWhite(input_clean[y][x])) return Point<int>(x, y);
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
		int max_dist, std::vector<Point<int> >& input_border, std::vector<int>& correspondance_l2)
	{
		Point<int> x1(point[0] + max_dist * normal[0], point[1] + max_dist * normal[1]);
		Point<int> x2(point[0] - max_dist * normal[0], point[1] - max_dist * normal[1]);

		Point<int> c1 = Bresenham(input_clean, model_clean, point, x1);
		Point<int> c2 = Bresenham(input_clean, model_clean, point, x2);

		int l1_2 = -1;
		if (c1.IsDefined()) l1_2 = (point[0] - c1[0]) * (point[0] - c1[0]) + (point[1] - c1[1]) * (point[1] - c1[1]);
		int l2_2 = -1;
		if (c2.IsDefined()) l2_2 = (point[0] - c2[0]) * (point[0] - c2[0]) + (point[1] - c2[1]) * (point[1] - c2[1]);

		if (l1_2 > 0 && l2_2 > 0)
		{
			if (l1_2 > l2_2)
			{
				input_border.push_back(c2);
				correspondance_l2.push_back(l2_2);
			}
			else
			{
				input_border.push_back(c1);
				correspondance_l2.push_back(l1_2);
			}
		}
		else if (l1_2 > 0)
		{
			input_border.push_back(c1);
			correspondance_l2.push_back(l1_2);
		}
		else if (l2_2 > 0)
		{
			input_border.push_back(c2);
			correspondance_l2.push_back(l2_2);
		}
		else
		{
			input_border.push_back(Point<int>());
			correspondance_l2.push_back(-1);
		}
	}

	//void SilhouetteOptimizer::FindCorrespondances(Image& input, Image& model)
	//{
	//	Image input_clean, model_clean;
	//	std::vector<Point<int> > model_border;
	//	std::vector<Point<int> > input_border;
	//	std::vector<int> correspondance_l2;

	//	// Create the clean silhouette from the input
	//	for (int j = 0; j < IMAGE_HEIGHT; j++)
	//	{
	//		for (int i = 0; i < IMAGE_WIDTH; i++)
	//		{
	//			if (!IsBlack(input[j][i]))
	//				if (IsBorderingPixelBlack(input, i, j))	input_clean[j][i] = WHITE;
	//				else input_clean[j][i] = BLACK;

	//			if (!IsBlack(model[j][i]))
	//				if (IsBorderingPixelBlack(model, i, j))
	//				{
	//					model_clean[j][i] = WHITE;
	//					model_border.push_back(Point<int>(i, j));
	//				}
	//				else model_clean[j][i] = BLACK;
	//		}
	//	}

	//	input_clean.SavePNG("input_clean.png");
	//	model_clean.SavePNG("model_clean.png");

	//	for (auto& point : model_border)
	//	{
	//		float red = model[point.y][point.x].rgbtRed / 255.f;
	//		float green = model[point.y][point.x].rgbtGreen / 255.f;
	//		Point<float> normal(red*2.f - 1.f, 1.f - green * 2.f);
	//		AddCorrespondence(input_clean, model_clean, point, normal, MAX_DIST, input_border, correspondance_l2);
	//	}

	//	model_clean.SavePNG("correspondances.png");
	//}


	void SilhouetteOptimizer::FindCorrespondances(Image& input, Image& model)
	{
		Image clean, result;

		// Create the clean silhouette from the input
		for (int j = 0; j < IMAGE_HEIGHT; j++)
		{
			for (int i = 0; i < IMAGE_WIDTH; i++)
			{
				if (!IsBlack(input[j][i]))
				{
					result[j][i] = WHITE;
					clean[j][i] = WHITE;
				}
			}
		}

		result.SavePNG("clean_silhouette.png");

		// Bresenham marching for every border pixel
		int marching_counter = 0;
		for (int j = 0; j < IMAGE_HEIGHT; j++)
		{
			for (int i = 0; i < IMAGE_WIDTH; i++)
			{
				if (IsBlack(model[j][i])) continue;
				if (!IsBorderingPixelBlack(model, i, j)) continue;
				
				// working on the boundary
				//result[j][i] = RED;


				int x0 = i; int y0 = j;
				float red = model[j][i].rgbtRed / 255.f;
				float green = model[j][i].rgbtGreen / 255.f;
				float deltax = red * 2.f - 1.f;
				float deltay = 1.f - green * 2.f;
				int sign = deltay / abs(deltay); // deltay != 0
				float deltaerr = abs(deltay / deltax);
				float error = 0.f;

				// first pixel
				bool is_previous_black = (IsBlack(clean[j][i])) ? true : false;
				int y = y0;
				
				result[j][i] = RED;
				error = error + deltaerr;
				if (error >= 0.5f)
				{
					y = y + sign;
					error = error - 1.f;
				}
				
				for (int x = x0 + 1; ; x += sign)
				{
					if (x >= IMAGE_WIDTH || x < 0 || y >= IMAGE_HEIGHT || y < 0)
					{
						++marching_counter;
						break;
					}

					result[y][x] = BLUE;
					
					if ((IsBlack(clean[y0][x0]) && !is_previous_black) ||
						(!IsBlack(clean[y0][x0]) && is_previous_black))
					{
						++marching_counter;
						break;
					}

					is_previous_black = (IsBlack(clean[y0][x0])) ? 1 : 0;
					error = error + deltaerr;
					if (error >= 0.5f)
					{
						y = y + sign;
						error = error - 1.f;
					}
				}

				if (marching_counter >= 1)
				{
					result.SavePNG("correspondances.png");
					return;
				}
			}
		}



		result.SavePNG("correspondances.png");
	}
}