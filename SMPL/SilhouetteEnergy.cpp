#include <stdexcept>
#include <cmath>
#include <functional>
#include <tuple>

#include "SilhouetteEnergy.h"
#include "LevenbergMarquardt.h"

namespace smpl
{
	bool IsBorderingPixelBlack(const Image& image, int i, int j)
	{
		if (image(std::max(0, i - 1), std::max(0, j - 1)).IsBlack() ||
			image(i, std::max(0, j - 1)).IsBlack() ||
			image(std::min(IMAGE_WIDTH - 1, i + 1), std::max(0, j - 1)).IsBlack() ||

			image(std::max(0, i - 1), j).IsBlack() ||
			image(std::min(IMAGE_WIDTH - 1, i + 1), j).IsBlack() ||

			image(std::max(0, i - 1), std::min(IMAGE_HEIGHT - 1, j + 1)).IsBlack() ||
			image(i, std::min(IMAGE_HEIGHT - 1, j + 1)).IsBlack() ||
			image(std::min(IMAGE_WIDTH - 1, i + 1), std::min(IMAGE_HEIGHT - 1, j + 1)).IsBlack()
			)
			return true;
		else
			return false;
	}

	Point<int> Bresenham(
		std::function<bool(int, int)> stop_condition,
		const Point<int>& p0, const Point<int>& p1)
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
		int nPixels = static_cast<int>(std::max(dx, -dy));
		for (int i = 0; i < nPixels; ++i)
		{
			if (x < 0 || y < 0 || x >= IMAGE_WIDTH || y >= IMAGE_HEIGHT) return Point<int>();
			if (stop_condition(x, y)) return Point<int>(x, y);

			// update error
			e2 = 2 * err;
			// update coordinates depending on the error
			if (e2 > dy) { err += dy; x += sx; } /* e_xy+e_x > 0 */
			if (e2 < dx) { err += dx; y += sy; } /* e_xy+e_y < 0 */
		}

		return Point<int>();
	}

	Point<int> Bresenham(std::function<bool(int, int)> stop_condition, 
		const Point<int>& p0, const Point<int>& p1, 
		Image& model, bool painted)
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
		int nPixels = static_cast<int>(std::max(dx, -dy));
		for (int i = 0; i < nPixels; ++i)
		{
			if (x < 0 || y < 0 || x >= IMAGE_WIDTH || y >= IMAGE_HEIGHT) return Point<int>();
			if (painted) model(x, y) = Pixel::Blue();
			if (stop_condition(x, y)) return Point<int>(x, y);
			
			// update error
			e2 = 2 * err;
			// update coordinates depending on the error
			if (e2 > dy) { err += dy; x += sx; } /* e_xy+e_x > 0 */
			if (e2 < dx) { err += dx; y += sy; } /* e_xy+e_y < 0 */
		}

		return Point<int>();
	}

	std::tuple<Point<int>, Point<int>> 
	BresenhamNext(std::function<bool(int, int)> stop_condition,
		const Point<int>& p0, const Point<int>& p1,
		Image& log_marching, bool painted)
	{
		if (!p1.IsDefined()) return std::make_tuple(Point<int>(), Point<int>());

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
		int nPixels = static_cast<int>(std::max(dx, -dy));
		for (int i = 0; i < nPixels; ++i)
		{
			if (x < 0 || y < 0 || x >= IMAGE_WIDTH || y >= IMAGE_HEIGHT) 
				return std::make_tuple(Point<int>(), Point<int>());
			if (painted) log_marching(x, y) = Pixel::Blue();
			
			Point<int> correspondence(x, y);

			// update error
			e2 = 2 * err;
			// update coordinates depending on the error
			if (e2 > dy) { err += dy; x += sx; } /* e_xy+e_x > 0 */
			if (e2 < dx) { err += dx; y += sy; } /* e_xy+e_y < 0 */

			if (stop_condition(correspondence.x, correspondence.y))
				return std::make_tuple(correspondence, Point<int>(x, y));
		}

		return std::make_tuple(Point<int>(), Point<int>());
	}

	void AddCorrespondence(std::function<bool(int, int)> stop_condition,
		const Point<int>& point, const Point<float>& normal, int max_dist,
		std::vector<Point<int> >& model_correspondence,
		std::vector<Point<int> >& input_correspondence,
		std::vector<Point<float> >& distance, Image& model)
	{
		Point<int> x1(static_cast<int>(point[0] + max_dist * normal[0]), static_cast<int>(point[1] + max_dist * normal[1]));
		Point<int> x2(static_cast<int>(point[0] - max_dist * normal[0]), static_cast<int>(point[1] - max_dist * normal[1]));

		// input correspondences
		Point<int> c1 = Bresenham(stop_condition, point, x1, model, false);
		Point<int> c2 = Bresenham(stop_condition, point, x2, model, false);
		Point<float> d1, d2; // distance to x1,x2

		int l1_2 = -1;
		if (c1.IsDefined())
		{
			l1_2 = (point[0] - c1[0]) * (point[0] - c1[0]) + (point[1] - c1[1]) * (point[1] - c1[1]);
			d1 = Point<float>(static_cast<float>(point[0] - c1[0]), static_cast<float>(point[1] - c1[1]));
		}
		int l2_2 = -1;
		if (c2.IsDefined())
		{
			l2_2 = (point[0] - c2[0]) * (point[0] - c2[0]) + (point[1] - c2[1]) * (point[1] - c2[1]);
			d2 = Point<float>(static_cast<float>(point[0] - c2[0]), static_cast<float>(point[1] - c2[1]));
		}

		if (l1_2 > 0 && l2_2 > 0)
		{
			if (l1_2 > l2_2)
			{
				model_correspondence.push_back(point);
				input_correspondence.push_back(c2);
				distance.push_back(d2);
			}
			else
			{
				model_correspondence.push_back(point);
				input_correspondence.push_back(c1);
				distance.push_back(d1);
			}
		}
		else if (l1_2 > 0)
		{
			model_correspondence.push_back(point);
			input_correspondence.push_back(c1);
			distance.push_back(d1);
		}
		else if (l2_2 > 0)
		{
			model_correspondence.push_back(point);
			input_correspondence.push_back(c2);
			distance.push_back(d2);
		}
	}

	SilhouetteEnergy::SilhouetteEnergy(
		const Generator& generator, 
		const Projector& projector, 
		const SilhouetteRenderer& renderer,
		const int ray_dist, const int pruning_derivative_half_dx) :
		generator_(generator), projector_(projector), silhouette_renderer_(renderer),
		ray_dist_(ray_dist), pd_(pruning_derivative_half_dx)
	{
	}

	Correspondences SilhouetteEnergy::FindCorrespondences(
		const Image& input, 
		const Image& model, const std::vector<float4>& normals) const
	{
		Image correspondences(model);
		Image input_contour(input);

		// model border detected visually
		std::vector<Point<int> > model_border;

		// model border with input correspondences
		std::vector<Point<int> > model_correspondence;
		std::vector<Point<int> > input_correspondence;
		std::vector<Point<float> > distance;

		// Create the clean silhouette from the input
		for (int j = 0; j < IMAGE_HEIGHT; j++)
		{
			for (int i = 0; i < IMAGE_WIDTH; i++)
			{
				if (!input(i, j).IsBlack())
					if (IsBorderingPixelBlack(input, i, j))
					{
						input_contour(i, j) = Pixel::White();
					}
					else input_contour(i, j) = Pixel::Black();

				if (!model(i, j).IsBlack())
					if (IsBorderingPixelBlack(model, i, j))
					{
						correspondences(i, j) = Pixel::White();
						model_border.push_back(Point<int>(i, j));
					}
					else correspondences(i, j) = Pixel::Black();
			}
		}

		auto stop_condition = [&input_contour](int x, int y) { return (input_contour(x, y).IsWhite()); };
		for (auto& point : model_border)
		{
			float4 normal4 = normals[point.y*IMAGE_WIDTH + point.x];
			Point<float> normal = Point<float>(normal4.x, -normal4.y).normalized();
			AddCorrespondence(stop_condition, point, normal, ray_dist_,
				model_correspondence, input_correspondence, distance, correspondences);
		}

		// Draw correspondences
		for (int i = 0; i < model_correspondence.size(); i++)
		{		
			Bresenham(stop_condition, model_correspondence[i], input_correspondence[i],
				correspondences, true);
		}

		// Draw input onto correspondences for clarity
		for (int y = 0; y < IMAGE_HEIGHT; y++)
			for (int x = 0; x < IMAGE_WIDTH; x++)
			{
				if (input_contour(x, y).IsWhite())
				{
					correspondences(x, y) = Pixel::Yellow();
				}
			}

		Correspondences result(correspondences, model_correspondence, input_correspondence, distance);
		return result;
	}

	void SilhouetteEnergy::PruneCorrepondences(
		const Image& input,
		const Image& model, const std::vector<float4>& normals,
		Correspondences& correspondences) const
	{
		std::vector<Point<float> > input_normals;
		input_normals.reserve(correspondences.input_border.size());

		Image draw_normals(input);
		auto stop_condition = [](int x, int y) { return false; };

		// Compute the input_border normals
		for (const auto& p : correspondences.input_border)
		{
			// Derivatives in FreeImage space, inverted to point out of the silhouette
			// Taking step to because the contours can have a width of 2
			float dx = (input(p.x + pd_, p.y).GrayScale() - input(p.x - pd_, p.y).GrayScale()) / 2.f;
			float dy = (input(p.x, p.y + pd_).GrayScale() - input(p.x, p.y - pd_).GrayScale()) / 2.f;
			Point<float> n = Point<float>(-dx, -dy).normalized();
			input_normals.push_back(n);

			// Draw the normals
			const int radius = 5;
			Point<int> end(static_cast<int>(p.x + radius * n.x), static_cast<int>(p.y + radius * n.y));
			Bresenham(stop_condition, p, end, draw_normals, true);
		}

		//draw_normals.SavePNG("input_normals.png");

		// Prune the correspondances based on normals
		int n = static_cast<int>(correspondences.input_border.size());
		assert(n == correspondences.model_border.size());
		const float threshold = 0.1f;
		std::vector<int> filtered_indices;
		filtered_indices.reserve(n);

		// Filtering the correpondences
		for (int i = 0; i < n; i++)
		{
			auto& p = correspondences.model_border[i];
			float4 normal4 = normals[p.y*IMAGE_WIDTH + p.x];
			Point<float> n = Point<float>(normal4.x, -normal4.y).normalized();

			Point<float>& n_target = input_normals[i];
			if (n.dot(n_target) > threshold) filtered_indices.push_back(i);
		}

		int filtered_n = static_cast<int>(correspondences.input_border.size());
		std::vector<Point<int> > model_border; model_border.reserve(filtered_n);
		std::vector<Point<int> > input_border; input_border.reserve(filtered_n);
		std::vector<Point<float> > distance; distance.reserve(filtered_n);

		for (const auto& i : filtered_indices)
		{
			model_border.push_back(correspondences.model_border[i]);
			input_border.push_back(correspondences.input_border[i]);
			distance.push_back(correspondences.distance[i]);
		}

		correspondences.model_border = model_border;
		correspondences.input_border = input_border;
		correspondences.distance = distance;

		// Draw correspondences
		Image draw_correspondences(model);
		for (int j = 0; j < IMAGE_HEIGHT; j++)
		{
			for (int i = 0; i < IMAGE_WIDTH; i++)
			{
				if (!model(i, j).IsBlack())
					if (IsBorderingPixelBlack(model, i, j))
						draw_correspondences(i, j) = Pixel::White();
					else draw_correspondences(i, j) = Pixel::Black();
			}
		}

		for (int i = 0; i < model_border.size(); i++)
		{
			Bresenham(stop_condition, model_border[i], input_border[i],
				draw_correspondences, true);
		}

		for (int j = 0; j < IMAGE_HEIGHT; j++)
		{
			for (int i = 0; i < IMAGE_WIDTH; i++)
			{
				if (!input(i, j).IsBlack())
					if (IsBorderingPixelBlack(input, i, j))
					{
						draw_correspondences(i, j) = Pixel::Yellow();
					}
			}
		}

		correspondences.image = draw_correspondences;
	}

	std::tuple<Point<int>, Point<float>>
	SilhouetteEnergy::MarchingPruned(
		const std::function<bool(int, int)>& stop_condition,
		const Point<int>& model_point,
		const Point<float>& model_normal, int max_dist,
		const Image& input_silhouette,
		bool logging_on,
		Image& log_marching,
		Image& log_input_derivatives) const
	{
		Point<int> endpoint(
			static_cast<int>(model_point[0] + max_dist * model_normal[0]),
			static_cast<int>(model_point[1] + max_dist * model_normal[1]));

		Point<int> current, next(model_point);

		while (true)
		{
			std::tie(current, next) = BresenhamNext(stop_condition, next, endpoint, log_marching, logging_on);

			if (!current.IsDefined()) break;
			else 
			{
				// Derivatives in FreeImage space, inverted to point out of the silhouette
				// Taking step to because the contours can have a width of 2
				float dx = (input_silhouette(current.x + pd_, current.y).GrayScale()
					- input_silhouette(current.x - pd_, current.y).GrayScale()) / 2.f;
				float dy = (input_silhouette(current.x, current.y + pd_).GrayScale()
					- input_silhouette(current.x, current.y - pd_).GrayScale()) / 2.f;
				Point<float> input_normal = Point<float>(-dx, -dy).normalized();

				if (logging_on)
				{
					const int radius = 5;
					Point<int> log_end(static_cast<int>(current.x + radius * input_normal.x),
						static_cast<int>(current.y + radius * input_normal.y));
					auto stop_on_endpoint = [](int x, int y) { return false; };
					Bresenham(stop_on_endpoint, current, log_end, log_input_derivatives, true);
				}

				const float threshold = 0.2f;
				if (model_normal.dot(input_normal) > threshold)
				{
					Point<float> distance(
						static_cast<float>(model_point[0] - current[0]),
						static_cast<float>(model_point[1] - current[1]));
					return std::make_tuple(current, distance);
				}
				else
				{
					//std::cout << "!";
				}
			}
		}

		return std::make_tuple(Point<int>(), Point<float>());
	}

	void SilhouetteEnergy::AddCorrespondencePruned(
		std::function<bool(int, int)> stop_condition,
		const Point<int>& model_point,
		const Point<float>& model_normal, int max_dist,
		const Image& input_silhouette,
		std::vector<Point<int>>& model_correspondence,
		std::vector<Point<int>>& input_correspondence,
		std::vector<Point<float>>& distance,
		bool logging_on,
		Image& log_marching,
		Image& log_input_normals) const
	{
		Point<int> c1, c2;		// input_point
		Point<float> d1, d2;	// distance

		std::tie(c1, d1) = MarchingPruned(stop_condition, model_point, model_normal, 
			max_dist, input_silhouette, logging_on, log_marching, log_input_normals);
		std::tie(c2, d2) = MarchingPruned(stop_condition, model_point, model_normal, 
			-max_dist, input_silhouette, logging_on, log_marching, log_input_normals);

		if (!c1.IsDefined() && !c2.IsDefined()) return;
		if (c1.IsDefined() && !c2.IsDefined())
		{
			model_correspondence.push_back(model_point);
			input_correspondence.push_back(c1);
			distance.push_back(d1);
		}
		if (!c1.IsDefined() && c2.IsDefined())
		{
			model_correspondence.push_back(model_point);
			input_correspondence.push_back(c2);
			distance.push_back(d2);
		}
		if (d1.Norm() < d2.Norm())
		{
			model_correspondence.push_back(model_point);
			input_correspondence.push_back(c1);
			distance.push_back(d1);
		}
		else
		{
			model_correspondence.push_back(model_point);
			input_correspondence.push_back(c2);
			distance.push_back(d2);
		}
	}

	Correspondences SilhouetteEnergy::FindCorrespondencesPruned(
		const Image& input_silhouette,
		const Image& model_silhouette, 
		const std::vector<float4>& model_normals,
		const std::string& output_path) const
	{
		bool logging_on = true;
		if (output_path.empty()) logging_on = false;

		Image model_contour(model_silhouette), input_contour(input_silhouette);
		// model border detected visually
		std::vector<Point<int>> model_border;

		// create contours, detect model border
		for (int j = 0; j < IMAGE_HEIGHT; j++)
		{
			for (int i = 0; i < IMAGE_WIDTH; i++)
			{
				if (!input_silhouette(i, j).IsBlack())
					if (IsBorderingPixelBlack(input_silhouette, i, j))
					{
						input_contour(i, j) = Pixel::White();
					}
					else input_contour(i, j) = Pixel::Black();

				if (!model_silhouette(i, j).IsBlack())
					if (IsBorderingPixelBlack(model_silhouette, i, j))
					{
						model_contour(i, j) = Pixel::White();
						model_border.push_back(Point<int>(i, j));
					}
					else model_contour(i, j) = Pixel::Black();
			}
		}

		// model border points that have input correspondences
		std::vector<Point<int>> model_correspondence;
		std::vector<Point<int>> input_correspondence;
		std::vector<Point<float>> distance;

		Image log_correspondences(model_contour), 
			log_marching(model_contour), 
			log_input_normals(input_contour);

		auto stop_condition = [&input_contour](int x, int y) { return (input_contour(x, y).IsWhite()); };
		//auto stop_condition = [](int x, int y) { return false; };
		for (auto& point : model_border)
		{
			float4 normal4 = model_normals[point.y*IMAGE_WIDTH + point.x];
			Point<float> normal = Point<float>(normal4.x, -normal4.y).normalized();
			AddCorrespondencePruned(stop_condition, point, normal, ray_dist_,
				input_silhouette, model_correspondence, input_correspondence, distance,
				logging_on, log_marching, log_input_normals);
		}

		// Draw correspondences
		auto stop_on_endpoint = [](int x, int y) { return false; };
		for (int i = 0; i < model_correspondence.size(); i++)
		{
			Bresenham(stop_on_endpoint, model_correspondence[i], input_correspondence[i], log_correspondences, true);
		}

		// Draw input onto correspondences for clarity
		for (int y = 0; y < IMAGE_HEIGHT; y++)
			for (int x = 0; x < IMAGE_WIDTH; x++)
			{
				if (input_contour(x, y).IsWhite())
				{
					log_correspondences(x, y) = Pixel::Yellow();
				}
			}

		if (logging_on)
		{
			input_contour.SavePNG(output_path + "input_contour.png");
			model_contour.SavePNG(output_path + "model_contour.png");
			log_correspondences.SavePNG(output_path + "correspondences.png");

			for (int y = 0; y < IMAGE_HEIGHT; y++)
				for (int x = 0; x < IMAGE_WIDTH; x++)
				{
					if (model_contour(x, y).IsWhite())
					{
						log_marching(x, y) = Pixel::Yellow();
					}

					if (input_contour(x, y).IsWhite())
					{
						log_marching(x, y) = Pixel::Red();
					}
				}

			log_marching.SavePNG(output_path + "marching.png");
			log_input_normals.SavePNG(output_path + "input_normals.png");
		}

		Correspondences result(log_correspondences, model_correspondence, input_correspondence, distance);
		return result;
	}

	Silhouette SilhouetteEnergy::Infer(const Eigen::Vector3f& translation,
		const ShapeCoefficients& betas,
		const PoseEulerCoefficients& thetas) const
	{
		Body body = generator_(betas, thetas, true);
		Silhouette result = silhouette_renderer_(body, projector_.CalculateView(translation),
			projector_.DirectXProjection(IMAGE_WIDTH, IMAGE_HEIGHT));
		return result;
	}

	void SilhouetteEnergy::ComputeSilhouetteError(const Correspondences& correspondences,
		const int residuals, const float weight, Eigen::VectorXf& error) const
	{
		assert(error.size() == residuals);

#pragma omp parallel for
		for (int m = 0; m < residuals; m += 2)
		{
			error(m) = correspondences.distance[m / 2].x;
			error(m + 1) = correspondences.distance[m / 2].y;
		}
		
		error *= weight;
		// need to normalize since there might be unequal number of correspondences
		// that one adds up compared to the previous iteration
		float norm = static_cast<float>(correspondences.model_border.size() * 2);
		error /= norm;
	}


	void SilhouetteEnergy::ComputeSilhouetteFromShapeJacobian(
		const Body& body, const std::vector<float3>& dshape, const Eigen::Vector3f& translation,
		const Silhouette& silhouette, const Correspondences& correspondences, 
		const int residuals, const float weight, Eigen::MatrixXf& jacobian) const
	{
		assert(jacobian.rows() == residuals);
		assert(jacobian.cols() == BETA_COUNT);

		Eigen::Matrix3f view = projector_.CalculateView(translation).block<3, 3>(0, 0);

#pragma omp parallel for
		for (int m = 0; m < residuals; m += 2)
		{
			const int x = correspondences.model_border[m / 2].x;
			const int y = correspondences.model_border[m / 2].y;
			const int4& indices = silhouette.GetVertexIndices()[y*IMAGE_WIDTH + x];
			const float4& barycentrics = silhouette.GetBarycentrics()[y*IMAGE_WIDTH + x];

			Eigen::Vector3f v0 = view * body.vertices[indices[0]].ToEigen() + translation;
			Eigen::Vector3f v1 = view * body.vertices[indices[1]].ToEigen() + translation;
			Eigen::Vector3f v2 = view * body.vertices[indices[2]].ToEigen() + translation;
			Eigen::Vector3f interpolated = barycentrics[0] * v0 +
				barycentrics[1] * v1 + barycentrics[2] * v2;

			for (int j = 0; j < BETA_COUNT; j++)
			{
				Eigen::Vector3f dv0 = view * dshape[indices[0] * BETA_COUNT + j].ToEigen();
				Eigen::Vector3f dv1 = view * dshape[indices[1] * BETA_COUNT + j].ToEigen();
				Eigen::Vector3f dv2 = view * dshape[indices[2] * BETA_COUNT + j].ToEigen();
				Eigen::Vector3f dinterpolated = barycentrics[0] * dv0 +
					barycentrics[1] * dv1 + barycentrics[2] * dv2;

				Eigen::Vector2f dsilhouette = Image::Jacobian(
					projector_.Jacobian(interpolated, dinterpolated));

				jacobian(m, j) = dsilhouette.x();
				jacobian(m + 1, j) = dsilhouette.y();
			}
		}

		jacobian *= weight;
		// need to normalize since there might be unequal number of correspondences
		// that one adds up compared to the previous iteration
		float norm = static_cast<float>(correspondences.model_border.size() * 2);
		jacobian /= norm;
	}

	void SilhouetteEnergy::ComputeSilhouetteFromPoseJacobian(
		const Body& body, const std::vector<float3>& dpose, const Eigen::Vector3f& translation,
		const Silhouette& silhouette, const Correspondences& correspondences,
		const int residuals, const float weight, Eigen::MatrixXf& jacobian) const
	{
		assert(jacobian.rows() == residuals);
		assert(jacobian.cols() == THETA_COMPONENT_COUNT);

		Eigen::Matrix3f view = projector_.CalculateView(translation).block<3, 3>(0, 0);

#pragma omp parallel for
		for (int m = 0; m < residuals; m += 2)
		{
			const int x = correspondences.model_border[m / 2].x;
			const int y = correspondences.model_border[m / 2].y;
			const int4& indices = silhouette.GetVertexIndices()[y*IMAGE_WIDTH + x];
			const float4& barycentrics = silhouette.GetBarycentrics()[y*IMAGE_WIDTH + x];

			Eigen::Vector3f v0 = view * body.vertices[indices[0]].ToEigen() + translation;
			Eigen::Vector3f v1 = view * body.vertices[indices[1]].ToEigen() + translation;
			Eigen::Vector3f v2 = view * body.vertices[indices[2]].ToEigen() + translation;
			Eigen::Vector3f interpolated = barycentrics[0] * v0 +
				barycentrics[1] * v1 + barycentrics[2] * v2;

			for (int k = 0; k < THETA_COMPONENT_COUNT; k++)
			{
				Eigen::Vector3f dv0 = view * dpose[indices[0] * THETA_COMPONENT_COUNT + k].ToEigen();
				Eigen::Vector3f dv1 = view * dpose[indices[1] * THETA_COMPONENT_COUNT + k].ToEigen();
				Eigen::Vector3f dv2 = view * dpose[indices[2] * THETA_COMPONENT_COUNT + k].ToEigen();
				Eigen::Vector3f dinterpolated = barycentrics[0] * dv0 +
					barycentrics[1] * dv1 + barycentrics[2] * dv2;

				Eigen::Vector2f dsilhouette = Image::Jacobian(
					projector_.Jacobian(interpolated, dinterpolated));

				jacobian(m, k) = dsilhouette.x();
				jacobian(m + 1, k) = dsilhouette.y();
			}
		}

		jacobian *= weight;
		// need to normalize since there might be unequal number of correspondences
		// that one adds up compared to the previous iteration
		float norm = static_cast<float>(correspondences.model_border.size() * 2);
		jacobian /= norm;
	}
}