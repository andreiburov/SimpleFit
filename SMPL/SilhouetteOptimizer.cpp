#include "SilhouetteOptimizer.h"

#include <stdexcept>
#include <cmath>
#include "LevenbergMarquardt.h"

namespace smpl
{
	const int MAX_DIST = 25;

	bool IsBorderingPixelBlack(const Image& image, int i, int j)
	{
		if (image(max(0, i - 1), max(0, j - 1)).IsBlack() ||
			image(i, max(0, j - 1)).IsBlack() ||
			image(min(IMAGE_WIDTH - 1, i + 1), max(0, j - 1)).IsBlack() ||

			image(max(0, i - 1), j).IsBlack() ||
			image(min(IMAGE_WIDTH - 1, i + 1), j).IsBlack() ||

			image(max(0, i - 1), min(IMAGE_HEIGHT - 1, j + 1)).IsBlack() ||
			image(i, min(IMAGE_HEIGHT - 1, j + 1)).IsBlack() ||
			image(min(IMAGE_WIDTH - 1, i + 1), min(IMAGE_HEIGHT - 1, j + 1)).IsBlack()
			)
			return true;
		else
			return false;
	}

	Point<int> Bresenham(const Image& input, Image& model, const Point<int>& p0,
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
			if (painted) model(x, y) = BLUE;

			if (!input(x, y).IsBlack()) return Point<int>(x, y);
			if (x < 0 || y < 0 || x >= IMAGE_WIDTH || y >= IMAGE_HEIGHT) return Point<int>();
			// update error
			e2 = 2 * err;
			// update coordinates depending on the error
			if (e2 > dy) { err += dy; x += sx; } /* e_xy+e_x > 0 */
			if (e2 < dx) { err += dx; y += sy; } /* e_xy+e_y < 0 */
		}

		return Point<int>();
	}

	void AddCorrespondence(const Image& input, Image& model, const Point<int>& point, 
		const Point<float>& normal,	int max_dist,
		std::vector<Point<int> >& model_correspondence, 
		std::vector<Point<int> >& input_correspondence,
		std::vector<Point<float> >& distance)
	{
		Point<int> x1(static_cast<int>(point[0] + max_dist * normal[0]), static_cast<int>(point[1] + max_dist * normal[1]));
		Point<int> x2(static_cast<int>(point[0] - max_dist * normal[0]), static_cast<int>(point[1] - max_dist * normal[1]));

		// input correspondences
		Point<int> c1 = Bresenham(input, model, point, x1, false);
		Point<int> c2 = Bresenham(input, model, point, x2, false);
		Point<float> d1, d2; // distance to x1,x2

		int l1_2 = -1;
		if (c1.IsDefined())
		{
			l1_2 = (point[0] - c1[0]) * (point[0] - c1[0]) + (point[1] - c1[1]) * (point[1] - c1[1]);
			d1 = Point<float>(point[0] - c1[0], point[1] - c1[1]);
		}
		int l2_2 = -1;
		if (c2.IsDefined())
		{
			l2_2 = (point[0] - c2[0]) * (point[0] - c2[0]) + (point[1] - c2[1]) * (point[1] - c2[1]);
			d2 = Point<float>(point[0] - c2[0], point[1] - c2[1]);
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

	Correspondences SilhouetteOptimizer::FindCorrespondences(const Image& input, const Image& model, const std::vector<float4>& normals)
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
						input_contour(i, j) = WHITE;
					}
					else input_contour(i, j) = BLACK;

				if (!model(i, j).IsBlack())
					if (IsBorderingPixelBlack(model, i, j))
					{
						correspondences(i, j) = WHITE;
						model_border.push_back(Point<int>(i, j));
					}
					else correspondences(i, j) = BLACK;
			}
		}

		/*input_contour.SavePNG("input_contour.png");
		correspondences.SavePNG("model_contour.png");*/

		for (auto& point : model_border)
		{
			float4 normal4 = normals[point.y*IMAGE_WIDTH + point.x];
			Point<float> normal = Point<float>(normal4.x, -normal4.y).normalized();
			AddCorrespondence(input_contour, correspondences, point, normal,
				MAX_DIST, model_correspondence, input_correspondence, distance);
		}

		// Draw correspondences
		for (int i = 0; i < model_correspondence.size(); i++)
		{
			Bresenham(input_contour, correspondences, model_correspondence[i], input_correspondence[i], true);
		}
		// Draw input onto correspondences for clarity
		for (int y = 0; y < IMAGE_HEIGHT; y++)
			for (int x = 0; x < IMAGE_WIDTH; x++)
			{
				if (input_contour(x, y).IsWhite())
				{
					correspondences(x, y) = YELLOW;
				}
			}

		Correspondences result(correspondences, model_correspondence, input_correspondence, distance);
		return result;
	}

	Silhouette SilhouetteOptimizer::Infer(const std::string& image_filename, Eigen::Vector3f& translation,
		ShapeCoefficients& betas, PoseEulerCoefficients& thetas)
	{
		Body body = generator_(betas, thetas, true);
		Silhouette result = silhouette_renderer_(body, CalculateView(translation),
			projector_.GetDirectXProjection(static_cast<float>(IMAGE_WIDTH), static_cast<float>(IMAGE_HEIGHT)));
		return result;
	}

	void SilhouetteOptimizer::ComputeSilhouetteError(const Correspondences& correspondences,
		const int residuals, Eigen::VectorXf& error) const
	{
#pragma omp parallel for
		for (int m = 0; m < residuals; m += 2)
		{
			error(m) = correspondences.distance[m / 2].x;
			error(m + 1) = correspondences.distance[m / 2].y;
		}
	}

	void SilhouetteOptimizer::ComputeSilhouetteFromShapeJacobian(
		const Body& body, const std::vector<float3>& dshape, const Eigen::Vector3f& translation,
		const Silhouette& silhouette, const Correspondences& correspondences, 
		const int residuals, Eigen::MatrixXf& jacobian) const
	{
		Eigen::Matrix3f view = CalculateView();

#pragma omp parallel for collapse(2)
		for (int m = 0; m < residuals; m += 2)
		{
			const int x = correspondences.model_border[m / 2].x;
			const int y = correspondences.model_border[m / 2].y;

			const int4& indices = silhouette.GetVertexIndices()[y*IMAGE_WIDTH + x];
			const float4& barycentrics = silhouette.GetBarycentrics()[y*IMAGE_WIDTH + x];

			for (int j = 0; j < BETA_COUNT; j++)
			{
				Eigen::Vector3f v0 = view * body.vertices[indices[0]].ToEigen() + translation;
				Eigen::Vector3f v1 = view * body.vertices[indices[1]].ToEigen() + translation;
				Eigen::Vector3f v2 = view * body.vertices[indices[2]].ToEigen() + translation;
				Eigen::Vector3f interpolated = barycentrics[0] * v0 +
					barycentrics[1] * v1 + barycentrics[2] * v2;

				Eigen::Vector3f dv0 = view * dshape[indices[0] * BETA_COUNT + j].ToEigen();
				Eigen::Vector3f dv1 = view * dshape[indices[1] * BETA_COUNT + j].ToEigen();
				Eigen::Vector3f dv2 = view * dshape[indices[2] * BETA_COUNT + j].ToEigen();
				Eigen::Vector3f dinterpolated = barycentrics[0] * dv0 +
					barycentrics[1] * dv1 + barycentrics[2] * dv2;

				Eigen::Vector2f dprojection = projector_.Jacobian(interpolated, dinterpolated);
				jacobian(m, j) = dprojection.x();
				// Flip the sign of y transforming from DirectX Image Space to FreeImage Image Space
				jacobian(m + 1, j) = -dprojection.y();
			}
		}
	}

	void SilhouetteOptimizer::ComputeSilhouetteFromPoseJacobian(
		const Body& body, const std::vector<float3>& dpose, const Eigen::Vector3f& translation,
		const Silhouette& silhouette, const Correspondences& correspondences,
		const int residuals, Eigen::MatrixXf& jacobian) const
	{
		Eigen::Matrix3f view = CalculateView();

#pragma omp parallel for collapse(2)
		for (int m = 0; m < residuals; m += 2)
		{
			const int x = correspondences.model_border[m / 2].x;
			const int y = correspondences.model_border[m / 2].y;

			const int4& indices = silhouette.GetVertexIndices()[y*IMAGE_WIDTH + x];
			const float4& barycentrics = silhouette.GetBarycentrics()[y*IMAGE_WIDTH + x];

			for (int k = 0; k < THETA_COMPONENT_COUNT; k++)
			{
				Eigen::Vector3f v0 = view * body.vertices[indices[0]].ToEigen() + translation;
				Eigen::Vector3f v1 = view * body.vertices[indices[1]].ToEigen() + translation;
				Eigen::Vector3f v2 = view * body.vertices[indices[2]].ToEigen() + translation;
				Eigen::Vector3f interpolated = barycentrics[0] * v0 +
					barycentrics[1] * v1 + barycentrics[2] * v2;

				Eigen::Vector3f dv0 = view * dpose[indices[0] * THETA_COMPONENT_COUNT + k].ToEigen();
				Eigen::Vector3f dv1 = view * dpose[indices[1] * THETA_COMPONENT_COUNT + k].ToEigen();
				Eigen::Vector3f dv2 = view * dpose[indices[2] * THETA_COMPONENT_COUNT + k].ToEigen();
				Eigen::Vector3f dinterpolated = barycentrics[0] * dv0 +
					barycentrics[1] * dv1 + barycentrics[2] * dv2;

				Eigen::Vector2f dprojection = projector_.Jacobian(interpolated, dinterpolated);
				jacobian(m, k) = dprojection.x();
				// Flip the sign of y transforming from DirectX Image Space to FreeImage Image Space
				jacobian(m + 1, k) = -dprojection.y();
			}
		}
	}

	void SilhouetteOptimizer::ReconstructShape(const std::string& log_path, const Image& input,
		Eigen::Vector3f& translation, ShapeCoefficients& betas, PoseEulerCoefficients& thetas)
	{
		std::vector<float3> dshape(VERTEX_COUNT * BETA_COUNT); // dsmpl/dbeta
		const int iterations_ = 20;
		const int unknowns = BETA_COUNT;

		LevenbergMarquardt lm_solver(unknowns);

		for (uint iteration = 0; iteration < iterations_; iteration++)
		{
			std::cout << "Iteration " << iteration << std::endl;
			std::cout << "Beta" << std::endl;
			for (uint j = 0; j < BETA_COUNT; j++)
				std::cout << betas[j] << " ";
			std::cout << std::endl;
				
			Body body = generator_(betas, thetas, true);
			Silhouette silhouette = silhouette_renderer_(body, CalculateView(translation),
				projector_.GetDirectXProjection(static_cast<float>(IMAGE_WIDTH), 
					static_cast<float>(IMAGE_HEIGHT)));
			silhouette.GetImage().SavePNG(
				log_path + std::string("/silhouette") + 
				std::to_string(iteration) + std::string(".png"));
			Correspondences correspondences = FindCorrespondences(input, silhouette.GetImage(), silhouette.GetNormals());
			correspondences.image.SavePNG(
				log_path + std::string("/correspondences") +
				std::to_string(iteration) + std::string(".png"));

			// should be x2 since each point has two components
			const int residuals = correspondences.model_border.size() * 2; 

			Eigen::MatrixXf jacobian = Eigen::MatrixXf::Zero(residuals, unknowns);
			Eigen::VectorXf error = Eigen::VectorXf::Zero(residuals);
			Eigen::VectorXf delta = Eigen::VectorXf::Zero(unknowns);

			ComputeSilhouetteError(correspondences, residuals, error);

			generator_.ComputeBodyFromShapeJacobian(dshape);
			ComputeSilhouetteFromShapeJacobian(body, dshape, translation, silhouette, 
				correspondences, residuals, jacobian);

			// levenberg marquardt
			bool minimized = lm_solver(jacobian, error, residuals, iteration, delta);
			if (minimized)
				for (int i = 0; i < BETA_COUNT; i++)
					betas[i] -= delta(i);
			else
				for (int i = 0; i < BETA_COUNT; i++)
					betas[i] += delta(i);
		}
	}

	void SilhouetteOptimizer::ReconstructPose(const std::string& log_path, const Image& input,
		Eigen::Vector3f& translation, ShapeCoefficients& betas, PoseEulerCoefficients& thetas)
	{
		std::vector<float3> dpose(VERTEX_COUNT * THETA_COMPONENT_COUNT); // dsmpl/dtheta
		const int iterations_ = 50;
		const int unknowns = THETA_COMPONENT_COUNT;

		LevenbergMarquardt lm_solver(unknowns);

		for (uint iteration = 0; iteration < iterations_; iteration++)
		{
			std::cout << "Iteration " << iteration << std::endl;

			Body body = generator_(betas, thetas, true);
			Silhouette silhouette = silhouette_renderer_(body, CalculateView(translation),
				projector_.GetDirectXProjection(static_cast<float>(IMAGE_WIDTH),
					static_cast<float>(IMAGE_HEIGHT)));
			silhouette.GetImage().SavePNG(
				log_path + std::string("/silhouette") +
				std::to_string(iteration) + std::string(".png"));
			Correspondences correspondences = FindCorrespondences(input, silhouette.GetImage(), silhouette.GetNormals());
			correspondences.image.SavePNG(
				log_path + std::string("/correspondences") +
				std::to_string(iteration) + std::string(".png"));

			// should be x2 since each point has two components
			const int residuals = correspondences.model_border.size() * 2;

			Eigen::MatrixXf jacobian = Eigen::MatrixXf::Zero(residuals, unknowns);
			Eigen::VectorXf error = Eigen::VectorXf::Zero(residuals);
			Eigen::VectorXf delta = Eigen::VectorXf::Zero(unknowns);

			ComputeSilhouetteError(correspondences, residuals, error);

			generator_.ComputeBodyFromPoseJacobian(body, dpose);
			ComputeSilhouetteFromPoseJacobian(body, dpose, translation, silhouette,
				correspondences, residuals, jacobian);

			// levenberg marquardt
			bool minimized = lm_solver(jacobian, error, residuals, iteration, delta);
			if (minimized)
				for (int i = 0; i < BETA_COUNT; i++)
					thetas(i) -= delta(i);
			else
				for (int i = 0; i < BETA_COUNT; i++)
					thetas(i) += delta(i);
		}
	}

	Eigen::Matrix4f SilhouetteOptimizer::CalculateView(Eigen::Vector3f translation) const
	{
		Eigen::Matrix4f view(Eigen::Matrix4f::Identity());
		// mesh is facing from us, rotate 180 around y
		view(0, 0) = -1;
		view(1, 1) = 1;
		view(2, 2) = -1;
		// mesh should be put at distance
		view(0, 3) = translation.x();
		view(1, 3) = translation.y();
		view(2, 3) = translation.z();
		return view;
	}

	Eigen::Matrix3f SilhouetteOptimizer::CalculateView() const
	{
		Eigen::Matrix3f view(Eigen::Matrix3f::Identity());
		// mesh is facing from us, rotate 180 around y
		view(0, 0) = -1;
		view(1, 1) = 1;
		view(2, 2) = -1;
		return view;
	}
}