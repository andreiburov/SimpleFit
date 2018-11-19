#include "SilhouetteEnergy.h"

#include <stdexcept>
#include <cmath>
#include <functional>
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
			if (painted) model(x, y) = BLUE;

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

	SilhouetteEnergy::SilhouetteEnergy(const Generator& generator, const Projector& projector) :
		generator_(generator), projector_(projector), silhouette_renderer_(generator_(true)), is_rhs_(projector.IsRhs()),
		pose_prior_per_theta_({
		10, 10, 10, // 0
		1, 10, 2, //1 
		1, 10, 2, //2
		2, 4, 4, //3
		1, 100, 100, //4
		1, 100, 100, //5
		2, 4, 4, //6
		100, 100, 100, //7
		100, 100, 100, //8
		10, 4, 4, //9
		100, 100, 100, //10
		100, 100, 100, //11
		1, 1, 2, //12
		10, 10, 10, //13
		10, 10, 10, //14
		4, 2, 4, //15
		2, 1, 1, //16
		2, 1, 1, //17
		100, 2, 1, //18
		100, 2, 1, //19
		100, 100, 100, //20
		100, 100, 100, //21
		100, 100, 100, //22
		100, 100, 100  //23
		})
	{
	}

	Correspondences SilhouetteEnergy::FindCorrespondences(const Image& input, const Image& model, const std::vector<float4>& normals)
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
					correspondences(x, y) = YELLOW;
				}
			}

		Correspondences result(correspondences, model_correspondence, input_correspondence, distance);
		return result;
	}

	void SilhouetteEnergy::PruneCorrepondences(const Image& input, const Image& model, const std::vector<float4>& normals,
		Correspondences& correspondences)
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
						draw_correspondences(i, j) = WHITE;
					else draw_correspondences(i, j) = BLACK;
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
				if(!input(i, j).IsBlack())
					if (IsBorderingPixelBlack(input, i, j))
					{
						draw_correspondences(i, j) = YELLOW;
					}
			}
		}

		correspondences.image = draw_correspondences;
	}

	Silhouette SilhouetteEnergy::Infer(const std::string& image_filename, Eigen::Vector3f& translation,
		ShapeCoefficients& betas, PoseEulerCoefficients& thetas)
	{
		Body body = generator_(betas, thetas, true);
		Silhouette result = silhouette_renderer_(body, CalculateView(translation),
			projector_.DirectXProjection(static_cast<float>(IMAGE_WIDTH), static_cast<float>(IMAGE_HEIGHT)));
		return result;
	}

	void SilhouetteEnergy::ComputeSilhouetteError(const Correspondences& correspondences,
		const int residuals, Eigen::VectorXf& error) const
	{
#pragma omp parallel for
		for (int m = 0; m < residuals; m += 2)
		{
			error(m) = correspondences.distance[m / 2].x;
			error(m + 1) = correspondences.distance[m / 2].y;
		}
		
		// need to normalize since there might be unequal number of correspondences
		// that one adds up compared to the previous iteration
		float norm = sqrt(static_cast<float>(correspondences.model_border.size() * 2));
		error /= norm;
		error *= silhouette_weight_;
	}

	void SilhouetteEnergy::ComputePosePriorError(
		const PoseEulerCoefficients& thetas, Eigen::VectorXf error) const
	{
#pragma omp parallel for
		for (int k = 0; k < THETA_COMPONENT_COUNT; k++)
		{
			error(k) = pose_prior_weight_ * pose_prior_per_theta_[k] * thetas(k);
		}
	}

	void SilhouetteEnergy::ComputeSilhouetteFromShapeJacobian(
		const Body& body, const std::vector<float3>& dshape, const Eigen::Vector3f& translation,
		const Silhouette& silhouette, const Correspondences& correspondences, 
		const int residuals, Eigen::MatrixXf& jacobian) const
	{
		Eigen::Matrix3f view = CalculateView(translation).block<3, 3>(0, 0);

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

				Eigen::Vector2f dprojection = projector_.Jacobian(interpolated, dinterpolated);
				jacobian(m, j) = dprojection.x();
				// Flip the sign of y transforming from DirectX Image Space to FreeImage Image Space
				jacobian(m + 1, j) = -dprojection.y();
			}
		}
	}

	void SilhouetteEnergy::ComputeSilhouetteFromPoseJacobian(
		const Body& body, const std::vector<float3>& dpose, const Eigen::Vector3f& translation,
		const Silhouette& silhouette, const Correspondences& correspondences,
		const int residuals, Eigen::MatrixXf& jacobian) const
	{
		Eigen::Matrix3f view = CalculateView(translation).block<3, 3>(0, 0);

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

				Eigen::Vector2f dprojection = projector_.Jacobian(interpolated, dinterpolated);
				jacobian(m, k) = dprojection.x();
				// Flip the sign of y transforming from DirectX Image Space to FreeImage Image Space
				jacobian(m + 1, k) = -dprojection.y();
			}
		}

		// need to normalize since there might be unequal number of correspondences
		// that one adds up compared to the previous iteration
		float norm = sqrt(static_cast<float>(correspondences.model_border.size() * 2));
		jacobian /= norm;
		jacobian *= silhouette_weight_;
	}

	void SilhouetteEnergy::ComputePosePriorJacobian(Eigen::MatrixXf& jacobian) const
	{
#pragma omp parallel for
		for (int k = 0; k < THETA_COMPONENT_COUNT; k++)
		{
			jacobian(k, k) = pose_prior_weight_ * pose_prior_per_theta_[k];
		}
	}

	void SilhouetteEnergy::ReconstructShape(const std::string& log_path, const Image& input,
		Eigen::Vector3f& translation, ShapeCoefficients& betas, PoseEulerCoefficients& thetas)
	{
		std::vector<float3> dshape(VERTEX_COUNT * BETA_COUNT); // dsmpl/dbeta
		const int iterations_ = 20;
		const int unknowns = BETA_COUNT;

		LevenbergMarquardt lm_solver(unknowns, true);

		for (uint iteration = 0; iteration < iterations_; iteration++)
		{
			std::cout << "Iteration " << iteration << std::endl;
			std::cout << "Beta" << std::endl;
			for (uint j = 0; j < BETA_COUNT; j++)
				std::cout << betas[j] << " ";
			std::cout << std::endl;
				
			Body body = generator_(betas, thetas, true);
			Silhouette silhouette = silhouette_renderer_(body, CalculateView(translation),
				projector_.DirectXProjection(static_cast<float>(IMAGE_WIDTH), 
					static_cast<float>(IMAGE_HEIGHT)));
			silhouette.GetImage().SavePNG(
				log_path + std::string("/silhouette") + 
				std::to_string(iteration) + std::string(".png"));
			Correspondences correspondences = FindCorrespondences(input, silhouette.GetImage(), silhouette.GetNormals());
			correspondences.image.SavePNG(
				log_path + std::string("/correspondences") +
				std::to_string(iteration) + std::string(".png"));

			// should be x2 since each point has two components
			const int residuals = static_cast<int>(correspondences.model_border.size() * 2); 

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
				for (int i = 0; i < unknowns; i++)
					betas[i] -= delta(i);
			else
				for (int i = 0; i < unknowns; i++)
					betas[i] += delta(i);
		}
	}

	void SilhouetteEnergy::ReconstructPose(
		const std::string& log_path, const Image& input, const int ray_dist,
		Eigen::Vector3f& translation, ShapeCoefficients& betas, PoseEulerCoefficients& thetas)
	{
		std::vector<float3> dpose(VERTEX_COUNT * THETA_COMPONENT_COUNT); // dsmpl/dtheta
		ray_dist_ = ray_dist;
		const int iterations_ = 30;
		const int unknowns = THETA_COMPONENT_COUNT;

		LevenbergMarquardt lm_solver(unknowns, true);

		for (uint iteration = 0; iteration < iterations_; iteration++)
		{
			std::cout << "Iteration " << iteration << std::endl;

			Body body = generator_(betas, thetas, true);
			Silhouette silhouette = silhouette_renderer_(body, CalculateView(translation),
				projector_.DirectXProjection(static_cast<float>(IMAGE_WIDTH),
					static_cast<float>(IMAGE_HEIGHT)));
			silhouette.GetImage().SavePNG(
				log_path + std::string("/silhouette") +
				std::to_string(iteration) + std::string(".png"));
			Correspondences correspondences = FindCorrespondences(input, silhouette.GetImage(), silhouette.GetNormals());
			PruneCorrepondences(input, silhouette.GetImage(), silhouette.GetNormals(), correspondences);
			correspondences.image.SavePNG(
				log_path + std::string("/correspondences") +
				std::to_string(iteration) + std::string(".png"));

			// should be x2 since each point has two components
			const int residuals_silhouette = static_cast<int>(correspondences.model_border.size() * 2);
			Eigen::MatrixXf jacobian_silhouette = Eigen::MatrixXf::Zero(residuals_silhouette, unknowns);
			Eigen::VectorXf error_silhouette = Eigen::VectorXf::Zero(residuals_silhouette);

			ComputeSilhouetteError(correspondences, residuals_silhouette, error_silhouette);
			generator_.ComputeBodyFromPoseJacobian(body, dpose);
			ComputeSilhouetteFromPoseJacobian(body, dpose, translation, 
				silhouette,	correspondences, 
				residuals_silhouette, jacobian_silhouette);

			const int residuals_pose_prior = THETA_COMPONENT_COUNT;
			Eigen::MatrixXf jacobian_pose_prior = Eigen::MatrixXf::Zero(residuals_pose_prior, unknowns);
			Eigen::VectorXf error_pose_prior = Eigen::VectorXf::Zero(residuals_pose_prior);

			ComputePosePriorError(thetas, error_pose_prior);
			ComputePosePriorJacobian(jacobian_pose_prior);

			const int residuals = residuals_silhouette + residuals_pose_prior;
			Eigen::MatrixXf jacobian(residuals, unknowns);
			Eigen::VectorXf error(residuals);
			Eigen::VectorXf delta = Eigen::VectorXf::Zero(unknowns);

			jacobian << jacobian_silhouette, jacobian_pose_prior;
			error << error_silhouette, error_pose_prior;

			// levenberg marquardt
			bool minimized = lm_solver(jacobian, error, residuals, iteration, delta);
			if (minimized)
				for (int i = 0; i < unknowns; i++)
					thetas(i) -= delta(i);
			else
				for (int i = 0; i < unknowns; i++)
					thetas(i) += delta(i);

			std::cout << "Thetas\n";
			for (int i = 0; i < unknowns; i++)
				std::cout << thetas(i) << " ";
			std::cout << "\n";
		}
	}

	Eigen::Matrix4f SilhouetteEnergy::CalculateView(Eigen::Vector3f translation) const
	{
		Eigen::Matrix4f view(Eigen::Matrix4f::Identity());

		if (is_rhs_)
		{
			// put mesh at negative distance
			view(0, 3) = translation.x();
			view(1, 3) = translation.y();
			view(2, 3) = -translation.z();
		}
		else 
		{
			// mesh is facing from us in LHS, rotate 180 around y
			view(0, 0) = -1;
			view(1, 1) = 1;
			view(2, 2) = -1;

			view(0, 3) = translation.x();
			view(1, 3) = translation.y();
			view(2, 3) = translation.z();
		}

		return view;
	}
}