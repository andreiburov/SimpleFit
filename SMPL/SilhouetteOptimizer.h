#pragma once
#include "Definitions.h"
#include "Utils.h"
#include "Image.h"
#include "Generator.h"
#include "Projector.h"
#include "SilhouetteRenderer.h"
#include <set>

namespace smpl
{
	struct Correspondences
	{
		Correspondences(const Image& image, const std::vector<Point<int> >& model_border,
			const std::vector<Point<int> >& input_border, std::vector<Point<float> >&  distance) :
			image(image), model_border(model_border), 
			input_border(input_border), distance(distance)
		{
		}

		Image image;
		std::vector<Point<int> > model_border;
		std::vector<Point<int> > input_border;
		std::vector<Point<float> > distance;
	};

	class SilhouetteOptimizer
	{
	public:
		SilhouetteOptimizer(const Generator& generator, const Projector& projector);

		Correspondences FindCorrespondences(const Image& input, const Image& model, const std::vector<float4>& normals);

		void PruneCorrepondences(const Image& input, const Image& model, const std::vector<float4>& normals,
			Correspondences& correspondences);

		Silhouette Infer(const std::string& image_filename, Eigen::Vector3f& translation,
			ShapeCoefficients& betas, PoseEulerCoefficients& thetas);

		void ComputeSilhouetteError(const Correspondences& correspondences, 
			const int residuals, Eigen::VectorXf& error) const;

		void ComputePosePriorError(const PoseEulerCoefficients& thetas, Eigen::VectorXf error) const;

		void ComputeSilhouetteFromShapeJacobian(
			const Body& body, const std::vector<float3>& dshape, const Eigen::Vector3f& translation,
			const Silhouette& silhouette, const Correspondences& correspondences,
			const int residuals, Eigen::MatrixXf& jacobian) const;

		void ComputeSilhouetteFromPoseJacobian(
			const Body& body, const std::vector<float3>& dpose, const Eigen::Vector3f& translation,
			const Silhouette& silhouette, const Correspondences& correspondences,
			const int residuals, Eigen::MatrixXf& jacobian) const;

		void ComputePosePriorJacobian(Eigen::MatrixXf& jacobian) const;

		void ReconstructShape(const std::string& log_path, const Image& input,
			Eigen::Vector3f& translation, ShapeCoefficients& betas, PoseEulerCoefficients& thetas);

		void ReconstructPose(const std::string& log_path, const Image& input, const int ray_dist,
			Eigen::Vector3f& translation, ShapeCoefficients& betas, PoseEulerCoefficients& thetas);

	private:

		Eigen::Matrix4f CalculateView(Eigen::Vector3f translation) const;
		Eigen::Matrix3f CalculateView() const;

	private:

		const Generator& generator_;
		const Projector& projector_;
		SilhouetteRenderer silhouette_renderer_;

		// hyper parameters
		int ray_dist_ = 35;
		const int pd_ = 5; 
		const float silhouette_weight_ = 10.f;
		const float pose_prior_weight_ = 50.f;

		// priors per theta components
		const std::vector<float> pose_prior_per_theta_;
	};
}