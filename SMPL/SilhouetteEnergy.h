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

	class SilhouetteEnergy
	{
	public:
		SilhouetteEnergy(
			const Generator& generator,
			const Projector& projector,
			const SilhouetteRenderer& renderer,
			const int ray_dist, const int pruning_derivative_half_dx);

		Correspondences FindCorrespondences(const Image& input, 
			const Image& model, const std::vector<float4>& normals) const;

		void PruneCorrepondences(const Image& input, 
			const Image& model, const std::vector<float4>& normals,
			Correspondences& correspondences) const;

		Silhouette Infer(const Eigen::Vector3f& translation,
			const ShapeCoefficients& betas, 
			const PoseEulerCoefficients& thetas) const;

		void ComputeSilhouetteError(const Correspondences& correspondences, 
			const int residuals, const float weight, Eigen::VectorXf& error) const;

		void ComputeSilhouetteFromShapeJacobian(
			const Body& body, const std::vector<float3>& dshape, const Eigen::Vector3f& translation,
			const Silhouette& silhouette, const Correspondences& correspondences,
			const int residuals, const float weight, Eigen::MatrixXf& jacobian) const;

		void ComputeSilhouetteFromPoseJacobian(
			const Body& body, const std::vector<float3>& dpose, const Eigen::Vector3f& translation,
			const Silhouette& silhouette, const Correspondences& correspondences,
			const int residuals, const float weight, Eigen::MatrixXf& jacobian) const;

	private:

		const Generator& generator_;
		const Projector& projector_;
		const SilhouetteRenderer& silhouette_renderer_;

		// hyper parameters
		const int ray_dist_ = 35; // max distance when ray marching
		const int pd_ = 5; // the lookup distance when computing derivatives for pruning

		// priors per theta components
		const std::vector<float> pose_prior_per_theta_;
	};
}