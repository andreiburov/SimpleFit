#pragma once
#include "Definitions.h"
#include "Utils.h"
#include "Image.h"
#include "Generator.h"
#include "Projector.h"
#include "SilhouetteCorrespondences.h"
#include "SilhouetteRenderer.h"
#include <set>

namespace smpl
{
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

		Correspondences FindCorrespondencesPruned(
			const Image& input,
			const Image& model, 
			const std::vector<float4>& normals,
			const std::string& output_path) const;

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

		std::tuple<Point<int>, Point<float>> 
		MarchingPruned(
			const std::function<bool(int, int)>& stop_condition,
			const Point<int>& model_point,
			const Point<float>& model_normal, int max_dist,
			const Image& input_silhouette,
			bool logging_on,
			Image& log_marching,
			Image& log_input_derivatives) const;

		void AddCorrespondencePruned(
			std::function<bool(int, int)> stop_condition,
			const Point<int>& model_point,
			const Point<float>& model_normal, int max_dist,
			const Image& input_silhouette,
			std::vector<Point<int>>& model_correspondence,
			std::vector<Point<int>>& input_correspondence,
			std::vector<Point<float>>& distance,
			bool logging_on,
			Image& log_marching,
			Image& log_input_derivatives) const;

		const Generator& generator_;
		const Projector& projector_;
		const SilhouetteRenderer& silhouette_renderer_;

		// hyper parameters
		const int ray_dist_ = 35; // max distance when ray marching
		const int pd_ = 5; // the lookup distance when computing derivatives for pruning
	};
}