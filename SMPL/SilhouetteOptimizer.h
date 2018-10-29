#pragma once
#include "Definitions.h"
#include "Utils.h"
#include "Image.h"
#include "Generator.h"
#include "Projector.h"
#include "SilhouetteMaker.h"

namespace smpl
{
	struct Correspondences
	{
		Correspondences(const Image& image, const std::vector<Point<int> > model_border,
			const std::vector<Point<int> > input_border, std::vector<Point<float> >&  distance) :
			image(image), model_border(model_border), input_border(input_border), distance(distance)
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
		SilhouetteOptimizer(const Generator& generator, const Projector& projector) 
		:	generator_(generator), projector_(projector), silhouette_maker_(generator_(true))
		{
		}

		Correspondences FindCorrespondences(const Image& input, const Image& model, const std::vector<float4>& normals);

		Silhouette Infer(const std::string& image_filename, Eigen::Vector3f& translation,
			ShapeCoefficients& betas, PoseEulerCoefficients& thetas);

		void ComputeSilhouetteError(const Correspondences& correspondences, 
			const int residuals, Eigen::VectorXf& error) const;

		void ComputeShapeJacobian(const PoseEulerCoefficients& thetas,
			const Body& body, std::vector<float3>& dshape);

		void ComputeSilhouetteFromShapeJacobian(
			const Body& body, const std::vector<float3>& dshape, const Eigen::Vector3f& translation,
			const Silhouette& silhouette, const Correspondences& correspondences,
			const int residuals, Eigen::MatrixXf& jacobian);

		void Reconstruct(const std::string& image_filename, const Image& input,
			Eigen::Vector3f& translation, ShapeCoefficients& betas, PoseEulerCoefficients& thetas);

	private:

		Eigen::Matrix4f CalculateView(Eigen::Vector3f translation) const;
		Eigen::Matrix3f CalculateView() const;

	private:

		const Generator& generator_;
		const Projector& projector_;
		SilhouetteMaker silhouette_maker_;
	};
}