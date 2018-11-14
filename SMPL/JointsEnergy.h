#pragma once

#include "Generator.h"
#include "Projector.h"
#include "Image.h"

namespace smpl
{
	struct OpenPoseJoint
	{
		int x;
		int y;
		float confidence;
	};

	class JointsEnergy
	{
	public:
		JointsEnergy(const Generator& generator, const Projector& projector);

		/*
			We can start by reading the values from the file.
			But then it should be from the openpose.
		*/
		std::vector<OpenPoseJoint> Infer(const std::string& image_filename, Eigen::Vector3f& translation,
			ShapeCoefficients& betas, PoseEulerCoefficients& thetas);

		void ComputeError(const Joints& model_joints, const std::vector<Point<int> >& tracked_joints,
			const int residuals, Eigen::VectorXf& error) const;

		void ComputeJacobianFromShape(
			const Body& body, const std::vector<float3>& dshape, const Eigen::Vector3f& translation,
			const int residuals, Eigen::MatrixXf& jacobian) const;

		void ComputeJacobianFromPose(
			const Body& body, const std::vector<float3>& dpose, const Eigen::Vector3f& translation,
			const int residuals, Eigen::MatrixXf& jacobian) const;

	private:

		Eigen::Matrix4f CalculateView(Eigen::Vector3f translation) const;

	private:

		const Generator& generator_;
		const Projector& projector_;

		// global parameters
		int is_rhs_;

		// hyper parameters
	};
}