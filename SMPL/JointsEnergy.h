#pragma once

#include "Generator.h"
#include "Projector.h"
#include "Image.h"

namespace smpl
{
	class JointsEnergy
	{
	public:
		JointsEnergy(const Generator& generator, 
			const Projector& projector, 
			const JointsRegressor& regressor);

		/*
			We can start by reading the values from the file.
			But then it should be from the openpose.
		*/
		/*std::vector<float> Infer(const std::string& image_filename, Eigen::Vector3f& translation,
			ShapeCoefficients& betas, PoseEulerCoefficients& thetas);*/

		void InitializeCameraPosition(
			const JointsRegressor::JointType& joint_type,
			const std::vector<float>& input_joints,
			const float focal_length_y, Eigen::Vector3f& translation) const;

		void ComputeError(const std::vector<float>& input_joints, 
			const RegressedJoints& model_joints, const Eigen::Vector3f& translation,
			const int residuals, const float weight, Eigen::VectorXf& error) const;

		void ComputeJacobianFromShape(
			const Body& body, const RegressedJoints& model_joints, 
			const std::vector<float3>& dshape, const Eigen::Vector3f& translation,
			const int residuals, const float weight, Eigen::MatrixXf& jacobian) const;

		void ComputeJacobianFromPose(
			const Body& body, const RegressedJoints& model_joints, 
			const std::vector<float3>& dpose, const Eigen::Vector3f& translation,
			const int residuals, const float weight, Eigen::MatrixXf& jacobian) const;

		void ComputeJacobianFromTranslation(
			const Body& body, const RegressedJoints& model_joints, const Eigen::Vector3f& translation,
			const int residuals, const float weight, Eigen::MatrixXf& jacobian) const;
		
	private:

		// rhs
		Eigen::Vector3f ToView(const Eigen::Vector3f& vertex, const Eigen::Vector3f& translation) const;
		Eigen::Vector3f ToView(const Eigen::Vector3f& vertex) const;

	private:

		const Generator& generator_;
		const Projector& projector_;
		const JointsRegressor& regressor_;
	};
}