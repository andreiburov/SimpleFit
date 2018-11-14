#include "Reconstruction.h"

#include "JointsEnergy.h"
#include "SilhouetteEnergy.h"
#include "PriorEnergy.h"
#include "LevenbergMarquardt.h"

namespace smpl
{
	Reconstruction::Reconstruction(Configuration&& configuration) :
		generator_(configuration.configuration_path),
		projector_(configuration.configuration_path),
		silhouette_renderer_(generator_(true)),

		// configuration
		output_path_(configuration.output_path_),
		iterations_(configuration.iterations_),
		translation_(configuration.translation_)
	{
	}

	void Reconstruction::BodyFromSilhouette(const Image& silhouette, ShapeCoefficients& betas, PoseEulerCoefficients& thetas) const
	{

	}

	void Reconstruction::BodyFromJoints(const std::vector<Point<int> >& joints, Eigen::Vector3f& translation, ShapeCoefficients& betas, PoseEulerCoefficients& thetas) const
	{
		JointsEnergy joint_energy(generator_, projector_);
	}

	void Reconstruction::ShapeFromJoints(const std::vector<Point<int> >& joints, ShapeCoefficients& betas) const
	{
		PoseEulerCoefficients thetas;
		Eigen::Vector3f translation(translation_[0], translation_[1], translation_[2]);
		JointsEnergy joint_energy(generator_, projector_);

		const int unknowns = BETA_COUNT;
		LevenbergMarquardt lm_solver(unknowns);

		std::vector<float3> dshape(VERTEX_COUNT * BETA_COUNT); // dsmpl/dbeta
		for (int iteration = 0; iteration < iterations_; iteration++)
		{
			std::cout << "Iteration " << iteration << std::endl;

			Body body = generator_(betas, thetas, false);	

			// Prepare Jacobians and Errors

			// should be x2 since each point has two components
			const int residuals_joints = static_cast<int>(joints.size() * 2);
			Eigen::MatrixXf jacobian_joints = Eigen::MatrixXf::Zero(residuals_joints, unknowns);
			Eigen::VectorXf error_joints = Eigen::VectorXf::Zero(residuals_joints);

			joint_energy.ComputeError(body.joints, joints, residuals_joints, error_joints);
			generator_.ComputeBodyFromShapeJacobian(dshape);
			joint_energy.ComputeJacobianFromShape(body, dshape, translation, residuals_joints, jacobian_joints);

			// Reconstruction
			const int residuals = residuals_joints;
			Eigen::MatrixXf jacobian(residuals, unknowns);
			Eigen::VectorXf error(residuals);
			Eigen::VectorXf delta = Eigen::VectorXf::Zero(unknowns);

			jacobian << jacobian_joints;
			error << error_joints;

			// levenberg marquardt
			bool minimized = lm_solver(jacobian, error, residuals, iteration, delta);
			if (minimized)
				for (int i = 0; i < unknowns; i++)
					betas[i] -= delta(i);
			else
				for (int i = 0; i < unknowns; i++)
					betas[i] += delta(i);

			std::cout << "Betas\n";
			for (int i = 0; i < unknowns; i++)
				std::cout << betas[i] << " ";
			std::cout << "\n";
		}
	}
}