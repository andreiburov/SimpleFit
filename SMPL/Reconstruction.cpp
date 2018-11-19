#include "Reconstruction.h"

#include "JointsEnergy.h"
#include "SilhouetteEnergy.h"
#include "PriorEnergy.h"
#include "LevenbergMarquardt.h"

namespace smpl
{
	Reconstruction::Reconstruction(const Configuration& configuration) :
		generator_(configuration.model_path_),
		projector_(configuration.model_path_),
		regressor_(JointsRegressor::Configuration(configuration.model_path_, JointsRegressor::COCO)),
		silhouette_renderer_(generator_(true)),

		// configuration
		logging_on_(configuration.logging_on_),
		joints_weight_(configuration.joints_weight_),
		iterations_(configuration.iterations_),
		model_path_(configuration.model_path_),
		output_path_(configuration.output_path_),
		translation_(configuration.translation_)
	{
	}

	void Reconstruction::BodyFromSilhouette(const Image& silhouette, ShapeCoefficients& betas, PoseEulerCoefficients& thetas) const
	{
	}

	void Reconstruction::BodyFromJoints(
		const std::vector<float>& joints, Eigen::Vector3f& translation, 
		ShapeCoefficients& betas, PoseEulerCoefficients& thetas) const
	{
		JointsEnergy joint_energy(generator_, projector_, regressor_);
	}

	void Reconstruction::ShapeFromJoints(const std::vector<float>& input_joints, ShapeCoefficients& betas) const
	{
		PoseEulerCoefficients thetas;
		Eigen::Vector3f translation(translation_);
		JointsEnergy joints_energy(generator_, projector_, regressor_);

		const int unknowns = BETA_COUNT;
		LevenbergMarquardt lm_solver(unknowns, logging_on_);

		std::vector<float3> dshape(VERTEX_COUNT * BETA_COUNT); // dsmpl/dbeta
		for (int iteration = 0; iteration < iterations_; iteration++)
		{
			if (logging_on_)
			std::cout << "Iteration " << iteration << std::endl;

			Body body = generator_(betas, thetas, false);	

			// Prepare Jacobians and Errors

			// should be x2 since each point has two components
			const int residuals_joints = static_cast<int>(input_joints.size());
			Eigen::MatrixXf jacobian_joints = Eigen::MatrixXf::Zero(residuals_joints, unknowns);
			Eigen::VectorXf error_joints = Eigen::VectorXf::Zero(residuals_joints);

			joints_energy.ComputeError(input_joints, regressor_(body.vertices), translation, 
				residuals_joints, joints_weight_, error_joints);
			generator_.ComputeBodyFromShapeJacobian(dshape);
			joints_energy.ComputeJacobianFromShape(body, dshape, translation, 
				residuals_joints, joints_weight_, jacobian_joints);

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

			if (logging_on_)
			{
				if (minimized)
				{
					std::cout << "Betas" << std::endl;
					for (int i = 0; i < unknowns; i++)
						std::cout << betas[i] << " ";
					std::cout << std::endl;
				}

				std::vector<float> joints = Image::Coordinates(
					projector_.FromRegressed(regressor_(body.vertices), translation));

				Image image;
				Image::Draw3D(image, Pixel::White(), projector_, translation, body.vertices);
				Image::Draw2D(image, Pixel::Blue(), joints);
				Image::Draw2D(image, Pixel::Yellow(), input_joints);
				image.SavePNG(output_path_ + std::to_string(iteration) + ".png");
			}
		}
	}
}