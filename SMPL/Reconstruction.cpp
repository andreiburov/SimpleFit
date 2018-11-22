#include "Reconstruction.h"

#include "JointsEnergy.h"
#include "SilhouetteEnergy.h"
#include "PriorEnergy.h"
#include "LevenbergMarquardt.h"

namespace smpl
{
	Reconstruction::Reconstruction(const Configuration& configuration) :
		generator_(configuration.model_path),
		projector_(configuration.model_path),
		regressor_(JointsRegressor::Configuration(
			configuration.model_path, JointsRegressor::COCO)),
		silhouette_renderer_(generator_(true)),

		// configuration
		joints_weight_(configuration.joints_weight),
		silhouette_weight_(configuration.silhouette_weight),
		iterations_(configuration.iterations),
		ray_dist_(configuration.ray_dist)
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

	void Reconstruction::ShapeFromJoints(
		const std::string& output_path,
		const std::vector<float>& input_joints, 
		const Eigen::Vector3f& translation,
		ShapeCoefficients& betas) const
	{
		bool logging_on = true;
		if (output_path.empty()) logging_on = false;

		PoseEulerCoefficients thetas;
		JointsEnergy joints_energy(generator_, projector_, regressor_);

		const int unknowns = BETA_COUNT;
		LevenbergMarquardt lm_solver(unknowns, logging_on);

		std::vector<float3> dshape(VERTEX_COUNT * BETA_COUNT); // dsmpl/dbeta
		for (int iteration = 0; iteration < iterations_; iteration++)
		{
			if (logging_on)
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

			if (logging_on)
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
				Image::Draw2D(image, Pixel::Yellow(), input_joints);
				Image::Draw2D(image, Pixel::Blue(), joints);
				image.SavePNG(output_path + std::to_string(iteration) + ".png");
			}
		}
	}

	void Reconstruction::PoseFromJoints(
		const std::string& output_path,
		const std::vector<float>& input_joints,
		const Eigen::Vector3f& translation,
		PoseEulerCoefficients& thetas) const
	{
		bool logging_on = true;
		if (output_path.empty()) logging_on = false;

		ShapeCoefficients betas;
		JointsEnergy joints_energy(generator_, projector_, regressor_);

		const int unknowns = THETA_COMPONENT_COUNT;
		LevenbergMarquardt lm_solver(unknowns, logging_on);

		std::vector<float3> dpose(VERTEX_COUNT * THETA_COMPONENT_COUNT); // dsmpl/dpose
		for (int iteration = 0; iteration < iterations_; iteration++)
		{
			if (logging_on)
				std::cout << "Iteration " << iteration << std::endl;

			Body body = generator_(betas, thetas, false);

			// Prepare Jacobians and Errors

			// should be x2 since each point has two components
			const int residuals_joints = static_cast<int>(input_joints.size());
			Eigen::MatrixXf jacobian_joints = Eigen::MatrixXf::Zero(residuals_joints, unknowns);
			Eigen::VectorXf error_joints = Eigen::VectorXf::Zero(residuals_joints);

			joints_energy.ComputeError(input_joints, regressor_(body.vertices), translation,
				residuals_joints, joints_weight_, error_joints);
			generator_.ComputeBodyFromPoseJacobian(body, dpose);
			joints_energy.ComputeJacobianFromPose(body, dpose, translation,
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
					thetas(i) -= delta(i);
			else
				for (int i = 0; i < unknowns; i++)
					thetas(i) += delta(i);

			if (logging_on)
			{
				if (minimized)
				{
					std::cout << "Thetas" << std::endl;
					for (int i = 0; i < unknowns; i++)
						std::cout << thetas(i) << " ";
					std::cout << std::endl;
				}

				std::vector<float> joints = Image::Coordinates(
					projector_.FromRegressed(regressor_(body.vertices), translation));

				Image image;
				Image::Draw3D(image, Pixel::White(), projector_, translation, body.vertices);
				Image::Draw2D(image, Pixel::Yellow(), input_joints);
				Image::Draw2D(image, Pixel::Blue(), joints);
				image.SavePNG(output_path + std::to_string(iteration) + ".png");
			}
		}
	}

	void Reconstruction::ShapeFromSilhouette(
		const std::string& output_path,
		const Image& input_silhouette,
		const Eigen::Vector3f& translation,
		ShapeCoefficients& betas) const
	{
		bool logging_on = true;
		if (output_path.empty()) logging_on = false;

		PoseEulerCoefficients thetas;
		SilhouetteEnergy silhouette_energy(generator_, projector_, silhouette_renderer_);

		const int unknowns = BETA_COUNT;
		LevenbergMarquardt lm_solver(unknowns, logging_on);

		std::vector<float3> dshape(VERTEX_COUNT * BETA_COUNT); // dsmpl/dbeta

		for (int iteration = 0; iteration < iterations_; iteration++)
		{
			if (logging_on)
			{
				std::cout << "Iteration " << iteration << std::endl;
				std::cout << "Beta" << std::endl;
				for (uint j = 0; j < BETA_COUNT; j++)
					std::cout << betas[j] << " ";
				std::cout << std::endl;
			}

			Body body = generator_(betas, thetas, true);
			Silhouette silhouette = silhouette_renderer_(body,
				projector_.CalculateView(translation),
				projector_.DirectXProjection(
					IMAGE_WIDTH, IMAGE_HEIGHT));
			if (logging_on)
				silhouette.GetImage().SavePNG(
				output_path + std::string("silhouette") +
				std::to_string(iteration) + std::string(".png"));

			Correspondences correspondences = silhouette_energy.FindCorrespondences(
				input_silhouette, silhouette.GetImage(), silhouette.GetNormals());
			if (logging_on)
				correspondences.image.SavePNG(
				output_path + std::string("/correspondences") +
				std::to_string(iteration) + std::string(".png"));

			// should be x2 since each point has two components
			const int residuals = static_cast<int>(correspondences.model_border.size() * 2);

			Eigen::MatrixXf jacobian = Eigen::MatrixXf::Zero(residuals, unknowns);
			Eigen::VectorXf error = Eigen::VectorXf::Zero(residuals);
			Eigen::VectorXf delta = Eigen::VectorXf::Zero(unknowns);

			silhouette_energy.ComputeSilhouetteError(
				correspondences, residuals, silhouette_weight_, error);

			generator_.ComputeBodyFromShapeJacobian(dshape);
			silhouette_energy.ComputeSilhouetteFromShapeJacobian(
				body, dshape, translation, silhouette,
				correspondences, residuals, silhouette_weight_, jacobian);

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

	void Reconstruction::PoseFromSilhouette(
		const std::string& output_path,
		const Image& input_silhouette,
		const Eigen::Vector3f& translation,
		PoseEulerCoefficients& thetas) const
	{
		bool logging_on = true;
		if (output_path.empty()) logging_on = false;

		ShapeCoefficients betas;
		SilhouetteEnergy silhouette_energy(generator_, projector_, silhouette_renderer_);

		const int unknowns = THETA_COMPONENT_COUNT;

		std::vector<float3> dpose(VERTEX_COUNT * THETA_COMPONENT_COUNT);
		LevenbergMarquardt lm_solver(unknowns, logging_on);

		for (int iteration = 0; iteration < iterations_; iteration++)
		{
			if (logging_on)
			std::cout << "Iteration " << iteration << std::endl;

			Body body = generator_(betas, thetas, true);
			Silhouette silhouette = silhouette_renderer_(body, 
				projector_.CalculateView(translation),
				projector_.DirectXProjection(
					IMAGE_WIDTH,IMAGE_HEIGHT));
			if (logging_on)
				silhouette.GetImage().SavePNG(
				output_path + std::string("/silhouette") +
				std::to_string(iteration) + std::string(".png"));

			Correspondences correspondences = silhouette_energy.FindCorrespondences(
				input_silhouette, silhouette.GetImage(), silhouette.GetNormals());
			silhouette_energy.PruneCorrepondences(input_silhouette, 
				silhouette.GetImage(), silhouette.GetNormals(), correspondences);
			if (logging_on)
				correspondences.image.SavePNG(
				output_path + std::string("/correspondences") +
				std::to_string(iteration) + std::string(".png"));

			// should be x2 since each point has two components
			const int residuals_silhouette = static_cast<int>(correspondences.model_border.size() * 2);
			Eigen::MatrixXf jacobian_silhouette = Eigen::MatrixXf::Zero(residuals_silhouette, unknowns);
			Eigen::VectorXf error_silhouette = Eigen::VectorXf::Zero(residuals_silhouette);

			silhouette_energy.ComputeSilhouetteError(
				correspondences, residuals_silhouette, silhouette_weight_, error_silhouette);
			generator_.ComputeBodyFromPoseJacobian(body, dpose);
			silhouette_energy.ComputeSilhouetteFromPoseJacobian(
				body, dpose, translation,
				silhouette, correspondences,
				residuals_silhouette, silhouette_weight_, jacobian_silhouette);

			const int residuals_pose_prior = THETA_COMPONENT_COUNT;
			Eigen::MatrixXf jacobian_pose_prior = Eigen::MatrixXf::Zero(residuals_pose_prior, unknowns);
			Eigen::VectorXf error_pose_prior = Eigen::VectorXf::Zero(residuals_pose_prior);

			/*ComputePosePriorError(thetas, error_pose_prior);
			ComputePosePriorJacobian(jacobian_pose_prior);

			const int residuals = residuals_silhouette + residuals_pose_prior;
			Eigen::MatrixXf jacobian(residuals, unknowns);
			Eigen::VectorXf error(residuals);*/
			Eigen::VectorXf delta = Eigen::VectorXf::Zero(unknowns);

			const int residuals = residuals_silhouette;
			Eigen::MatrixXf jacobian(jacobian_silhouette);
			Eigen::VectorXf error(error_silhouette);

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
}