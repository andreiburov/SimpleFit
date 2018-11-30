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
		pose_prior_weight_(configuration.pose_prior_weight),
		bend_prior_weight_(configuration.bend_prior_weight),
		shape_prior_weight_(configuration.shape_prior_weight),
		iterations_(configuration.iterations),
		ray_dist_(configuration.ray_dist),
		pruning_derivative_half_dx_(configuration.pruning_derivative_half_dx)
	{
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
		SilhouetteEnergy silhouette_energy(generator_, projector_, silhouette_renderer_,
			ray_dist_, pruning_derivative_half_dx_);

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
		SilhouetteEnergy silhouette_energy(generator_, projector_, silhouette_renderer_,
			ray_dist_, pruning_derivative_half_dx_);

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
					IMAGE_WIDTH, IMAGE_HEIGHT));
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

			if (logging_on)
			{
				if (minimized)
				{
					std::cout << "Thetas\n";
					for (int i = 0; i < unknowns; i++)
						std::cout << thetas(i) << " ";
					std::cout << "\n";
				}
			}
		}
	}

	void Reconstruction::BodyFromSilhouette(const std::string& output_path,
		const Image& input_silhouette,
		const Eigen::Vector3f& translation,
		ShapeCoefficients& betas, PoseEulerCoefficients& thetas) const
	{
		bool logging_on = true;
		if (output_path.empty()) logging_on = false;

		SilhouetteEnergy silhouette_energy(generator_, projector_,
			silhouette_renderer_, ray_dist_, pruning_derivative_half_dx_);

		const int unknowns = 3 + BETA_COUNT + THETA_COMPONENT_COUNT;
		LevenbergMarquardt lm_solver(unknowns, logging_on);

		std::vector<float3> dshape(VERTEX_COUNT * BETA_COUNT);
		std::vector<float3> dpose(VERTEX_COUNT * THETA_COMPONENT_COUNT);
		for (int iteration = 0; iteration < iterations_; iteration++)
		{
			if (logging_on)
				std::cout << "Iteration " << iteration << std::endl;

			Body body = generator_(betas, thetas, true);
			
			Silhouette silhouette = silhouette_renderer_(body,
				projector_.CalculateView(translation),
				projector_.DirectXProjection(
					IMAGE_WIDTH, IMAGE_HEIGHT));
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

			// Prepare Jacobians and Errors
			generator_.ComputeBodyFromShapeJacobian(dshape);
			generator_.ComputeBodyFromPoseJacobian(body, dpose);

			// SILHOUETTE
			const int residuals_silhouette = static_cast<int>(correspondences.model_border.size() * 2);
			Eigen::VectorXf error_silhouette = Eigen::VectorXf::Zero(residuals_silhouette);

			Eigen::MatrixXf jacobian_silhouette_translation = Eigen::MatrixXf::Zero(residuals_silhouette, 3);
			Eigen::MatrixXf jacobian_silhouette_shape = Eigen::MatrixXf::Zero(residuals_silhouette, BETA_COUNT);
			Eigen::MatrixXf jacobian_silhouette_pose = Eigen::MatrixXf::Zero(residuals_silhouette, THETA_COMPONENT_COUNT);

			silhouette_energy.ComputeSilhouetteError(
				correspondences, residuals_silhouette, silhouette_weight_, error_silhouette);
			generator_.ComputeBodyFromPoseJacobian(body, dpose);
			silhouette_energy.ComputeSilhouetteFromShapeJacobian(
				body, dshape, translation, silhouette,
				correspondences, residuals_silhouette, silhouette_weight_, jacobian_silhouette_shape);
			silhouette_energy.ComputeSilhouetteFromPoseJacobian(
				body, dpose, translation,
				silhouette, correspondences,
				residuals_silhouette, silhouette_weight_, jacobian_silhouette_pose);

			Eigen::MatrixXf jacobian_silhouette(residuals_silhouette, unknowns);
			jacobian_silhouette << jacobian_silhouette_translation, jacobian_silhouette_shape, jacobian_silhouette_pose;

			// Reconstruction
			const int residuals = residuals_silhouette;
			Eigen::MatrixXf jacobian(residuals, unknowns);
			Eigen::VectorXf error(residuals);
			Eigen::VectorXf delta = Eigen::VectorXf::Zero(unknowns);

			jacobian << jacobian_silhouette;
			error << error_silhouette;

			// levenberg marquardt
			bool minimized = lm_solver(jacobian, error, residuals, iteration, delta);
			if (minimized)
			{
				for (int i = 0; i < BETA_COUNT; i++) betas[i] -= shape_delta_weight_ * delta(i + 3);
				for (int i = 0; i < THETA_COMPONENT_COUNT; i++) thetas(i) -= delta(i + 3 + BETA_COUNT);
			}
			else
			{
				for (int i = 0; i < BETA_COUNT; i++) betas[i] += shape_delta_weight_ * delta(i + 3);
				for (int i = 0; i < THETA_COMPONENT_COUNT; i++) thetas(i) += delta(i + 3 + BETA_COUNT);
			}

			if (logging_on)
			{
				if (minimized)
				{
					std::cout << "Betas + Thetas" << std::endl;
					for (int i = 0; i < BETA_COUNT; i++) std::cout << betas[i] << std::endl;
					std::cout << "---------------------------------" << std::endl;
					for (int i = 0; i < THETA_COMPONENT_COUNT; i++) std::cout << thetas(i) << std::endl;
					std::cout << std::endl;
				}
			}
		}
	}

	void Reconstruction::BodyFromJointsAndSilhouette(
		const std::string& output_path,
		const std::vector<float>& input_joints, const Image& input_silhouette, 
		Eigen::Vector3f& translation, ShapeCoefficients& betas, PoseEulerCoefficients& thetas) const
	{
		bool logging_on = true;
		if (output_path.empty()) logging_on = false;

		JointsEnergy joints_energy(generator_, projector_, regressor_);
		SilhouetteEnergy silhouette_energy(generator_, projector_, 
			silhouette_renderer_, ray_dist_, pruning_derivative_half_dx_);

		const int unknowns = 3 + BETA_COUNT + THETA_COMPONENT_COUNT;
		LevenbergMarquardt lm_solver(unknowns, logging_on);

		joints_energy.InitializeCameraPosition(regressor_.GetJointType(),
			input_joints, projector_.GetIntrinsics()(1, 1), translation);

		std::vector<float3> dshape(VERTEX_COUNT * BETA_COUNT);
		std::vector<float3> dpose(VERTEX_COUNT * THETA_COMPONENT_COUNT);
		for (int iteration = 0; iteration < iterations_; iteration++)
		{
			if (logging_on)
				std::cout << "Iteration " << iteration << std::endl;

			Body body = generator_(betas, thetas, true);
			RegressedJoints model_joints = regressor_(body.vertices);
			if (logging_on)
			{
				std::vector<float> joints = Image::Coordinates(
					projector_.FromRegressed(model_joints, translation));

				Image image;
				Image::Draw3D(image, Pixel::White(), projector_, translation, body.vertices);
				Image::Draw2D(image, Pixel::Yellow(), input_joints);
				Image::Draw2D(image, Pixel::Blue(), joints);
				image.SavePNG(output_path + std::string("/joints") + 
					std::to_string(iteration) + ".png");
			}

			Silhouette silhouette = silhouette_renderer_(body,
				projector_.CalculateView(translation),
				projector_.DirectXProjection(
					IMAGE_WIDTH, IMAGE_HEIGHT));
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

			// Prepare Jacobians and Errors
			generator_.ComputeBodyFromShapeJacobian(dshape);
			generator_.ComputeBodyFromPoseJacobian(body, dpose);

			// JOINTS
			const int residuals_joints = static_cast<int>(input_joints.size());
			Eigen::VectorXf error_joints = Eigen::VectorXf::Zero(residuals_joints);

			Eigen::MatrixXf jacobian_joints_translation = Eigen::MatrixXf::Zero(residuals_joints, 3);
			Eigen::MatrixXf jacobian_joints_shape = Eigen::MatrixXf::Zero(residuals_joints, BETA_COUNT);
			Eigen::MatrixXf jacobian_joints_pose = Eigen::MatrixXf::Zero(residuals_joints, THETA_COMPONENT_COUNT);

			joints_energy.ComputeError(input_joints, model_joints, translation,
				residuals_joints, joints_weight_, error_joints);
			joints_energy.ComputeJacobianFromTranslation(body, model_joints, translation,
				residuals_joints, joints_weight_, jacobian_joints_translation);
			joints_energy.ComputeJacobianFromShape(body, model_joints, dshape, translation,
				residuals_joints, joints_weight_, jacobian_joints_shape);
			joints_energy.ComputeJacobianFromPose(body, model_joints, dpose, translation,
				residuals_joints, joints_weight_, jacobian_joints_pose);

			Eigen::MatrixXf jacobian_joints(residuals_joints, unknowns);
			jacobian_joints << jacobian_joints_translation, jacobian_joints_shape, jacobian_joints_pose;

			// SILHOUETTE
			const int residuals_silhouette = static_cast<int>(correspondences.model_border.size() * 2);
			Eigen::VectorXf error_silhouette = Eigen::VectorXf::Zero(residuals_silhouette);

			Eigen::MatrixXf jacobian_silhouette_translation = Eigen::MatrixXf::Zero(residuals_silhouette, 3);
			Eigen::MatrixXf jacobian_silhouette_shape = Eigen::MatrixXf::Zero(residuals_silhouette, BETA_COUNT);
			Eigen::MatrixXf jacobian_silhouette_pose = Eigen::MatrixXf::Zero(residuals_silhouette, THETA_COMPONENT_COUNT);
			
			silhouette_energy.ComputeSilhouetteError(
				correspondences, residuals_silhouette, silhouette_weight_, error_silhouette);
			generator_.ComputeBodyFromPoseJacobian(body, dpose);
			silhouette_energy.ComputeSilhouetteFromShapeJacobian(
				body, dshape, translation, silhouette,
				correspondences, residuals_silhouette, silhouette_weight_, jacobian_silhouette_shape);
			silhouette_energy.ComputeSilhouetteFromPoseJacobian(
				body, dpose, translation,
				silhouette, correspondences,
				residuals_silhouette, silhouette_weight_, jacobian_silhouette_pose);

			Eigen::MatrixXf jacobian_silhouette(residuals_silhouette, unknowns);
			jacobian_silhouette << jacobian_silhouette_translation, jacobian_silhouette_shape, jacobian_silhouette_pose;

			// Reconstruction
			const int residuals = residuals_joints + residuals_silhouette;
			Eigen::MatrixXf jacobian(residuals, unknowns);
			Eigen::VectorXf error(residuals);
			Eigen::VectorXf delta = Eigen::VectorXf::Zero(unknowns);

			jacobian << jacobian_joints, jacobian_silhouette;
			error << error_joints, error_silhouette;

			// levenberg marquardt
			bool minimized = lm_solver(jacobian, error, residuals, iteration, delta);
			if (minimized)
			{
				for (int i = 0; i < 3; i++)	translation(i) -= delta(i);
				for (int i = 0; i < BETA_COUNT; i++) betas[i] -= delta(i + 3);
				for (int i = 0; i < THETA_COMPONENT_COUNT; i++) thetas(i) -= delta(i + 3 + BETA_COUNT);
			}
			else
			{
				for (int i = 0; i < 3; i++)	translation(i) += delta(i);
				for (int i = 0; i < BETA_COUNT; i++) betas[i] += delta(i + 3);
				for (int i = 0; i < THETA_COMPONENT_COUNT; i++) thetas(i) += delta(i + 3 + BETA_COUNT);
			}

			if (logging_on)
			{
				if (minimized)
				{
					std::cout << "Translation + Betas + Thetas" << std::endl;
					for (int i = 0; i < 3; i++)	std::cout << translation(i) << std::endl;
					std::cout << "---------------------------------" << std::endl;
					for (int i = 0; i < BETA_COUNT; i++) std::cout << betas[i] << std::endl;
					std::cout << "---------------------------------" << std::endl;
					for (int i = 0; i < THETA_COMPONENT_COUNT; i++) std::cout << thetas(i) << std::endl;
					std::cout << std::endl;
				}
			}
		}
	}

	void Reconstruction::BodyFromJointsRegularized(
		const std::string& output_path,
		const std::vector<float>& input_joints, Eigen::Vector3f& translation,
		ShapeCoefficients& betas, PoseEulerCoefficients& thetas) const
	{
		bool logging_on = true;
		if (output_path.empty()) logging_on = false;

		JointsEnergy joints_energy(generator_, projector_, regressor_);
		PriorEnergy prior_energy;

		const int unknowns = 3 + BETA_COUNT + THETA_COMPONENT_COUNT;
		LevenbergMarquardt lm_solver(unknowns, logging_on);

		joints_energy.InitializeCameraPosition(regressor_.GetJointType(),
			input_joints, projector_.GetIntrinsics()(1, 1), translation);

		std::vector<float3> dshape(VERTEX_COUNT * BETA_COUNT);
		std::vector<float3> dpose(VERTEX_COUNT * THETA_COMPONENT_COUNT);
		for (int iteration = 0; iteration < iterations_; iteration++)
		{
			if (logging_on)
				std::cout << "Iteration " << iteration << std::endl;

			Body body = generator_(betas, thetas, false);
			RegressedJoints model_joints = regressor_(body.vertices);

			// Prepare Jacobians and Errors
			generator_.ComputeBodyFromShapeJacobian(dshape);
			generator_.ComputeBodyFromPoseJacobian(body, dpose);

			// JOINTS
			const int residuals_joints = static_cast<int>(input_joints.size());
			Eigen::VectorXf error_joints = Eigen::VectorXf::Zero(residuals_joints);

			Eigen::MatrixXf jacobian_joints_translation = Eigen::MatrixXf::Zero(residuals_joints, 3);
			Eigen::MatrixXf jacobian_joints_shape = Eigen::MatrixXf::Zero(residuals_joints, BETA_COUNT);
			Eigen::MatrixXf jacobian_joints_pose = Eigen::MatrixXf::Zero(residuals_joints, THETA_COMPONENT_COUNT);

			joints_energy.ComputeError(input_joints, model_joints, translation,
				residuals_joints, joints_weight_, error_joints);
			joints_energy.ComputeJacobianFromTranslation(body, model_joints, translation,
				residuals_joints, joints_weight_, jacobian_joints_translation);
			joints_energy.ComputeJacobianFromShape(body, model_joints, dshape, translation,
				residuals_joints, joints_weight_, jacobian_joints_shape);
			joints_energy.ComputeJacobianFromPose(body, model_joints, dpose, translation,
				residuals_joints, joints_weight_, jacobian_joints_pose);

			Eigen::MatrixXf jacobian_joints(residuals_joints, unknowns);
			jacobian_joints << jacobian_joints_translation, jacobian_joints_shape, jacobian_joints_pose;

			// REGULARIZER
			const int residuals_prior = BETA_COUNT + THETA_COMPONENT_COUNT + THETA_COMPONENT_COUNT;
			Eigen::VectorXf error_shape_prior = Eigen::VectorXf::Zero(BETA_COUNT);
			Eigen::VectorXf error_pose_prior = Eigen::VectorXf::Zero(THETA_COMPONENT_COUNT);
			Eigen::VectorXf error_bend_prior = Eigen::VectorXf::Zero(THETA_COMPONENT_COUNT);

			Eigen::MatrixXf jacobian_shape_prior = Eigen::MatrixXf::Zero(BETA_COUNT, BETA_COUNT);
			Eigen::MatrixXf jacobian_pose_prior = Eigen::MatrixXf::Zero(THETA_COMPONENT_COUNT, THETA_COMPONENT_COUNT);
			Eigen::MatrixXf jacobian_bend_prior = Eigen::MatrixXf::Zero(THETA_COMPONENT_COUNT, THETA_COMPONENT_COUNT);

			prior_energy.ComputeShapePriorError(betas, shape_prior_weight_, error_shape_prior);
			prior_energy.ComputePosePriorError(thetas, pose_prior_weight_, error_pose_prior);
			prior_energy.ComputeBendPriorError(thetas, bend_prior_weight_, error_bend_prior);

			prior_energy.ComputeShapePriorJacobian(shape_prior_weight_, jacobian_shape_prior);
			prior_energy.ComputePosePriorJacobian(pose_prior_weight_, jacobian_pose_prior);
			prior_energy.ComputeBendPriorJacobian(thetas, bend_prior_weight_, jacobian_bend_prior);

			Eigen::MatrixXf jacobian_prior = Eigen::MatrixXf::Zero(residuals_prior, unknowns);
			jacobian_prior.block<BETA_COUNT, BETA_COUNT>(0, 3) = jacobian_shape_prior;
			jacobian_prior.block<THETA_COMPONENT_COUNT, THETA_COMPONENT_COUNT>
				(BETA_COUNT, 3 + BETA_COUNT) = jacobian_pose_prior;
			jacobian_prior.block<THETA_COMPONENT_COUNT, THETA_COMPONENT_COUNT>
				(BETA_COUNT + THETA_COMPONENT_COUNT, 3 + BETA_COUNT) = jacobian_bend_prior;

			// Reconstruction
			const int residuals = residuals_joints + residuals_prior;
			Eigen::MatrixXf jacobian(residuals, unknowns);
			Eigen::VectorXf error(residuals);
			Eigen::VectorXf delta = Eigen::VectorXf::Zero(unknowns);

			jacobian << jacobian_joints, jacobian_prior;
			error << error_joints, error_shape_prior, error_pose_prior, error_bend_prior;

			// levenberg marquardt
			bool minimized = lm_solver(jacobian, error, residuals, iteration, delta);
			if (minimized)
			{
				for (int i = 0; i < 3; i++)	translation(i) -= delta(i);
				for (int i = 0; i < BETA_COUNT; i++) betas[i] -= delta(i + 3);
				for (int i = 0; i < THETA_COMPONENT_COUNT; i++) thetas(i) -= delta(i + 3 + BETA_COUNT);
			}
			else
			{
				for (int i = 0; i < 3; i++)	translation(i) += delta(i);
				for (int i = 0; i < BETA_COUNT; i++) betas[i] += delta(i + 3);
				for (int i = 0; i < THETA_COMPONENT_COUNT; i++) thetas(i) += delta(i + 3 + BETA_COUNT);
			}

			if (logging_on)
			{
				if (minimized)
				{
					std::cout << "Translation + Betas + Thetas" << std::endl;
					for (int i = 0; i < 3; i++)	std::cout << translation(i) << std::endl;
					std::cout << "---------------------------------" << std::endl;
					for (int i = 0; i < BETA_COUNT; i++) std::cout << betas[i] << std::endl;
					std::cout << "---------------------------------" << std::endl;
					for (int i = 0; i < THETA_COMPONENT_COUNT; i++) std::cout << thetas(i) << std::endl;
					std::cout << std::endl;
				}

				std::vector<float> joints = Image::Coordinates(
					projector_.FromRegressed(model_joints, translation));

				Image image;
				Image::Draw3D(image, Pixel::White(), projector_, translation, body.vertices);
				Image::Draw2D(image, Pixel::Yellow(), input_joints);
				Image::Draw2D(image, Pixel::Blue(), joints);
				image.SavePNG(output_path + std::to_string(iteration) + ".png");
			}
		}
	}

	void Reconstruction::BodyFromJoints(
		const std::string& output_path,
		const std::vector<float>& input_joints, Eigen::Vector3f& translation, 
		ShapeCoefficients& betas, PoseEulerCoefficients& thetas) const
	{
		bool logging_on = true;
		if (output_path.empty()) logging_on = false;

		JointsEnergy joints_energy(generator_, projector_, regressor_);

		const int unknowns = 3 + BETA_COUNT + THETA_COMPONENT_COUNT;
		LevenbergMarquardt lm_solver(unknowns, logging_on);

		joints_energy.InitializeCameraPosition(regressor_.GetJointType(),
			input_joints, projector_.GetIntrinsics()(1, 1), translation);

		std::vector<float3> dshape(VERTEX_COUNT * BETA_COUNT);
		std::vector<float3> dpose(VERTEX_COUNT * THETA_COMPONENT_COUNT);
		for (int iteration = 0; iteration < iterations_; iteration++)
		{
			if (logging_on)
				std::cout << "Iteration " << iteration << std::endl;

			Body body = generator_(betas, thetas, false);
			RegressedJoints model_joints = regressor_(body.vertices);

			// Prepare Jacobians and Errors
			generator_.ComputeBodyFromShapeJacobian(dshape);
			generator_.ComputeBodyFromPoseJacobian(body, dpose);

			// should be x2 since each point has two components
			const int residuals_joints = static_cast<int>(input_joints.size());
			Eigen::VectorXf error_joints = Eigen::VectorXf::Zero(residuals_joints);
			Eigen::MatrixXf jacobian_joints_translation = Eigen::MatrixXf::Zero(residuals_joints, 3);
			Eigen::MatrixXf jacobian_joints_shape = Eigen::MatrixXf::Zero(residuals_joints, BETA_COUNT);
			Eigen::MatrixXf jacobian_joints_pose = Eigen::MatrixXf::Zero(residuals_joints, THETA_COMPONENT_COUNT);

			joints_energy.ComputeError(input_joints, model_joints, translation,
				residuals_joints, joints_weight_, error_joints);
			joints_energy.ComputeJacobianFromTranslation(body, model_joints, translation,
				residuals_joints, joints_weight_, jacobian_joints_translation);
			joints_energy.ComputeJacobianFromShape(body, model_joints, dshape, translation,
				residuals_joints, joints_weight_, jacobian_joints_shape);
			joints_energy.ComputeJacobianFromPose(body, model_joints, dpose, translation,
				residuals_joints, joints_weight_, jacobian_joints_pose);

			Eigen::MatrixXf jacobian_joints(residuals_joints, unknowns);
			jacobian_joints << jacobian_joints_translation, jacobian_joints_shape, jacobian_joints_pose;

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
			{
				for (int i = 0; i < 3; i++)	translation(i) -= delta(i);
				for (int i = 0; i < BETA_COUNT; i++) betas[i] -= delta(i + 3);
				for (int i = 0; i < THETA_COMPONENT_COUNT; i++) thetas(i) -= delta(i + 3 + BETA_COUNT);
			}
			else
			{
				for (int i = 0; i < 3; i++)	translation(i) += delta(i);
				for (int i = 0; i < BETA_COUNT; i++) betas[i] += delta(i + 3);
				for (int i = 0; i < THETA_COMPONENT_COUNT; i++) thetas(i) += delta(i + 3 + BETA_COUNT);
			}

			if (logging_on)
			{
				if (minimized)
				{
					std::cout << "Translation + Betas + Thetas" << std::endl;
					for (int i = 0; i < 3; i++)	std::cout << translation(i) << std::endl;
					for (int i = 0; i < BETA_COUNT; i++) std::cout << betas[i] << std::endl;
					for (int i = 0; i < THETA_COMPONENT_COUNT; i++) std::cout << thetas(i) << std::endl;
					std::cout << std::endl;
				}

				std::vector<float> joints = Image::Coordinates(
					projector_.FromRegressed(model_joints, translation));

				Image image;
				Image::Draw3D(image, Pixel::White(), projector_, translation, body.vertices);
				Image::Draw2D(image, Pixel::Yellow(), input_joints);
				Image::Draw2D(image, Pixel::Blue(), joints);
				image.SavePNG(output_path + std::to_string(iteration) + ".png");
			}
		}
	}

	void Reconstruction::TranslationFromJoints(
		const std::string& output_path,
		const std::vector<float>& input_joints,
		Eigen::Vector3f& translation) const
	{
		bool logging_on = true;
		if (output_path.empty()) logging_on = false;

		ShapeCoefficients betas;
		PoseEulerCoefficients thetas;
		JointsEnergy joints_energy(generator_, projector_, regressor_);

		const int unknowns = 3;
		LevenbergMarquardt lm_solver(unknowns, logging_on);

		joints_energy.InitializeCameraPosition(regressor_.GetJointType(),
			input_joints, projector_.GetIntrinsics()(1, 1), translation);

		for (int iteration = 0; iteration < iterations_; iteration++)
		{
			if (logging_on)
				std::cout << "Iteration " << iteration << std::endl;

			Body body = generator_(betas, thetas, false);
			RegressedJoints model_joints = regressor_(body.vertices);

			// Prepare Jacobians and Errors

			// should be x2 since each point has two components
			const int residuals_joints = static_cast<int>(input_joints.size());
			Eigen::MatrixXf jacobian_joints = Eigen::MatrixXf::Zero(residuals_joints, unknowns);
			Eigen::VectorXf error_joints = Eigen::VectorXf::Zero(residuals_joints);

			joints_energy.ComputeError(input_joints, model_joints, translation,
				residuals_joints, joints_weight_, error_joints);
			joints_energy.ComputeJacobianFromTranslation(body, model_joints, translation, 
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
					translation(i) -= delta(i);
			else
				for (int i = 0; i < unknowns; i++)
					translation(i) += delta(i);

			if (logging_on)
			{
				if (minimized)
				{
					std::cout << "Translation" << std::endl;
					for (int i = 0; i < unknowns; i++)
						std::cout << translation(i) << " ";
					std::cout << std::endl;
				}

				std::vector<float> joints = Image::Coordinates(
					projector_.FromRegressed(model_joints, translation));

				Image image;
				Image::Draw3D(image, Pixel::White(), projector_, translation, body.vertices);
				Image::Draw2D(image, Pixel::Yellow(), input_joints);
				Image::Draw2D(image, Pixel::Blue(), joints);
				image.SavePNG(output_path + std::to_string(iteration) + ".png");
			}
		}
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
			RegressedJoints model_joints = regressor_(body.vertices);

			// Prepare Jacobians and Errors

			// should be x2 since each point has two components
			const int residuals_joints = static_cast<int>(input_joints.size());
			Eigen::MatrixXf jacobian_joints = Eigen::MatrixXf::Zero(residuals_joints, unknowns);
			Eigen::VectorXf error_joints = Eigen::VectorXf::Zero(residuals_joints);

			joints_energy.ComputeError(input_joints, model_joints, translation,
				residuals_joints, joints_weight_, error_joints);
			generator_.ComputeBodyFromShapeJacobian(dshape);
			joints_energy.ComputeJacobianFromShape(body, model_joints,
				dshape, translation,
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
					projector_.FromRegressed(model_joints, translation));

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
			RegressedJoints model_joints = regressor_(body.vertices);

			// Prepare Jacobians and Errors

			// should be x2 since each point has two components
			const int residuals_joints = static_cast<int>(input_joints.size());
			Eigen::MatrixXf jacobian_joints = Eigen::MatrixXf::Zero(residuals_joints, unknowns);
			Eigen::VectorXf error_joints = Eigen::VectorXf::Zero(residuals_joints);

			joints_energy.ComputeError(input_joints, model_joints, translation,
				residuals_joints, joints_weight_, error_joints);
			generator_.ComputeBodyFromPoseJacobian(body, dpose);
			joints_energy.ComputeJacobianFromPose(body, model_joints,
				dpose, translation,
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
					projector_.FromRegressed(model_joints, translation));

				Image image;
				Image::Draw3D(image, Pixel::White(), projector_, translation, body.vertices);
				Image::Draw2D(image, Pixel::Yellow(), input_joints);
				Image::Draw2D(image, Pixel::Blue(), joints);
				image.SavePNG(output_path + std::to_string(iteration) + ".png");
			}
		}
	}
}