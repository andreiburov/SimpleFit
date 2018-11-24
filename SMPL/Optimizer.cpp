#include "Optimizer.h"
#include "Image.h"
//#include <unsupported/Eigen/NonLinearOptimization>

namespace smpl
{
	void Optimizer::Configuration::ReadOptimizationConfiguration(const std::string& filename)
	{
		std::ifstream file(filename, std::ios::in);
		if (!file)
		{
			MessageBoxA(NULL, std::string("Can not open file ").append(filename).c_str(), "Error", MB_OK);
		}

		std::string line;
		while (std::getline(file, line))
		{
			std::istringstream iss(line);
			std::string key;
			iss >> key;
			if (key.compare("#") == 0)
			{
			}
			else
			{
				float v;
				iss >> v;
				optimization_parameters[key] = v;
			}
		}
	}

	Optimizer::Optimizer(Configuration& configuration, const Generator& generate, const std::vector<float>& tracked_joints) :
		generate_(generate), project_(configuration.intrinsics),
		coco_regress_(configuration.coco_regressor, JointsRegressor::COCO, COCO_JOINT_COUNT), 
		smpl_regress_(configuration.smpl_regressor, JointsRegressor::SMPL, JOINT_COUNT),
		tracked_joints_(tracked_joints), optimization_parameters(std::move(configuration.optimization_parameters)),
		checked_joint_sets_smpl_({
			{ HIP_RIGHT, HIP_LEFT, BACKBONE, CHEST },
			{ KNEE_RIGHT, KNEE_LEFT, PECK_RIGHT, PECK_LEFT, SHOULDER_CENTER },
			{ ANKLE_RIGHT, ANKLE_LEFT, CHIN, SHOULDER_RIGHT, SHOULDER_LEFT },
			{ FOOT_RIGHT, FOOT_LEFT, ELBOW_RIGHT, ELBOW_LEFT },
			{ WRIST_RIGHT, WRIST_LEFT },
			{ HAND_RIGHT, HAND_LEFT }
			}),
		movable_joint_sets_smpl_({
			{ HIP_CENTER, STOMACH },
			{ HIP_RIGHT, HIP_LEFT, BACKBONE, CHEST },
			{ KNEE_RIGHT, KNEE_LEFT, PECK_RIGHT, PECK_LEFT, SHOULDER_CENTER },
			{ ANKLE_RIGHT, ANKLE_LEFT, SHOULDER_RIGHT, SHOULDER_LEFT },
			{ ELBOW_RIGHT, ELBOW_LEFT },
			{ WRIST_RIGHT, WRIST_LEFT }
			}),
		checked_joint_sets_coco_({
			/*{ COCO_SHOULDER_CENTER, COCO_SHOULDER_LEFT, COCO_SHOULDER_RIGHT, COCO_HIP_LEFT, COCO_HIP_RIGHT },*/
			{ COCO_SHOULDER_CENTER, COCO_SHOULDER_LEFT, COCO_SHOULDER_RIGHT, COCO_HIP_LEFT, COCO_HIP_RIGHT },
			{ COCO_NOSE, COCO_ELBOW_LEFT, COCO_ELBOW_RIGHT, COCO_KNEE_LEFT, COCO_KNEE_RIGHT },
			{ COCO_WRIST_LEFT, COCO_WRIST_RIGHT, COCO_ANKLE_LEFT, COCO_ANKLE_RIGHT },
			{ COCO_EYE_LEFT, COCO_EYE_RIGHT, COCO_EAR_LEFT, COCO_EAR_RIGHT }
			}),
		movable_joint_sets_coco_({
			/*{ HIP_CENTER },*/
			{ STOMACH /*, BACKBONE, CHEST, PECK_RIGHT, PECK_LEFT*/ },
			{ HIP_RIGHT, HIP_LEFT, SHOULDER_CENTER, SHOULDER_RIGHT, SHOULDER_LEFT },
			{ KNEE_RIGHT, KNEE_LEFT, ELBOW_RIGHT, ELBOW_LEFT },
			{ SHOULDER_CENTER, CHIN }
			}),
		active_betas_({0,1,2,3,4,5,6,7,8,9})

	{
		ComputeShapeDerivativesAllJointTypes();
	}

	void Optimizer::OptimizeExtrinsics(const std::string& image_filename, const Body& body, Eigen::Vector3f& translation)
	{
		try
		{
			learning_rate_ = optimization_parameters.at("extrinsics_learning_rate");
			iterations_ = (uint)optimization_parameters.at("extrinsics_iterations");
			log_every_ = (uint)optimization_parameters.at("extrinsics_log_every");
		}
		catch (std::out_of_range&)
		{
			MessageBoxA(NULL, "Did not load optimization configuration.", "Error", MB_OK);
		}

		float energy = 1000.f;
		uint count = 0;

		for (uint iteration = 0; iteration < iterations_; iteration++)
		{
			Image image;

			energy = 0.f;

			Eigen::Vector3f dscaling(0.f, 0.f, 0.f);
			Eigen::Vector3f dtranslation(0.f, 0.f, 0.f);

			RegressedJoints joints = coco_regress_(body.vertices);

			for (uint i = 0; i < COCO_JOINT_COUNT; i++)
			{
				Eigen::Vector3f joint = joints.col(i);
				Eigen::Vector3f transformed_joint = joint + translation;
				Eigen::Vector2f projection = project_(transformed_joint);
				Eigen::Vector2f error = Eigen::Vector2f(projection(0) - tracked_joints_[2 * i], projection(1) - tracked_joints_[2 * i + 1]);

				energy += error.squaredNorm();
				//dscaling += Eigen::Scaling(joint) * project_.Jacobian(transformed_joint) * error * 2.f;
				dtranslation += project_.Jacobian(transformed_joint) * error * 2.f;
			}

			//scaling -= learning_rate * dscaling;
			translation -= learning_rate_ * dtranslation;

			if (count % log_every_ == 0)
			{
				Image image(image_filename.c_str());
				Image::Draw3D(image, WHITE, project_, translation, body.vertices);
				Image::Draw3D(image, BLUE, 2, project_, translation, Joints2Vector(joints));
				Image::Draw2D(image, YELLOW, 2, tracked_joints_);
				image.SavePNG(std::string("ExtrinscisTemp/").append(std::to_string(count)).append(".png").c_str());
				std::cout << "Iteration: " << count << std::endl;
				std::cout << "Energy: " << energy << std::endl;
				std::cout << "Translation:" << std::endl << translation << std::endl << std::endl;
			}
			count++;
		}
	}

	void Optimizer::OptimizeShapeFromJoints2D(const JOINT_TYPE& joint_type, const std::string& image_filename,
		const Eigen::Vector3f& translation,
		const PoseEulerCoefficients& thetas, ShapeCoefficients& betas)
	{
		try
		{
			learning_rate_ = optimization_parameters.at("shape_2dreconstruction_learning_rate");
			iterations_ = (uint)optimization_parameters.at("shape_2dreconstruction_iterations");
			log_every_ = (uint)optimization_parameters.at("shape_2dreconstruction_log_every");
		}
		catch (std::out_of_range&)
		{
			MessageBoxA(NULL, "Did not load optimization configuration.", "Error", MB_OK);
		}

		float energy = 1000.f;
		uint count = 0;
		uint joint_count = (joint_type == JOINT_TYPE::COCO ? COCO_JOINT_COUNT : JOINT_COUNT);

		//ComputeShapeDerivatives(joint_type);

		for (uint iteration = 0; iteration < 1000; iteration++)
		{
			Body body = generate_(betas, thetas);

			energy = 0.f;
			float dbetas[BETA_COUNT] = { 0 };

			RegressedJoints joints = (joint_type == JOINT_TYPE::COCO ? coco_regress_(body.vertices) : smpl_regress_(body.vertices));

			for (uint m = 0; m < COCO_JOINT_COUNT; m++)
			{
				Eigen::Vector3f joint = joints.col(m);
				Eigen::Vector3f transformed_joint = joint + translation;
				Eigen::Vector2f projection = project_(transformed_joint);
				Eigen::Vector2f error = Eigen::Vector2f(projection(0) - tracked_joints_[2 * m], projection(1) - tracked_joints_[2 * m + 1]);

				energy += error.squaredNorm();

				for (uint j = 0; j < BETA_COUNT; j++)
				{
					dbetas[j] += 2.f * error.transpose() * project_.Jacobian(transformed_joint).transpose() * dshape_coco_[m*BETA_COUNT + j];
				}
			}

			for (uint j = 0; j < BETA_COUNT; j++)
			{
				betas[j] -= learning_rate_ * dbetas[j];
			}

			if (count % log_every_ == 0)
			{
				Image image(image_filename.c_str());
				Image::Draw3D(image, WHITE, project_, translation, body.vertices);
				Image::Draw3D(image, BLUE, 2, project_, translation, Joints2Vector(joints));
				Image::Draw2D(image, YELLOW, 2, tracked_joints_);
				image.SavePNG(std::string("ShapeReconstructionTemp/").append(std::to_string(count)).append(".png"));
	
				std::cout << "Iteration: " << count << std::endl;
				std::cout << "Energy: " << energy << std::endl;
				std::cout << "Betas: ";
				for (uint j = 0; j < BETA_COUNT; j++)
					std::cout << betas[j] << " ";
				std::cout << std::endl;
			}
			count++;
		}
	}

	void Optimizer::OptimizePoseFromJoints2D(const JOINT_TYPE& joint_type, const std::string& image_filename,
		const Eigen::Vector3f& translation, const ShapeCoefficients& betas,
		PoseEulerCoefficients& thetas)
	{
		float energy = 1000.f;
		uint count = 0;
		std::vector<std::vector<int> > checked_joints_sets;
		std::vector<std::vector<int> > movable_thetas_sets;

		try
		{
			if (joint_type == JOINT_TYPE::COCO)
			{
				learning_rate_ = optimization_parameters.at("coco_pose_2dreconstruction_learning_rate");
				iterations_ = (uint)optimization_parameters.at("coco_pose_2dreconstruction_iterations");
				log_every_ = (uint)optimization_parameters.at("coco_pose_2dreconstruction_log_every");
				checked_joints_sets = {
					{ COCO_SHOULDER_CENTER, COCO_SHOULDER_LEFT, COCO_SHOULDER_RIGHT },
					{ COCO_SHOULDER_CENTER, COCO_SHOULDER_LEFT, COCO_SHOULDER_RIGHT, COCO_HIP_LEFT, COCO_HIP_RIGHT },
					{ COCO_NOSE, COCO_ELBOW_LEFT, COCO_ELBOW_RIGHT, COCO_KNEE_LEFT, COCO_KNEE_RIGHT },
					{ COCO_WRIST_LEFT, COCO_WRIST_RIGHT, COCO_ANKLE_LEFT, COCO_ANKLE_RIGHT },
					{ COCO_EYE_LEFT, COCO_EYE_RIGHT, COCO_EAR_LEFT, COCO_EAR_RIGHT }
				};
				movable_thetas_sets = {
					{ HIP_CENTER },
					{ STOMACH, BACKBONE, CHEST, PECK_RIGHT, PECK_LEFT },
					{ SHOULDER_CENTER, SHOULDER_RIGHT, SHOULDER_LEFT, HIP_RIGHT, HIP_LEFT },
					{ KNEE_RIGHT, KNEE_LEFT, ELBOW_RIGHT, ELBOW_LEFT },
					{ SHOULDER_CENTER, CHIN }
				};
			}
			else
			{
				learning_rate_ = optimization_parameters.at("smpl_pose_2dreconstruction_learning_rate");
				iterations_ = (uint)optimization_parameters.at("smpl_pose_2dreconstruction_iterations");
				log_every_ = (uint)optimization_parameters.at("smpl_pose_2dreconstruction_log_every");
				checked_joints_sets = {
					//{HIP_CENTER, STOMACH}, 
					{ HIP_RIGHT, HIP_LEFT, BACKBONE, CHEST },
					{ KNEE_RIGHT, KNEE_LEFT, PECK_RIGHT, PECK_LEFT, SHOULDER_CENTER },
					{ ANKLE_RIGHT, ANKLE_LEFT, CHIN, SHOULDER_RIGHT, SHOULDER_LEFT },
					{ FOOT_RIGHT, FOOT_LEFT, ELBOW_RIGHT, ELBOW_LEFT },
					{ WRIST_RIGHT, WRIST_LEFT },
					{ HAND_RIGHT, HAND_LEFT }
				};
				movable_thetas_sets = {
					{ HIP_CENTER, STOMACH },
					{ HIP_RIGHT, HIP_LEFT, BACKBONE, CHEST },
					{ KNEE_RIGHT, KNEE_LEFT, PECK_RIGHT, PECK_LEFT, SHOULDER_CENTER },
					{ ANKLE_RIGHT, ANKLE_LEFT, SHOULDER_RIGHT, SHOULDER_LEFT },
					{ ELBOW_RIGHT, ELBOW_LEFT },
					{ WRIST_RIGHT, WRIST_LEFT }
				};
			}
		}
		catch (std::out_of_range&)
		{
			MessageBoxA(NULL, "Did not load optimization configuration.", "Error", MB_OK);
		}

		const std::vector<Skin>& skins = generate_.GetSkins();

		for (int l = 0; l < checked_joints_sets.size(); l++)
		{
			std::cout << "NEW ACTIVE JOINT SET" << std::endl;
			for (uint iteration = 0; iteration < iterations_; iteration++)
			{
				energy = 0.f;
				Body body = generate_(betas, thetas);
				
				float dthetas[JOINT_COUNT * 3] = { 0 };

				RegressedJoints smpl_joints = smpl_regress_(body.vertices);
				RegressedJoints reconstruction_joints = (joint_type == JOINT_TYPE::COCO ? coco_regress_(body.vertices) : smpl_regress_(body.vertices));
				Eigen::Matrix4f* dskinning = new Eigen::Matrix4f[JOINT_COUNT * JOINT_COUNT * 3];
				ComputeSkinningDerivatives(thetas, smpl_joints, dskinning);

				for (auto& m : checked_joints_sets[l])
				{
					Eigen::Vector3f joint = reconstruction_joints.col(m);
					Eigen::Vector3f transformed_joint = joint + translation;
					Eigen::Vector2f projection = project_(transformed_joint);
					Eigen::Vector2f error = Eigen::Vector2f(
						projection(0) - tracked_joints_[2 * m], 
						projection(1) - tracked_joints_[2 * m + 1]);

					energy += error.squaredNorm();

					Eigen::VectorXf regressor_m = (joint_type == JOINT_TYPE::COCO ? coco_regress_.JointRegressor(m) : smpl_regress_.JointRegressor(m));

					Eigen::Vector3f djoint_alpha(0, 0, 0);
					Eigen::Vector3f djoint_beta(0, 0, 0);
					Eigen::Vector3f djoint_gamma(0, 0, 0);

					for (int k = 0; k < JOINT_COUNT; k++) // Update dthetas
					{
#pragma omp parallel for
						for (int i = 0; i < VERTEX_COUNT; i++)
						{
							if (regressor_m(i) > 0.001f)
							{
								Eigen::Vector4f temp;

								temp =
									(skins[i].weight.x * dskinning[ALPHA(k) * THETA_COUNT + skins[i].joint_index.x] +
										skins[i].weight.y * dskinning[ALPHA(k) * THETA_COUNT + skins[i].joint_index.y] +
										skins[i].weight.z * dskinning[ALPHA(k) * THETA_COUNT + skins[i].joint_index.z] +
										skins[i].weight.w * dskinning[ALPHA(k) * THETA_COUNT + skins[i].joint_index.w]) *
									body.deformed_template[i].ToEigen().homogeneous();

								djoint_alpha(0) = regressor_m(i) * temp(0);
								djoint_alpha(1) = regressor_m(i) * temp(1);
								djoint_alpha(2) = regressor_m(i) * temp(2);

								temp =
									(skins[i].weight.x * dskinning[BETA(k) * THETA_COUNT + skins[i].joint_index.x] +
										skins[i].weight.y * dskinning[BETA(k) * THETA_COUNT + skins[i].joint_index.y] +
										skins[i].weight.z * dskinning[BETA(k) * THETA_COUNT + skins[i].joint_index.z] +
										skins[i].weight.w * dskinning[BETA(k) * THETA_COUNT + skins[i].joint_index.w]) *
									body.deformed_template[i].ToEigen().homogeneous();

								djoint_beta(0) = regressor_m(i) * temp(0);
								djoint_beta(1) = regressor_m(i) * temp(1);
								djoint_beta(2) = regressor_m(i) * temp(2);

								temp =
									(skins[i].weight.x * dskinning[GAMMA(k) * THETA_COUNT + skins[i].joint_index.x] +
										skins[i].weight.y * dskinning[GAMMA(k) * THETA_COUNT + skins[i].joint_index.y] +
										skins[i].weight.z * dskinning[GAMMA(k) * THETA_COUNT + skins[i].joint_index.z] +
										skins[i].weight.w * dskinning[GAMMA(k) * THETA_COUNT + skins[i].joint_index.w]) *
									body.deformed_template[i].ToEigen().homogeneous();

								djoint_gamma(0) = regressor_m(i) * temp(0);
								djoint_gamma(1) = regressor_m(i) * temp(1);
								djoint_gamma(2) = regressor_m(i) * temp(2);
							}
						}

						dthetas[ALPHA(k)] += 2.f * error.transpose() * project_.Jacobian(transformed_joint).transpose() * djoint_alpha;
						dthetas[BETA(k)] += 2.f * error.transpose() * project_.Jacobian(transformed_joint).transpose() * djoint_beta;
						dthetas[GAMMA(k)] += 2.f * error.transpose() * project_.Jacobian(transformed_joint).transpose() * djoint_gamma;
					}
				}

				for (auto& m : movable_thetas_sets[l])
				{
					thetas(ALPHA(m)) -= learning_rate_ * dthetas[ALPHA(m)];
					thetas(BETA(m)) -= learning_rate_ * dthetas[BETA(m)];
					thetas(GAMMA(m)) -= learning_rate_ * dthetas[GAMMA(m)];
				}

				if (count % log_every_ == 0)
				{
					Image image(image_filename.c_str());
					Image::Draw3D(image, WHITE, project_, translation, body.vertices);
					Image::Draw3D(image, BLUE, 2, project_, translation, Joints2Vector(reconstruction_joints));

					std::vector<float3> checked_joint_coordinates;
					checked_joint_coordinates.reserve(10);
					for (auto&m : checked_joints_sets[l])
					{
						checked_joint_coordinates.push_back(float3(reconstruction_joints.col(m)));
					}
					Image::Draw3D(image, RED, 2, project_, translation, checked_joint_coordinates);

					std::vector<float3> movable_joint_coordinates;
					movable_joint_coordinates.reserve(10);
					for (auto&m : movable_thetas_sets[l])
					{
						movable_joint_coordinates.push_back(float3(smpl_joints.col(m)));
					}
					Image::Draw3D(image, GREEN, 2, project_, translation, movable_joint_coordinates);
					Image::Draw2D(image, YELLOW, 2, tracked_joints_);
					
					image.SavePNG(std::string("PoseReconstructionTemp2D/").append(std::to_string(count)).append(".png"));
					std::cout << "Iteration: " << count << std::endl;
					std::cout << "Energy: " << energy << std::endl;
					std::cout << "Thetas: ";
					for (uint j = 0; j < THETA_COUNT * 3; j++)
						std::cout << thetas(j) << " ";
					std::cout << std::endl;
					std::cout << "dThetas: ";
					for (uint j = 0; j < THETA_COUNT * 3; j++)
						std::cout << dthetas[j] << " ";
					std::cout << std::endl;
				}
				count++;

				delete[] dskinning;
			}
		}
	}

	void Optimizer::OptimizePoseFromSmplJoints3D(const ShapeCoefficients& betas, PoseEulerCoefficients& thetas)
	{
		try
		{
			learning_rate_ = optimization_parameters.at("smpl_pose_3dreconstruction_learning_rate");
			iterations_ = (uint)optimization_parameters.at("smpl_pose_3dreconstruction_iterations");
			log_every_ = (uint)optimization_parameters.at("smpl_pose_3dreconstruction_log_every");
		}
		catch (std::out_of_range&)
		{
			MessageBoxA(NULL, "Did not load optimization configuration.", "Error", MB_OK);
		}

		float energy = 1000.f;
		uint count = 0;

		const std::vector<Skin>& skins = generate_.GetSkins();

		std::vector<std::vector<int> > active_joints_sets = {
			//{HIP_CENTER, STOMACH}, 
			{ HIP_RIGHT, HIP_LEFT, BACKBONE, CHEST },
			{ KNEE_RIGHT, KNEE_LEFT, PECK_RIGHT, PECK_LEFT, SHOULDER_CENTER },
			{ ANKLE_RIGHT, ANKLE_LEFT, CHIN, SHOULDER_RIGHT, SHOULDER_LEFT },
			{ FOOT_RIGHT, FOOT_LEFT, ELBOW_RIGHT, ELBOW_LEFT},
			{ WRIST_RIGHT, WRIST_LEFT }, 
			{ HAND_RIGHT, HAND_LEFT }
		};

		for (auto& active_joints : active_joints_sets)
		{
			std::cout << "NEW ACTIVE JOINT SET" << std::endl;
			for (uint iteration = 0; iteration < iterations_; iteration++)
			{
				Body body = generate_(betas, thetas);

				energy = 0.f;
				float dthetas[THETA_COUNT * 3] = { 0 };
				RegressedJoints joints = smpl_regress_(body.vertices);
				Eigen::Matrix4f* dskinning = new Eigen::Matrix4f[JOINT_COUNT * JOINT_COUNT * 3];
				ComputeSkinningDerivatives(thetas, joints, dskinning);

				for (auto& m : active_joints)
				{
					Eigen::Vector3f joint = joints.col(m);
					Eigen::Vector3f error = Eigen::Vector3f(
						joint(0) - tracked_joints_[3 * m],
						joint(1) - tracked_joints_[3 * m + 1],
						joint(2) - tracked_joints_[3 * m + 2]);

					energy += error.squaredNorm();

					Eigen::VectorXf regressor_m = smpl_regress_.JointRegressor(m);

					Eigen::Vector3f djoint_alpha(0, 0, 0);
					Eigen::Vector3f djoint_beta(0, 0, 0);
					Eigen::Vector3f djoint_gamma(0, 0, 0);

					for (int k = 0; k < JOINT_COUNT; k++) // Update dthetas
					{
#pragma omp parallel for
						for (int i = 0; i < VERTEX_COUNT; i++)
						{
							if (regressor_m(i) > 0.001f)
							{
								Eigen::Vector4f temp;

								temp =
									(skins[i].weight.x * dskinning[ALPHA(k) * THETA_COUNT + skins[i].joint_index.x] +
										skins[i].weight.y * dskinning[ALPHA(k) * THETA_COUNT + skins[i].joint_index.y] +
										skins[i].weight.z * dskinning[ALPHA(k) * THETA_COUNT + skins[i].joint_index.z] +
										skins[i].weight.w * dskinning[ALPHA(k) * THETA_COUNT + skins[i].joint_index.w]) *
									body.deformed_template[i].ToEigen().homogeneous();

								djoint_alpha(0) = regressor_m(i) * temp(0);
								djoint_alpha(1) = regressor_m(i) * temp(1);
								djoint_alpha(2) = regressor_m(i) * temp(2);

								temp =
									(skins[i].weight.x * dskinning[BETA(k) * THETA_COUNT + skins[i].joint_index.x] +
										skins[i].weight.y * dskinning[BETA(k) * THETA_COUNT + skins[i].joint_index.y] +
										skins[i].weight.z * dskinning[BETA(k) * THETA_COUNT + skins[i].joint_index.z] +
										skins[i].weight.w * dskinning[BETA(k) * THETA_COUNT + skins[i].joint_index.w]) *
									body.deformed_template[i].ToEigen().homogeneous();

								djoint_beta(0) = regressor_m(i) * temp(0);
								djoint_beta(1) = regressor_m(i) * temp(1);
								djoint_beta(2) = regressor_m(i) * temp(2);

								temp =
									(skins[i].weight.x * dskinning[GAMMA(k) * THETA_COUNT + skins[i].joint_index.x] +
										skins[i].weight.y * dskinning[GAMMA(k) * THETA_COUNT + skins[i].joint_index.y] +
										skins[i].weight.z * dskinning[GAMMA(k) * THETA_COUNT + skins[i].joint_index.z] +
										skins[i].weight.w * dskinning[GAMMA(k) * THETA_COUNT + skins[i].joint_index.w]) *
									body.deformed_template[i].ToEigen().homogeneous();

								djoint_gamma(0) = regressor_m(i) * temp(0);
								djoint_gamma(1) = regressor_m(i) * temp(1);
								djoint_gamma(2) = regressor_m(i) * temp(2);
							}
						}

						dthetas[ALPHA(k)] += 2.f * error.transpose() * djoint_alpha;
						dthetas[BETA(k)] += 2.f * error.transpose() * djoint_beta;
						dthetas[GAMMA(k)] += 2.f * error.transpose() * djoint_gamma;
					}
				}

				for (auto& m : active_joints)
				{
					thetas(ALPHA(PARENT_INDEX[m])) -= learning_rate_ * dthetas[ALPHA(PARENT_INDEX[m])];
					thetas(BETA(PARENT_INDEX[m])) -= learning_rate_ * dthetas[BETA(PARENT_INDEX[m])];
					thetas(GAMMA(PARENT_INDEX[m])) -= learning_rate_ * dthetas[GAMMA(PARENT_INDEX[m])];
				}

				if (count % log_every_ == 0)
				{
					body.Dump(std::string("PoseReconstructionTemp/").append(std::to_string(count)).append(".obj"));
					std::cout << "Iteration: " << count << std::endl;
					std::cout << "Energy: " << energy << std::endl;
					std::cout << "Thetas: ";
					for (uint j = 0; j < THETA_COUNT * 3; j++)
						std::cout << thetas(j) << " ";
					std::cout << std::endl;
					std::cout << "dThetas: ";
					for (uint j = 0; j < THETA_COUNT * 3; j++)
						std::cout << dthetas[j] << " ";
					std::cout << std::endl;
				}
				count++;

				delete[] dskinning;
			}
		}
	}

	void Optimizer::ReconstructCoarse(const std::string& image_filename, Eigen::Vector3f& translation,
		ShapeCoefficients& betas, PoseEulerCoefficients& thetas, uint& count)
	{
		try
		{
			iterations_ = (uint)optimization_parameters.at("lm_reconstruction_iterations");
		}
		catch (std::out_of_range&)
		{
			MessageBoxA(NULL, "Did not load optimization configuration.", "Error", MB_OK);
		}

		const int residuals = COCO_JOINT_COUNT * 2 + 1; // projection error, beta0
		const int unknowns = 3 + 3 + 1; // translation, theta0, beta0
		const float shape_prior_weight = 10.f;
		float lambda = 2.0f;
		float lambda_min = 0.1f;
		float alpha = 1.8f;
		float beta = 0.6f;
		const float MINF = -100000.f;
		float last_residual_error = MINF;

		std::vector<int> coarse_joints = { COCO_SHOULDER_CENTER, COCO_SHOULDER_LEFT, COCO_SHOULDER_RIGHT, COCO_HIP_LEFT, COCO_HIP_RIGHT };

		Eigen::VectorXf delta_old = Eigen::VectorXf::Zero(unknowns);

		for (uint iteration = 0; iteration < iterations_; iteration++)
		{
			Eigen::MatrixXf jacobian = Eigen::MatrixXf::Zero(residuals, unknowns);
			Eigen::VectorXf error = Eigen::VectorXf::Zero(residuals);
			Eigen::VectorXf delta = Eigen::VectorXf::Zero(unknowns);

			Body body = generate_(betas, thetas);
			RegressedJoints coco_joints = coco_regress_(body.vertices);
			RegressedJoints smpl_joints = smpl_regress_(body.vertices);

			std::vector<Eigen::Vector3f> transformed_joint(COCO_JOINT_COUNT, Eigen::Vector3f(0.f, 0.f, 0.f));

			// Evaluate error
			for (auto& m : coarse_joints)
			{
				transformed_joint[m] = coco_joints.col(m) + translation;
				Eigen::Vector2f projection = project_(transformed_joint[m]);
				error(2 * m) = projection(0) - tracked_joints_[2 * m];
				error(2 * m + 1) = projection(1) - tracked_joints_[2 * m + 1];
			}
			// beta0
			error(COCO_JOINT_COUNT * 2) = shape_prior_weight * betas[0];

			// Evaluate jacobian
			Eigen::MatrixXf dtranslation = Eigen::MatrixXf::Zero(residuals, 3);
			for (auto& m : coarse_joints)
			{
				dtranslation.block<2, 3>(2 * m, 0) = project_.Jacobian(transformed_joint[m]).transpose();
			}

			Eigen::MatrixXf dtheta0 = Eigen::MatrixXf::Zero(residuals, 3);
			{
				const std::vector<Skin>& skins = generate_.GetSkins();
				Eigen::Matrix4f* dskinning = new Eigen::Matrix4f[JOINT_COUNT * JOINT_COUNT * 3];
				ComputeSkinningDerivatives(thetas, smpl_joints, dskinning);
				for (auto& m : coarse_joints)
				{
					Eigen::VectorXf regressor_m = coco_regress_.JointRegressor(m);
					Eigen::Vector3f djoint_alpha(0, 0, 0);
					Eigen::Vector3f djoint_beta(0, 0, 0);
					Eigen::Vector3f djoint_gamma(0, 0, 0);

					for (uint i = 0; i < VERTEX_COUNT; i++)
					{
						if (regressor_m(i) > 0.001f)
						{
							Eigen::Vector4f temp;

							temp =
								(skins[i].weight.x * dskinning[ALPHA(0) * THETA_COUNT + skins[i].joint_index.x] +
									skins[i].weight.y * dskinning[ALPHA(0) * THETA_COUNT + skins[i].joint_index.y] +
									skins[i].weight.z * dskinning[ALPHA(0) * THETA_COUNT + skins[i].joint_index.z] +
									skins[i].weight.w * dskinning[ALPHA(0) * THETA_COUNT + skins[i].joint_index.w]) *
								body.deformed_template[i].ToEigen().homogeneous();

							djoint_alpha(0) += regressor_m(i) * temp(0);
							djoint_alpha(1) += regressor_m(i) * temp(1);
							djoint_alpha(2) += regressor_m(i) * temp(2);

							temp =
								(skins[i].weight.x * dskinning[BETA(0) * THETA_COUNT + skins[i].joint_index.x] +
									skins[i].weight.y * dskinning[BETA(0) * THETA_COUNT + skins[i].joint_index.y] +
									skins[i].weight.z * dskinning[BETA(0) * THETA_COUNT + skins[i].joint_index.z] +
									skins[i].weight.w * dskinning[BETA(0) * THETA_COUNT + skins[i].joint_index.w]) *
								body.deformed_template[i].ToEigen().homogeneous();

							djoint_beta(0) += regressor_m(i) * temp(0);
							djoint_beta(1) += regressor_m(i) * temp(1);
							djoint_beta(2) += regressor_m(i) * temp(2);

							temp =
								(skins[i].weight.x * dskinning[GAMMA(0) * THETA_COUNT + skins[i].joint_index.x] +
									skins[i].weight.y * dskinning[GAMMA(0) * THETA_COUNT + skins[i].joint_index.y] +
									skins[i].weight.z * dskinning[GAMMA(0) * THETA_COUNT + skins[i].joint_index.z] +
									skins[i].weight.w * dskinning[GAMMA(0) * THETA_COUNT + skins[i].joint_index.w]) *
								body.deformed_template[i].ToEigen().homogeneous();

							djoint_gamma(0) += regressor_m(i) * temp(0);
							djoint_gamma(1) += regressor_m(i) * temp(1);
							djoint_gamma(2) += regressor_m(i) * temp(2);
						}
					}

					dtheta0.block<2, 1>(2 * m, ALPHA(0)) += project_.Jacobian(transformed_joint[m]).transpose() * djoint_alpha;
					dtheta0.block<2, 1>(2 * m, BETA(0)) += project_.Jacobian(transformed_joint[m]).transpose() * djoint_beta;
					dtheta0.block<2, 1>(2 * m, GAMMA(0)) += project_.Jacobian(transformed_joint[m]).transpose() * djoint_gamma;
				}
				delete dskinning;
			}

			Eigen::MatrixXf dbetas = Eigen::MatrixXf::Zero(residuals, 1);
			for (auto& m : coarse_joints)
			{
				dbetas.block<2, 1>(2 * m, 0) = project_.Jacobian(transformed_joint[m]).transpose() * dshape_coco_[m*BETA_COUNT];
			}
			// beta0
			dbetas(COCO_JOINT_COUNT * 2, 0) = shape_prior_weight;

			jacobian << dtranslation, dtheta0, dbetas;

			Log(image_filename, body, coco_joints, smpl_joints, translation, 0, iteration + count);
			std::cout << "Iteration: " << iteration + count << std::endl;
			std::cout << "Error: " << error.squaredNorm() << std::endl;
			std::cout << "Translation:" << std::endl << translation(0) << " " << translation(1) << " " << translation(2) << std::endl;
			std::cout << "Betas:" << std::endl << betas << std::endl;
			std::cout << "Thetas:" << std::endl << thetas << std::endl;

			Eigen::MatrixXf Jt = jacobian.transpose();
			Eigen::MatrixXf JtJ = Jt * jacobian;
			Eigen::VectorXf JtF = Jt * error;
			Eigen::MatrixXf JtJ_diag = JtJ.diagonal().asDiagonal();

			float current_residual_error = error.squaredNorm();
			if (last_residual_error == MINF || (last_residual_error >= current_residual_error && iteration + 1 != iterations_))
			{
				JtJ += lambda * JtJ_diag;

				Eigen::ConjugateGradient<Eigen::MatrixXf, Eigen::Lower | Eigen::Upper> cg;
				cg.compute(JtJ);
				delta = cg.solve(JtF);

				delta_old = delta;
				last_residual_error = current_residual_error;

				for (uint i = 0; i < 3; i++)
					translation(i) -= delta(i);
				thetas(ALPHA(0)) -= delta(3 + ALPHA(0));
				thetas(BETA(0)) -= delta(3 + BETA(0));
				thetas(GAMMA(0)) -= delta(3 + GAMMA(0));
				betas[0] -= delta(3 + 3);

				lambda *= beta;
				if (lambda < lambda_min) lambda = lambda_min;
			}
			else if (last_residual_error < current_residual_error)
			{
				for (uint i = 0; i < 3; i++)
					translation(i) += delta_old(i);
				thetas(ALPHA(0)) += delta_old(3 + ALPHA(0));
				thetas(BETA(0)) += delta_old(3 + BETA(0));
				thetas(GAMMA(0)) += delta_old(3 + GAMMA(0));
				betas[0] += delta_old(3 + 3);

				lambda *= alpha;
			}
		}

		count += iterations_;
	}

	void Optimizer::ReconstructCamera(const std::string& image_filename, Eigen::Vector3f& translation,
		ShapeCoefficients& betas, PoseEulerCoefficients& thetas, uint& count)
	{
		iterations_ = 10;

		const int residuals = COCO_JOINT_COUNT * 2 + 1 + 1; // projection error, beta0, theta0 pitch
		const int unknowns = 3 + 1 + 3; // translation, beta0, theta0
		const float shape_prior_weight = 10.f;
		const float pose_prior_weight = 1000.f;
		float lambda = 2.0f;
		float lambda_min = 0.1f;
		float alpha = 1.8f;
		float beta = 0.6f;
		const float MINF = -100000.f;
		float last_residual_error = MINF;

		std::vector<int> coarse_joints = { /*COCO_SHOULDER_CENTER, */COCO_SHOULDER_LEFT, COCO_SHOULDER_RIGHT, COCO_HIP_LEFT, COCO_HIP_RIGHT };
		Eigen::VectorXf delta_old = Eigen::VectorXf::Zero(unknowns);

		// initialize camera distance assuming figure pitch == 0
		{
			Body body = generate_(betas, thetas);
			RegressedJoints coco_joints = coco_regress_(body.vertices);

			float model_avg_shoulders_to_hips = (coco_joints.col(COCO_SHOULDER_LEFT) + coco_joints.col(COCO_SHOULDER_RIGHT)
				- coco_joints.col(COCO_HIP_LEFT) - coco_joints.col(COCO_HIP_RIGHT)).norm();
			float fas2h_x = tracked_joints_[2 * COCO_SHOULDER_LEFT] + tracked_joints_[2 * COCO_SHOULDER_LEFT]
				- tracked_joints_[2 * COCO_HIP_LEFT] - tracked_joints_[2 * COCO_HIP_RIGHT];
			float fas2h_y = tracked_joints_[2 * COCO_SHOULDER_LEFT + 1] + tracked_joints_[2 * COCO_SHOULDER_LEFT + 1]
				- tracked_joints_[2 * COCO_HIP_LEFT + 1] - tracked_joints_[2 * COCO_HIP_RIGHT + 1];
			float figure_avg_shoulders_to_hips = sqrt(fas2h_x * fas2h_x + fas2h_y * fas2h_y);
			float focal_length_y = project_.GetIntrinsics()(1, 1);

			float z = model_avg_shoulders_to_hips / figure_avg_shoulders_to_hips * focal_length_y;
			translation = Eigen::Vector3f(0.f, 0.f, -z);

			std::cout << "Model average shoulders2hips: " << model_avg_shoulders_to_hips << std::endl;
			std::cout << "Figure average shoulders2hips: " << figure_avg_shoulders_to_hips << std::endl;
			std::cout << "Focal length y: " << focal_length_y << std::endl;
			std::cout << "Camera distance: " << z << std::endl;
		}

		for (uint iteration = 0; iteration < iterations_; iteration++)
		{
			Eigen::MatrixXf jacobian = Eigen::MatrixXf::Zero(residuals, unknowns);
			Eigen::VectorXf error = Eigen::VectorXf::Zero(residuals);
			Eigen::VectorXf delta = Eigen::VectorXf::Zero(unknowns);

			Body body = generate_(betas, thetas);
			RegressedJoints coco_joints = coco_regress_(body.vertices);
			RegressedJoints smpl_joints = smpl_regress_(body.vertices);

			std::vector<Eigen::Vector3f> transformed_joint(COCO_JOINT_COUNT, Eigen::Vector3f(0.f, 0.f, 0.f));

			// Evaluate error
			for (auto& m : coarse_joints)
			{
				transformed_joint[m] = coco_joints.col(m) + translation;
				Eigen::Vector2f projection = project_(transformed_joint[m]);
				error(2 * m) = projection(0) - tracked_joints_[2 * m];
				error(2 * m + 1) = projection(1) - tracked_joints_[2 * m + 1];
			}
			// beta0
			error(COCO_JOINT_COUNT * 2) = shape_prior_weight * betas[0];
			// theta0 pitch
			error(COCO_JOINT_COUNT * 2 + 1) = pose_prior_weight * thetas(0);

			// Evaluate jacobian
			Eigen::MatrixXf dtranslation = Eigen::MatrixXf::Zero(residuals, 3);
			for (auto& m : coarse_joints)
			{
				dtranslation.block<2, 3>(2 * m, 0) = project_.Jacobian(transformed_joint[m]).transpose();
			}

			Eigen::MatrixXf dbetas = Eigen::MatrixXf::Zero(residuals, 1);
			for (auto& m : coarse_joints)
			{
				dbetas.block<2, 1>(2 * m, 0) = project_.Jacobian(transformed_joint[m]).transpose() * dshape_coco_[m*BETA_COUNT];
			}
			// beta0
			dbetas(COCO_JOINT_COUNT * 2, 0) = shape_prior_weight;

			Eigen::MatrixXf dtheta0 = Eigen::MatrixXf::Zero(residuals, 3);
			{
				const std::vector<Skin>& skins = generate_.GetSkins();
				Eigen::Matrix4f* dskinning = new Eigen::Matrix4f[JOINT_COUNT * JOINT_COUNT * 3];
				ComputeSkinningDerivatives(thetas, smpl_joints, dskinning);
				for (auto& m : coarse_joints)
				{
					Eigen::VectorXf regressor_m = coco_regress_.JointRegressor(m);
					Eigen::Vector3f djoint_alpha(0, 0, 0);
					Eigen::Vector3f djoint_beta(0, 0, 0);
					Eigen::Vector3f djoint_gamma(0, 0, 0);

					for (uint i = 0; i < VERTEX_COUNT; i++)
					{
						if (regressor_m(i) > 0.001f)
						{
							Eigen::Vector4f temp;

							temp =
								(skins[i].weight.x * dskinning[ALPHA(0) * THETA_COUNT + skins[i].joint_index.x] +
									skins[i].weight.y * dskinning[ALPHA(0) * THETA_COUNT + skins[i].joint_index.y] +
									skins[i].weight.z * dskinning[ALPHA(0) * THETA_COUNT + skins[i].joint_index.z] +
									skins[i].weight.w * dskinning[ALPHA(0) * THETA_COUNT + skins[i].joint_index.w]) *
								body.deformed_template[i].ToEigen().homogeneous();

							djoint_alpha(0) += regressor_m(i) * temp(0);
							djoint_alpha(1) += regressor_m(i) * temp(1);
							djoint_alpha(2) += regressor_m(i) * temp(2);

							temp =
								(skins[i].weight.x * dskinning[BETA(0) * THETA_COUNT + skins[i].joint_index.x] +
									skins[i].weight.y * dskinning[BETA(0) * THETA_COUNT + skins[i].joint_index.y] +
									skins[i].weight.z * dskinning[BETA(0) * THETA_COUNT + skins[i].joint_index.z] +
									skins[i].weight.w * dskinning[BETA(0) * THETA_COUNT + skins[i].joint_index.w]) *
								body.deformed_template[i].ToEigen().homogeneous();

							djoint_beta(0) += regressor_m(i) * temp(0);
							djoint_beta(1) += regressor_m(i) * temp(1);
							djoint_beta(2) += regressor_m(i) * temp(2);

							temp =
								(skins[i].weight.x * dskinning[GAMMA(0) * THETA_COUNT + skins[i].joint_index.x] +
									skins[i].weight.y * dskinning[GAMMA(0) * THETA_COUNT + skins[i].joint_index.y] +
									skins[i].weight.z * dskinning[GAMMA(0) * THETA_COUNT + skins[i].joint_index.z] +
									skins[i].weight.w * dskinning[GAMMA(0) * THETA_COUNT + skins[i].joint_index.w]) *
								body.deformed_template[i].ToEigen().homogeneous();

							djoint_gamma(0) += regressor_m(i) * temp(0);
							djoint_gamma(1) += regressor_m(i) * temp(1);
							djoint_gamma(2) += regressor_m(i) * temp(2);
						}
					}

					dtheta0.block<2, 1>(2 * m, ALPHA(0)) += project_.Jacobian(transformed_joint[m]).transpose() * djoint_alpha;
					dtheta0.block<2, 1>(2 * m, BETA(0)) += project_.Jacobian(transformed_joint[m]).transpose() * djoint_beta;
					dtheta0.block<2, 1>(2 * m, GAMMA(0)) += project_.Jacobian(transformed_joint[m]).transpose() * djoint_gamma;
				}
				delete dskinning;

				dtheta0(COCO_JOINT_COUNT * 2 + 1, 0) = pose_prior_weight;
			}

			jacobian << dtranslation, dbetas, dtheta0;

			//Log(image_filename, body, coco_joints, smpl_joints, translation, 0, iteration + count);
			std::cout << "Iteration: " << iteration + count << std::endl;
			std::cout << "Error: " << error.squaredNorm() << std::endl;
			std::cout << "Translation:" << std::endl << translation(0) << " " << translation(1) << " " << translation(2) << std::endl;
			std::cout << "Betas:" << std::endl << betas << std::endl;
			std::cout << "Thetas:" << std::endl << thetas << std::endl;

			Eigen::MatrixXf Jt = jacobian.transpose();
			Eigen::MatrixXf JtJ = Jt * jacobian;
			Eigen::VectorXf JtF = Jt * error;
			Eigen::MatrixXf JtJ_diag = JtJ.diagonal().asDiagonal();

			float current_residual_error = error.squaredNorm();
			if (last_residual_error == MINF || (last_residual_error >= current_residual_error && iteration + 1 != iterations_))
			{
				JtJ += lambda * JtJ_diag;

				Eigen::ConjugateGradient<Eigen::MatrixXf, Eigen::Lower | Eigen::Upper> cg;
				cg.compute(JtJ);
				delta = cg.solve(JtF);

				delta_old = delta;
				last_residual_error = current_residual_error;

				for (uint i = 0; i < 3; i++)
					translation(i) -= delta(i);
				betas[0] -= delta(3);
				thetas(ALPHA(0)) -= delta(4 + ALPHA(0));
				thetas(BETA(0)) -= delta(4 + BETA(0));
				thetas(GAMMA(0)) -= delta(4 + GAMMA(0));

				lambda *= beta;
				if (lambda < lambda_min) lambda = lambda_min;
			}
			else if (last_residual_error < current_residual_error)
			{
				for (uint i = 0; i < 3; i++)
					translation(i) += delta_old(i);
				betas[0] += delta_old(3);
				thetas(ALPHA(0)) += delta_old(4 + ALPHA(0));
				thetas(BETA(0)) += delta_old(4 + BETA(0));
				thetas(GAMMA(0)) += delta_old(4 + GAMMA(0));

				lambda *= alpha;
			}
		}

		//count += iterations_;
	}

	float Optimizer::ReconstructTotal(const std::string& image_filename, Eigen::Vector3f& translation,
		ShapeCoefficients& betas, PoseEulerCoefficients& thetas, uint& count)
	{
		iterations_ = 100;

		const int residuals = COCO_JOINT_COUNT * 2 + BETA_COUNT + THETA_COUNT * 6; // projection error, betas, thetas, bends
		const int unknowns = BETA_COUNT + THETA_COUNT * 3; // betas, thetas
		const float shape_prior_weight = 5.f;
		const float pose_prior_weight = 10.f;
		const float bend_prior_weight = 10.f;
		float lambda = 2.0f;
		float lambda_min = 0.1f;
		float alpha = 1.8f;
		float beta = 0.6f;
		const float MINF = -100000.f;
		float last_residual_error;

		Eigen::VectorXf delta_old = Eigen::VectorXf::Zero(unknowns);

		last_residual_error = MINF;

		std::vector<float> pose_prior = {
			10, 10, 10, // 0
			1, 2, 2, //1 
			1, 2, 2, //2
			2, 4, 4, //3
			1, 100, 100, //4
			1, 100, 100, //5
			2, 4, 4, //6
			100, 100, 100, //7
			100, 100, 100, //8
			10, 4, 4, //9
			100, 100, 100, //10
			100, 100, 100, //11
			1, 1, 2, //12
			10, 10, 10, //13
			10, 10, 10, //14
			4, 2, 4, //15
			2, 1, 1, //16
			2, 1, 1, //17
			100, 100, 1, //18
			100, 100, 1, //19
			100, 100, 100, //20
			100, 100, 100, //21
			100, 100, 100, //22
			100, 100, 100  //23
		};

		std::vector<int> bend_mask = {  
			0, 0, 0, //0 
			1, 0, 0, //1
			1, 0, 0, //2
			0, 0, 0, //3
			1, 0, 0, //4
			1, 0, 0, //5
			0, 0, 0, //6
			0, 0, 0, //7
			0, 0, 0, //8
			0, 0, 0, //9
			0, 0, 0, //10
			0, 0, 0, //11
			0, 0, 0, //12
			0, 0, 1, //13
			0, 0, 1, //14
			0, 0, 0, //15
			0, 1, 1, //16
			0, 1, 1, //17
			0, 0, 1, //18
			0, 0, 1, //19
			0, 0, 0, //20
			0, 0, 0, //21
			0, 0, 1, //22
			0, 0, 1  //23
		};

		std::vector<float> bend_bias = {
			0, 0, 0, //0 
			0.75f, 0, 0, //1
			0.75f, 0, 0, //2
			0, 0, 0, //3
			2.5f, 0, 0, //4
			2.5f, 0, 0, //5
			0, 0, 0, //6
			0, 0, 0, //7
			0, 0, 0, //8
			0, 0, 0, //9
			0, 0, 0, //10
			0, 0, 0, //11
			0, 0, 0, //12
			0, 0, 0.75f, //13
			0, 0, 0.37f, //14
			0, 0, 0, //15
			0, 0.37f, 1.f, //16
			0, 2.5f, 1.5f, //17
			0, 0, 2.5f, //18
			0, 0, 0.f, //19
			0, 0, 0, //20
			0, 0, 0, //21
			0, 0, 0.37f, //22
			0, 0, 1.5f  //23
		};

		for (uint iteration = 0; iteration < iterations_; iteration++)
		{
			Eigen::MatrixXf jacobian = Eigen::MatrixXf::Zero(residuals, unknowns);
			Eigen::VectorXf error = Eigen::VectorXf::Zero(residuals);
			Eigen::VectorXf delta = Eigen::VectorXf::Zero(unknowns);

			Body body = generate_(betas, thetas);
			RegressedJoints coco_joints = coco_regress_(body.vertices);
			RegressedJoints smpl_joints = smpl_regress_(body.vertices);

			std::vector<Eigen::Vector3f> transformed_joint(COCO_JOINT_COUNT, Eigen::Vector3f(0.f, 0.f, 0.f));

			// Evaluate error
			for (uint m = 0; m < COCO_JOINT_COUNT; m++)
			{
				transformed_joint[m] = coco_joints.col(m) + translation;
				Eigen::Vector2f projection = project_(transformed_joint[m]);
				error(2 * m) = projection(0) - tracked_joints_[2 * m];
				error(2 * m + 1) = projection(1) - tracked_joints_[2 * m + 1];
			}
			for (uint j = 0; j < BETA_COUNT; j++)
			{
				error(COCO_JOINT_COUNT * 2 + j) = shape_prior_weight * betas[j];
			}
			for (uint k = 0; k < JOINT_COUNT; k++)
			{
				error(COCO_JOINT_COUNT * 2 + BETA_COUNT + ALPHA(k)) = pose_prior_weight * pose_prior[ALPHA(k)] * thetas(ALPHA(k));
				error(COCO_JOINT_COUNT * 2 + BETA_COUNT + BETA(k)) = pose_prior_weight * pose_prior[BETA(k)] * thetas(BETA(k));
				error(COCO_JOINT_COUNT * 2 + BETA_COUNT + GAMMA(k)) = pose_prior_weight * pose_prior[GAMMA(k)] * thetas(GAMMA(k));

				error(COCO_JOINT_COUNT * 2 + BETA_COUNT + THETA_COUNT * 3 + ALPHA(k)) = bend_prior_weight * bend_mask[ALPHA(k)] * exp(thetas(ALPHA(k)) - bend_bias[ALPHA(k)]);
				error(COCO_JOINT_COUNT * 2 + BETA_COUNT + THETA_COUNT * 3 + BETA(k)) = bend_prior_weight * bend_mask[BETA(k)] * exp(thetas(BETA(k)) - bend_bias[BETA(k)]);
				error(COCO_JOINT_COUNT * 2 + BETA_COUNT + THETA_COUNT * 3 + GAMMA(k)) = bend_prior_weight * bend_mask[GAMMA(k)] * exp(thetas(GAMMA(k)) - bend_bias[GAMMA(k)]);
			}

			// Evaluate jacobian
			Eigen::MatrixXf dbetas = Eigen::MatrixXf::Zero(residuals, BETA_COUNT);
			for (uint m = 0; m < COCO_JOINT_COUNT; m++)
			{
				for (uint j = 0; j < BETA_COUNT; j++) // over columns
				{
					dbetas.block<2, 1>(2 * m, j) = project_.Jacobian(transformed_joint[m]).transpose() * dshape_coco_[m*BETA_COUNT + j];
				}
			}
			for (uint j = 0; j < BETA_COUNT; j++)
			{
				dbetas(COCO_JOINT_COUNT * 2 + j, j) = shape_prior_weight;
			}

			Eigen::MatrixXf dthetas = Eigen::MatrixXf::Zero(residuals, THETA_COUNT * 3);
			{
				const std::vector<Skin>& skins = generate_.GetSkins();
				Eigen::Matrix4f* dskinning = new Eigen::Matrix4f[JOINT_COUNT * JOINT_COUNT * 3];
				ComputeSkinningDerivatives(thetas, smpl_joints, dskinning);
				for (uint m = 0; m < COCO_JOINT_COUNT; m++)
				{
					Eigen::VectorXf regressor_m = coco_regress_.JointRegressor(m);

					for (uint k = 0; k < JOINT_COUNT; k++) // Update dthetas
					{
						Eigen::Vector3f djoint_alpha(0, 0, 0);
						Eigen::Vector3f djoint_beta(0, 0, 0);
						Eigen::Vector3f djoint_gamma(0, 0, 0);

						for (uint i = 0; i < VERTEX_COUNT; i++)
						{
							if (regressor_m(i) > 0.001f)
							{
								Eigen::Vector4f temp;

								temp =
									(skins[i].weight.x * dskinning[ALPHA(k) * THETA_COUNT + skins[i].joint_index.x] +
										skins[i].weight.y * dskinning[ALPHA(k) * THETA_COUNT + skins[i].joint_index.y] +
										skins[i].weight.z * dskinning[ALPHA(k) * THETA_COUNT + skins[i].joint_index.z] +
										skins[i].weight.w * dskinning[ALPHA(k) * THETA_COUNT + skins[i].joint_index.w]) *
									body.deformed_template[i].ToEigen().homogeneous();

								djoint_alpha(0) += regressor_m(i) * temp(0);
								djoint_alpha(1) += regressor_m(i) * temp(1);
								djoint_alpha(2) += regressor_m(i) * temp(2);

								temp =
									(skins[i].weight.x * dskinning[BETA(k) * THETA_COUNT + skins[i].joint_index.x] +
										skins[i].weight.y * dskinning[BETA(k) * THETA_COUNT + skins[i].joint_index.y] +
										skins[i].weight.z * dskinning[BETA(k) * THETA_COUNT + skins[i].joint_index.z] +
										skins[i].weight.w * dskinning[BETA(k) * THETA_COUNT + skins[i].joint_index.w]) *
									body.deformed_template[i].ToEigen().homogeneous();

								djoint_beta(0) += regressor_m(i) * temp(0);
								djoint_beta(1) += regressor_m(i) * temp(1);
								djoint_beta(2) += regressor_m(i) * temp(2);

								temp =
									(skins[i].weight.x * dskinning[GAMMA(k) * THETA_COUNT + skins[i].joint_index.x] +
										skins[i].weight.y * dskinning[GAMMA(k) * THETA_COUNT + skins[i].joint_index.y] +
										skins[i].weight.z * dskinning[GAMMA(k) * THETA_COUNT + skins[i].joint_index.z] +
										skins[i].weight.w * dskinning[GAMMA(k) * THETA_COUNT + skins[i].joint_index.w]) *
									body.deformed_template[i].ToEigen().homogeneous();

								djoint_gamma(0) += regressor_m(i) * temp(0);
								djoint_gamma(1) += regressor_m(i) * temp(1);
								djoint_gamma(2) += regressor_m(i) * temp(2);
							}
						}

						dthetas.block<2, 1>(2 * m, ALPHA(k)) += project_.Jacobian(transformed_joint[m]).transpose() * djoint_alpha;
						dthetas.block<2, 1>(2 * m, BETA(k)) += project_.Jacobian(transformed_joint[m]).transpose() * djoint_beta;
						dthetas.block<2, 1>(2 * m, GAMMA(k)) += project_.Jacobian(transformed_joint[m]).transpose() * djoint_gamma;
					}
				}
				delete dskinning;

				for (uint k = 0; k < JOINT_COUNT; k++)
				{
					dthetas(COCO_JOINT_COUNT * 2 + BETA_COUNT + ALPHA(k), ALPHA(k)) = pose_prior_weight * pose_prior[ALPHA(k)];
					dthetas(COCO_JOINT_COUNT * 2 + BETA_COUNT + BETA(k), BETA(k)) = pose_prior_weight * pose_prior[BETA(k)];
					dthetas(COCO_JOINT_COUNT * 2 + BETA_COUNT + GAMMA(k), GAMMA(k)) = pose_prior_weight * pose_prior[GAMMA(k)];

					dthetas(COCO_JOINT_COUNT * 2 + BETA_COUNT + THETA_COUNT * 3 + ALPHA(k), ALPHA(k)) = bend_prior_weight * bend_mask[ALPHA(k)] * exp(thetas(ALPHA(k)) - bend_bias[ALPHA(k)]);
					dthetas(COCO_JOINT_COUNT * 2 + BETA_COUNT + THETA_COUNT * 3 + BETA(k), BETA(k)) = bend_prior_weight * bend_mask[BETA(k)] * exp(thetas(BETA(k)) - bend_bias[BETA(k)]);
					dthetas(COCO_JOINT_COUNT * 2 + BETA_COUNT + THETA_COUNT * 3 + GAMMA(k), GAMMA(k)) = bend_prior_weight * bend_mask[GAMMA(k)] * exp(thetas(GAMMA(k)) - bend_bias[GAMMA(k)]);
				}
			}

			jacobian << dbetas, dthetas;

			std::cout << "Iteration: " << iteration + count << std::endl;
			std::cout << "Error: " << error.squaredNorm() << std::endl;
			std::cout << "Translation:" << std::endl << translation(0) << " " << translation(1) << " " << translation(2) << std::endl;
			std::cout << "Betas:" << std::endl << betas << std::endl;
			std::cout << "Thetas:" << std::endl << thetas << std::endl;

			Eigen::MatrixXf Jt = jacobian.transpose();
			Eigen::MatrixXf JtJ = Jt * jacobian;
			Eigen::VectorXf JtF = Jt * error;
			Eigen::MatrixXf JtJ_diag = JtJ.diagonal().asDiagonal();

			float current_residual_error = error.squaredNorm();
			if (last_residual_error == MINF || (last_residual_error >= current_residual_error && iteration + 1 != iterations_))
			{
				JtJ += lambda * JtJ_diag;

				Eigen::ConjugateGradient<Eigen::MatrixXf, Eigen::Lower | Eigen::Upper> cg;
				cg.compute(JtJ);
				delta = cg.solve(JtF);

				delta_old = delta;
				last_residual_error = current_residual_error;

				for (uint j = 0; j < BETA_COUNT; j++)
					betas[j] -= delta(j);
				for (uint k = 0; k < JOINT_COUNT; k++)
				{
					thetas(ALPHA(k)) -= delta(BETA_COUNT + ALPHA(k));
					thetas(BETA(k)) -= delta(BETA_COUNT + BETA(k));
					thetas(GAMMA(k)) -= delta(BETA_COUNT + GAMMA(k));
				}

				lambda *= beta;
				if (lambda < lambda_min) lambda = lambda_min;

				
				//Log(image_filename, body, coco_joints, smpl_joints, translation, 0, iteration + count);
			}
			else if (last_residual_error < current_residual_error)
			{
				for (uint j = 0; j < BETA_COUNT; j++)
					betas[j] += delta_old(j);
				for (uint k = 0; k < JOINT_COUNT; k++)
				{
					thetas(ALPHA(k)) += delta_old(BETA_COUNT + ALPHA(k));
					thetas(BETA(k)) += delta_old(BETA_COUNT + BETA(k));
					thetas(GAMMA(k)) += delta_old(BETA_COUNT + GAMMA(k));
				}

				lambda *= alpha;
			}

			last_residual_error = current_residual_error;
		}

		//count += iterations_;
		return last_residual_error;
	}

	void Optimizer::ReconstructFine(const std::string& image_filename, Eigen::Vector3f& translation,
		ShapeCoefficients& betas, PoseEulerCoefficients& thetas, uint& count)
	{
		try
		{
			iterations_ = (uint)optimization_parameters.at("lm_reconstruction_iterations");
		}
		catch (std::out_of_range&)
		{
			MessageBoxA(NULL, "Did not load optimization configuration.", "Error", MB_OK);
		}

		const int residuals = COCO_JOINT_COUNT * 2 + BETA_COUNT + THETA_COUNT * 3; // projection error, betas, thetas
		const int unknowns = BETA_COUNT + THETA_COUNT * 3; // betas, thetas
		const float shape_prior_weight = 10.f;
		const float pose_prior_weight = 10.f;
		float lambda = 2.0f;
		float lambda_min = 0.1f;
		float alpha = 1.8f;
		float beta = 0.6f;
		const float MINF = -100000.f;
		float last_residual_error;

		Eigen::VectorXf delta_old = Eigen::VectorXf::Zero(unknowns);

		for (uint level = 0; level < checked_joint_sets_coco_.size(); level++)
		{
			last_residual_error = MINF;

			for (uint iteration = 0; iteration < iterations_; iteration++)
			{
				Eigen::MatrixXf jacobian = Eigen::MatrixXf::Zero(residuals, unknowns);
				Eigen::VectorXf error = Eigen::VectorXf::Zero(residuals);
				Eigen::VectorXf delta = Eigen::VectorXf::Zero(unknowns);

				Body body = generate_(betas, thetas);
				RegressedJoints coco_joints = coco_regress_(body.vertices);
				RegressedJoints smpl_joints = smpl_regress_(body.vertices);

				std::vector<Eigen::Vector3f> transformed_joint(COCO_JOINT_COUNT, Eigen::Vector3f(0.f, 0.f, 0.f));

				// Evaluate error
				for (auto& m : checked_joint_sets_coco_[level])
				{
					transformed_joint[m] = coco_joints.col(m) + translation;
					Eigen::Vector2f projection = project_(transformed_joint[m]);
					error(2 * m) = projection(0) - tracked_joints_[2 * m];
					error(2 * m + 1) = projection(1) - tracked_joints_[2 * m + 1];
				}
				for (uint j = 0; j < BETA_COUNT; j++)
				{
					error(COCO_JOINT_COUNT * 2 + j) = shape_prior_weight * betas[j];
				}
				for (auto& m : movable_joint_sets_coco_[level])
				{
					error(COCO_JOINT_COUNT * 2 + BETA_COUNT + ALPHA(m)) = pose_prior_weight * thetas(ALPHA(m));
					error(COCO_JOINT_COUNT * 2 + BETA_COUNT + BETA(m)) = pose_prior_weight * thetas(BETA(m));
					error(COCO_JOINT_COUNT * 2 + BETA_COUNT + GAMMA(m)) = pose_prior_weight * thetas(GAMMA(m));
				}

				// Evaluate jacobian
				Eigen::MatrixXf dbetas = Eigen::MatrixXf::Zero(residuals, BETA_COUNT);
				for (auto& m : checked_joint_sets_coco_[level])
				{
					for (uint j = 0; j < BETA_COUNT; j++) // over columns
					{
						dbetas.block<2, 1>(2 * m, j) = project_.Jacobian(transformed_joint[m]).transpose() * dshape_coco_[m*BETA_COUNT + j];
					}
				}
				for (uint j = 0; j < BETA_COUNT; j++)
				{
					dbetas(COCO_JOINT_COUNT * 2 + j, j) = shape_prior_weight;
				}

				Eigen::MatrixXf dthetas = Eigen::MatrixXf::Zero(residuals, THETA_COUNT * 3);
				{
					const std::vector<Skin>& skins = generate_.GetSkins();
					Eigen::Matrix4f* dskinning = new Eigen::Matrix4f[JOINT_COUNT * JOINT_COUNT * 3];
					ComputeSkinningDerivatives(thetas, smpl_joints, dskinning);
					for (auto& m : checked_joint_sets_coco_[level])
					{
						Eigen::VectorXf regressor_m = coco_regress_.JointRegressor(m);
						
						for (auto& k : movable_joint_sets_coco_[level]) // Update dthetas
						{
							Eigen::Vector3f djoint_alpha(0, 0, 0);
							Eigen::Vector3f djoint_beta(0, 0, 0);
							Eigen::Vector3f djoint_gamma(0, 0, 0);

							for (uint i = 0; i < VERTEX_COUNT; i++)
							{
								if (regressor_m(i) > 0.001f)
								{
									Eigen::Vector4f temp;

									temp =
										(skins[i].weight.x * dskinning[ALPHA(k) * THETA_COUNT + skins[i].joint_index.x] +
											skins[i].weight.y * dskinning[ALPHA(k) * THETA_COUNT + skins[i].joint_index.y] +
											skins[i].weight.z * dskinning[ALPHA(k) * THETA_COUNT + skins[i].joint_index.z] +
											skins[i].weight.w * dskinning[ALPHA(k) * THETA_COUNT + skins[i].joint_index.w]) *
										body.deformed_template[i].ToEigen().homogeneous();

									djoint_alpha(0) += regressor_m(i) * temp(0);
									djoint_alpha(1) += regressor_m(i) * temp(1);
									djoint_alpha(2) += regressor_m(i) * temp(2);

									temp =
										(skins[i].weight.x * dskinning[BETA(k) * THETA_COUNT + skins[i].joint_index.x] +
											skins[i].weight.y * dskinning[BETA(k) * THETA_COUNT + skins[i].joint_index.y] +
											skins[i].weight.z * dskinning[BETA(k) * THETA_COUNT + skins[i].joint_index.z] +
											skins[i].weight.w * dskinning[BETA(k) * THETA_COUNT + skins[i].joint_index.w]) *
										body.deformed_template[i].ToEigen().homogeneous();

									djoint_beta(0) += regressor_m(i) * temp(0);
									djoint_beta(1) += regressor_m(i) * temp(1);
									djoint_beta(2) += regressor_m(i) * temp(2);

									temp =
										(skins[i].weight.x * dskinning[GAMMA(k) * THETA_COUNT + skins[i].joint_index.x] +
											skins[i].weight.y * dskinning[GAMMA(k) * THETA_COUNT + skins[i].joint_index.y] +
											skins[i].weight.z * dskinning[GAMMA(k) * THETA_COUNT + skins[i].joint_index.z] +
											skins[i].weight.w * dskinning[GAMMA(k) * THETA_COUNT + skins[i].joint_index.w]) *
										body.deformed_template[i].ToEigen().homogeneous();

									djoint_gamma(0) += regressor_m(i) * temp(0);
									djoint_gamma(1) += regressor_m(i) * temp(1);
									djoint_gamma(2) += regressor_m(i) * temp(2);
								}
							}

							dthetas.block<2, 1>(2 * m, ALPHA(k)) += project_.Jacobian(transformed_joint[m]).transpose() * djoint_alpha;
							dthetas.block<2, 1>(2 * m, BETA(k)) += project_.Jacobian(transformed_joint[m]).transpose() * djoint_beta;
							dthetas.block<2, 1>(2 * m, GAMMA(k)) += project_.Jacobian(transformed_joint[m]).transpose() * djoint_gamma;
						}
					}
					delete dskinning;

					for (auto& k : movable_joint_sets_coco_[level])
					{
						dthetas(COCO_JOINT_COUNT * 2 + BETA_COUNT + ALPHA(k), ALPHA(k)) = pose_prior_weight;
						dthetas(COCO_JOINT_COUNT * 2 + BETA_COUNT + BETA(k), BETA(k)) = pose_prior_weight;
						dthetas(COCO_JOINT_COUNT * 2 + BETA_COUNT + GAMMA(k), GAMMA(k)) = pose_prior_weight;
					}
				}


				jacobian << dbetas, dthetas;

				Log(image_filename, body, coco_joints, smpl_joints, translation, level, (level+1)*iterations_ + iteration + count);
				std::cout << "Iteration: " << (level+1) * iterations_ + iteration + count << std::endl;
				std::cout << "Error: " << error.squaredNorm() << std::endl;
				std::cout << "Translation:" << std::endl << translation(0) << " " << translation(1) << " " << translation(2) << std::endl;
				std::cout << "Betas:" << std::endl << betas << std::endl;
				std::cout << "Thetas:" << std::endl << thetas << std::endl;

				Eigen::MatrixXf Jt = jacobian.transpose();
				Eigen::MatrixXf JtJ = Jt * jacobian;
				Eigen::VectorXf JtF = Jt * error;
				Eigen::MatrixXf JtJ_diag = JtJ.diagonal().asDiagonal();

				float current_residual_error = error.squaredNorm();
				if (last_residual_error == MINF || (last_residual_error >= current_residual_error && iteration + 1 != iterations_))
				{
					JtJ += lambda * JtJ_diag;

					Eigen::ConjugateGradient<Eigen::MatrixXf, Eigen::Lower | Eigen::Upper> cg;
					cg.compute(JtJ);
					delta = cg.solve(JtF);

					delta_old = delta;
					last_residual_error = current_residual_error;

					for (uint j = 0; j < BETA_COUNT; j++)
						betas[j] -= delta(j);
					for (auto& k : movable_joint_sets_coco_[level])
					{
						thetas(ALPHA(k)) -= delta(BETA_COUNT + ALPHA(k));
						thetas(BETA(k)) -= delta(BETA_COUNT + BETA(k));
						thetas(GAMMA(k)) -= delta(BETA_COUNT + GAMMA(k));
					}

					lambda *= beta;
					if (lambda < lambda_min) lambda = lambda_min;
				}
				else if (last_residual_error < current_residual_error)
				{
					for (uint j = 0; j < BETA_COUNT; j++)
						betas[j] += delta_old(j);
					for (auto& k : movable_joint_sets_coco_[level])
					{
						thetas(ALPHA(k)) += delta_old(BETA_COUNT + ALPHA(k));
						thetas(BETA(k)) += delta_old(BETA_COUNT + BETA(k));
						thetas(GAMMA(k)) += delta_old(BETA_COUNT + GAMMA(k));
					}

					lambda *= alpha;
				}

				last_residual_error = current_residual_error;
			}
		}

		count += static_cast<uint>(checked_joint_sets_coco_.size()) * iterations_;
	}

	void Optimizer::Reconstruct(const std::string& image_filename, Eigen::Vector3f& translation,
		ShapeCoefficients& betas, PoseEulerCoefficients& thetas, uint& count)
	{
		// fixed pitch, reconstruct roll, yaw up to a sign, translation and beta0
		ReconstructCamera(image_filename, translation, betas, thetas, count);

		Eigen::Vector3f translation0(translation), translation1(translation);
		ShapeCoefficients shape0(betas), shape1(betas);
		PoseEulerCoefficients pose0(thetas), pose1(thetas);
		pose1(1) *= -1;

		std::cout << std::endl << std::endl;

		float error0 = ReconstructTotal(image_filename, translation0, shape0, pose0, count);
		std::cout << std::endl << std::endl;
		float error1 = ReconstructTotal(image_filename, translation1, shape1, pose1, count);
		std::cout << std::endl << std::endl;

		Body body;
		if (error0 < error1)
		{
			body = generate_(shape0, pose0);
			translation = translation0;
		}
		else
		{
			body = generate_(shape1, pose1);
			translation = translation1;
		}

		Log(image_filename, body, coco_regress_(body.vertices), smpl_regress_(body.vertices), translation, 0, count);
		body.Dump(std::string("Reconstruction3D/").append(std::to_string(count++)).append(".obj"));
	}

	void Optimizer::ComputeShapeDerivativesAllJointTypes()
	{
		ComputeShapeDerivatives(JOINT_TYPE::COCO, dshape_coco_);
		ComputeShapeDerivatives(JOINT_TYPE::SMPL, dshape_smpl_);
	}

	void Optimizer::ComputeShapeDerivatives(const JOINT_TYPE& joint_type, 
		std::vector<Eigen::Vector3f>& dshape)
	{
		uint joint_count = (joint_type == JOINT_TYPE::COCO ? COCO_JOINT_COUNT : JOINT_COUNT);
		dshape.resize(joint_count * BETA_COUNT);
		{
			const std::vector<float3>& shapedirs = generate_.GetShapeDirs();
			for (uint m = 0; m < joint_count; m++)
			{
				Eigen::VectorXf regressor_m = (joint_type == JOINT_TYPE::COCO ? coco_regress_.JointRegressor(m) : smpl_regress_.JointRegressor(m));
				for (uint j = 0; j < BETA_COUNT; j++)
				{
					Eigen::Vector3f temp(0, 0, 0);
					for (uint i = 0; i < VERTEX_COUNT; i++)
					{
						temp(0) += regressor_m(i) * shapedirs[i*BETA_COUNT + j].x;
						temp(1) += regressor_m(i) * shapedirs[i*BETA_COUNT + j].y;
						temp(2) += regressor_m(i) * shapedirs[i*BETA_COUNT + j].z;
					}
					dshape[m*BETA_COUNT + j] = temp;
				}
			}
		}
	}

	void Optimizer::ComputeSkinning(const PoseEulerCoefficients& thetas, const RegressedJoints& smpl_joints,
		Eigen::Matrix4f(&palette)[JOINT_COUNT]) const
	{
		palette[0] = EulerSkinningXYZ(thetas[0].x, thetas[0].y, thetas[0].z, smpl_joints.col(0)(0), smpl_joints.col(0)(1), smpl_joints.col(0)(2));
		for (uint i = 1; i < JOINT_COUNT; i++)
		{
			palette[i] = palette[PARENT_INDEX[i]] * EulerSkinningXYZ(thetas[i].x, thetas[i].y, thetas[i].z, smpl_joints.col(i)(0), smpl_joints.col(i)(1), smpl_joints.col(i)(2));
		}
	}

	void Optimizer::ComputeSkinningLastDerivatives(const PoseEulerCoefficients& thetas,
		const RegressedJoints& smpl_joints,
		Eigen::Matrix4f(&palette)[JOINT_COUNT],
		Eigen::Matrix4f(&dskinning)[JOINT_COUNT * 3]) const
	{
		// parent initialization
		{
			palette[0] = EulerSkinningXYZ(thetas[0].x, thetas[0].y, thetas[0].z, smpl_joints.col(0)(0), smpl_joints.col(0)(1), smpl_joints.col(0)(2));
			dskinning[0] = EulerSkinningXYZDerivativeToAlpha(thetas[0].x, thetas[0].y, thetas[0].z, smpl_joints.col(0)(0), smpl_joints.col(0)(1), smpl_joints.col(0)(2));
			dskinning[1] = EulerSkinningXYZDerivativeToBeta(thetas[0].x, thetas[0].y, thetas[0].z, smpl_joints.col(0)(0), smpl_joints.col(0)(1), smpl_joints.col(0)(2));
			dskinning[2] = EulerSkinningXYZDerivativeToGamma(thetas[0].x, thetas[0].y, thetas[0].z, smpl_joints.col(0)(0), smpl_joints.col(0)(1), smpl_joints.col(0)(2));
		}

		for (uint i = 1; i < JOINT_COUNT; i++)
		{
			palette[i] = palette[PARENT_INDEX[i]] * EulerSkinningXYZ(thetas[i].x, thetas[i].y, thetas[i].z, smpl_joints.col(i)(0), smpl_joints.col(i)(1), smpl_joints.col(i)(2));
			dskinning[i * 3 + 0] = palette[PARENT_INDEX[i]]
				* EulerSkinningXYZDerivativeToAlpha(thetas[i].x, thetas[i].y, thetas[i].z, smpl_joints.col(i)(0), smpl_joints.col(i)(1), smpl_joints.col(i)(2));
			dskinning[i * 3 + 1] = palette[PARENT_INDEX[i]]
				* EulerSkinningXYZDerivativeToBeta(thetas[i].x, thetas[i].y, thetas[i].z, smpl_joints.col(i)(0), smpl_joints.col(i)(1), smpl_joints.col(i)(2));
			dskinning[i * 3 + 2] = palette[PARENT_INDEX[i]]
				* EulerSkinningXYZDerivativeToGamma(thetas[i].x, thetas[i].y, thetas[i].z, smpl_joints.col(i)(0), smpl_joints.col(i)(1), smpl_joints.col(i)(2));
		}
	}

	void Optimizer::ComputeSkinningDerivatives(const PoseEulerCoefficients& thetas, const RegressedJoints& smpl_joints,
		Eigen::Matrix4f* dskinning) const
	{
		Eigen::Matrix4f palette[JOINT_COUNT];
		ComputeSkinningDerivatives(thetas, smpl_joints, palette, dskinning);
	}

	void Optimizer::ComputeSkinningDerivatives(const PoseEulerCoefficients& thetas,
		const RegressedJoints& joints,
		Eigen::Matrix4f(&palette)[JOINT_COUNT],
		Eigen::Matrix4f* dskinning) const
	{
		// palette initialization
		Eigen::Matrix4f skinning[JOINT_COUNT];
		skinning[0] = EulerSkinningXYZ(thetas[0].x, thetas[0].y, thetas[0].z, joints.col(0)(0), joints.col(0)(1), joints.col(0)(2));
		palette[0] = skinning[0];
		for (uint i = 1; i < JOINT_COUNT; i++)
		{
			skinning[i] = EulerSkinningXYZ(thetas[i].x, thetas[i].y, thetas[i].z, joints.col(i)(0), joints.col(i)(1), joints.col(i)(2));
			palette[i] = palette[PARENT_INDEX[i]] * skinning[i];
		}

		/*

		dskinning

		body_parts ------->
		theta0.x
		theta0.y
		theta0.z
		theta1.x
		...

		dskinning[angle_component * joint_count + body_part] - what is the derivative of skinning in this body part wrt this angle component

		*/

		ZeroMemory(dskinning, sizeof(Eigen::Matrix4f) * JOINT_COUNT * JOINT_COUNT * 3);

		// main diagonal initialization
		uint idx;
		idx = ALPHA(0)*JOINT_COUNT;
		dskinning[ALPHA(0)*JOINT_COUNT] = EulerSkinningXYZDerivativeToAlpha(thetas[0].x, thetas[0].y, thetas[0].z, joints.col(0)(0), joints.col(0)(1), joints.col(0)(2));
		idx = BETA(0)*JOINT_COUNT;
		dskinning[BETA(0)*JOINT_COUNT] = EulerSkinningXYZDerivativeToBeta(thetas[0].x, thetas[0].y, thetas[0].z, joints.col(0)(0), joints.col(0)(1), joints.col(0)(2));
		idx = GAMMA(0)*JOINT_COUNT;
		dskinning[GAMMA(0)*JOINT_COUNT] = EulerSkinningXYZDerivativeToGamma(thetas[0].x, thetas[0].y, thetas[0].z, joints.col(0)(0), joints.col(0)(1), joints.col(0)(2));

		for (uint i = 1; i < JOINT_COUNT; i++) // body parts with angles controlling them
		{
			idx = ALPHA(i)*JOINT_COUNT + i;
			dskinning[ALPHA(i)*JOINT_COUNT + i] = palette[PARENT_INDEX[i]] *
				EulerSkinningXYZDerivativeToAlpha(thetas[i].x, thetas[i].y, thetas[i].z, joints.col(i)(0), joints.col(i)(1), joints.col(i)(2));
			idx = BETA(i)*JOINT_COUNT + i;
			dskinning[BETA(i)*JOINT_COUNT + i] = palette[PARENT_INDEX[i]] *
				EulerSkinningXYZDerivativeToBeta(thetas[i].x, thetas[i].y, thetas[i].z, joints.col(i)(0), joints.col(i)(1), joints.col(i)(2));
			idx = GAMMA(i)*JOINT_COUNT + i;
			dskinning[GAMMA(i)*JOINT_COUNT + i] = palette[PARENT_INDEX[i]] *
				EulerSkinningXYZDerivativeToGamma(thetas[i].x, thetas[i].y, thetas[i].z, joints.col(i)(0), joints.col(i)(1), joints.col(i)(2));
		}

		for (uint i = 0; i < JOINT_COUNT; i++) // thetas
		{
			for (uint j = i + 1; j < JOINT_COUNT; j++) // body parts
			{
				idx = ALPHA(i) * JOINT_COUNT + j;
				dskinning[ALPHA(i) * JOINT_COUNT + j] = dskinning[ALPHA(i) * JOINT_COUNT + PARENT_INDEX[j]] * skinning[j];
				idx = BETA(i) * JOINT_COUNT + j;
				dskinning[BETA(i) * JOINT_COUNT + j] = dskinning[BETA(i) * JOINT_COUNT + PARENT_INDEX[j]] * skinning[j];
				idx = GAMMA(i) * JOINT_COUNT + j;
				dskinning[GAMMA(i) * JOINT_COUNT + j] = dskinning[GAMMA(i) * JOINT_COUNT + PARENT_INDEX[j]] * skinning[j];
			}
		}
	}

	void Optimizer::ComputeJacobianAndError(Eigen::MatrixXf& jacobian, Eigen::VectorXf& error, 
		const Body& body, const RegressedJoints& coco_joints, const RegressedJoints& smpl_joints,
		const Eigen::Vector3f& translation, const ShapeCoefficients& betas,
		const PoseEulerCoefficients& thetas, int active_set, 
		float shape_prior_weight, float pose_prior_weight)
	{
		std::vector<Eigen::Vector3f> transformed_joint(COCO_JOINT_COUNT, Eigen::Vector3f(0.f,0.f,0.f));

		// Evaluate error
		for (auto& m : checked_joint_sets_coco_[active_set])
		{
			transformed_joint[m] = coco_joints.col(m) + translation;
			Eigen::Vector2f projection = project_(transformed_joint[m]);
			error(2*m) = projection(0) - tracked_joints_[2*m];
			error(2*m+1) = projection(1) - tracked_joints_[2*m+1];
		}
		for (uint j = 0; j < BETA_COUNT; j++)
		{
			error(COCO_JOINT_COUNT * 2 + j) = shape_prior_weight * betas[j] * betas[j];
		}
		for (auto& m : movable_joint_sets_coco_[active_set])
		{
			error(COCO_JOINT_COUNT * 2 + BETA_COUNT + ALPHA(m)) = pose_prior_weight * thetas(ALPHA(m)) * thetas(ALPHA(m));
			error(COCO_JOINT_COUNT * 2 + BETA_COUNT + BETA(m)) = pose_prior_weight * thetas(BETA(m)) * thetas(BETA(m));
			error(COCO_JOINT_COUNT * 2 + BETA_COUNT + GAMMA(m)) = pose_prior_weight * thetas(GAMMA(m)) * thetas(GAMMA(m));
		}

		// Evaluate Jacobian
		Eigen::MatrixXf dtranslation = Eigen::MatrixXf::Zero(RESIDUALS, 3);
		for (auto& m : checked_joint_sets_coco_[active_set])
		{
			dtranslation.block<2, 3>(2*m,0) = project_.Jacobian(transformed_joint[m]).transpose();
		}

		Eigen::MatrixXf dbetas = Eigen::MatrixXf::Zero(RESIDUALS, BETA_COUNT);
		for (auto& m : checked_joint_sets_coco_[active_set]) // over rows
		{
			for (uint j = 0; j < BETA_COUNT; j++) // over columns
			{
				dbetas.block<2,1>(2*m,j) = project_.Jacobian(transformed_joint[m]).transpose() * dshape_coco_[m*BETA_COUNT + j];
			}
		}
		for (uint j = 0; j < BETA_COUNT; j++)
		{
			dbetas(COCO_JOINT_COUNT * 2 + j, j) = shape_prior_weight * 2.0f * betas[j];
		}

		Eigen::MatrixXf dthetas = Eigen::MatrixXf::Zero(RESIDUALS, THETA_COUNT * 3);
		const std::vector<Skin>& skins = generate_.GetSkins();
		Eigen::Matrix4f* dskinning = new Eigen::Matrix4f[JOINT_COUNT * JOINT_COUNT * 3];
		ComputeSkinningDerivatives(thetas, smpl_joints, dskinning);
		for (auto& m : checked_joint_sets_coco_[active_set])
		{
			Eigen::VectorXf regressor_m = coco_regress_.JointRegressor(m);

			Eigen::Vector3f djoint_alpha(0, 0, 0);
			Eigen::Vector3f djoint_beta(0, 0, 0);
			Eigen::Vector3f djoint_gamma(0, 0, 0);

			for (auto& k : movable_joint_sets_coco_[active_set]) // Update dthetas
			{
#pragma omp parallel for
				for (int i = 0; i < VERTEX_COUNT; i++)
				{
					if (regressor_m(i) > 0.001f)
					{
						Eigen::Vector4f temp;

						temp =
							(skins[i].weight.x * dskinning[ALPHA(k) * THETA_COUNT + skins[i].joint_index.x] +
								skins[i].weight.y * dskinning[ALPHA(k) * THETA_COUNT + skins[i].joint_index.y] +
								skins[i].weight.z * dskinning[ALPHA(k) * THETA_COUNT + skins[i].joint_index.z] +
								skins[i].weight.w * dskinning[ALPHA(k) * THETA_COUNT + skins[i].joint_index.w]) *
							body.deformed_template[i].ToEigen().homogeneous();

						djoint_alpha(0) = regressor_m(i) * temp(0);
						djoint_alpha(1) = regressor_m(i) * temp(1);
						djoint_alpha(2) = regressor_m(i) * temp(2);

						temp =
							(skins[i].weight.x * dskinning[BETA(k) * THETA_COUNT + skins[i].joint_index.x] +
								skins[i].weight.y * dskinning[BETA(k) * THETA_COUNT + skins[i].joint_index.y] +
								skins[i].weight.z * dskinning[BETA(k) * THETA_COUNT + skins[i].joint_index.z] +
								skins[i].weight.w * dskinning[BETA(k) * THETA_COUNT + skins[i].joint_index.w]) *
							body.deformed_template[i].ToEigen().homogeneous();

						djoint_beta(0) = regressor_m(i) * temp(0);
						djoint_beta(1) = regressor_m(i) * temp(1);
						djoint_beta(2) = regressor_m(i) * temp(2);

						temp =
							(skins[i].weight.x * dskinning[GAMMA(k) * THETA_COUNT + skins[i].joint_index.x] +
								skins[i].weight.y * dskinning[GAMMA(k) * THETA_COUNT + skins[i].joint_index.y] +
								skins[i].weight.z * dskinning[GAMMA(k) * THETA_COUNT + skins[i].joint_index.z] +
								skins[i].weight.w * dskinning[GAMMA(k) * THETA_COUNT + skins[i].joint_index.w]) *
							body.deformed_template[i].ToEigen().homogeneous();

						djoint_gamma(0) = regressor_m(i) * temp(0);
						djoint_gamma(1) = regressor_m(i) * temp(1);
						djoint_gamma(2) = regressor_m(i) * temp(2);
					}
				}

				dthetas.block<2,1>(m, ALPHA(k)) += project_.Jacobian(transformed_joint[m]).transpose() * djoint_alpha;
				dthetas.block<2,1>(m, BETA(k)) += project_.Jacobian(transformed_joint[m]).transpose() * djoint_beta;
				dthetas.block<2,1>(m, GAMMA(k)) += project_.Jacobian(transformed_joint[m]).transpose() * djoint_gamma;
			}
		}
		
		delete dskinning;

		for (auto& k : movable_joint_sets_coco_[active_set])
		{
			dthetas(COCO_JOINT_COUNT * 2 + BETA_COUNT + ALPHA(k), ALPHA(k)) = pose_prior_weight * 2.0f * thetas(ALPHA(k));
			dthetas(COCO_JOINT_COUNT * 2 + BETA_COUNT + BETA(k), BETA(k)) = pose_prior_weight * 2.0f * thetas(BETA(k));
			dthetas(COCO_JOINT_COUNT * 2 + BETA_COUNT + GAMMA(k), GAMMA(k)) = pose_prior_weight * 2.0f * thetas(GAMMA(k));
		}

		{
			jacobian << dtranslation, dbetas, dthetas;
		}
	}

	RegressedJoints Optimizer::RegressJoints(const Body& body, const JOINT_TYPE& joint_type) const
	{
		switch (joint_type)
		{
		case SMPL:
			return smpl_regress_(body.vertices);
		case COCO:
			return coco_regress_(body.vertices);
		default:
			return RegressedJoints();
		}
	}

	void Optimizer::Log(const std::string& image_filename, const Body& body, 
		const RegressedJoints& reconstruction_joints, const RegressedJoints& smpl_joints,
		const Eigen::Vector3f& translation, int active_set, int count) const
	{
		Image image(image_filename.c_str());
		Image::Draw3D(image, WHITE, project_, translation, body.vertices);
		Image::Draw3D(image, BLUE, 2, project_, translation, Joints2Vector(reconstruction_joints));

		std::vector<float3> checked_joint_coordinates;
		checked_joint_coordinates.reserve(10);
		for (auto&m : checked_joint_sets_coco_[active_set])
		{
			checked_joint_coordinates.push_back(float3(reconstruction_joints.col(m)));
		}
		Image::Draw3D(image, RED, 2, project_, translation, checked_joint_coordinates);

		std::vector<float3> movable_joint_coordinates;
		movable_joint_coordinates.reserve(10);
		for (auto&m : movable_joint_sets_coco_[active_set])
		{
			movable_joint_coordinates.push_back(float3(smpl_joints.col(m)));
		}
		Image::Draw3D(image, GREEN, 2, project_, translation, movable_joint_coordinates);
		Image::Draw2D(image, YELLOW, 2, tracked_joints_);

		image.SavePNG(std::string("ReconstructionTemp/").append(std::to_string(count)).append(".png"));
	}

	void Optimizer::operator()(const std::string& image_filename, ShapeCoefficients& betas,
		PoseAxisAngleCoefficients& thetas, Eigen::Vector3f& translation)
	{
		Body body = generate_(betas, thetas);
		//body.Dump("new.obj");

		//OptimizeExtrinsics(image_filename, body, translation);

		// jake - tpose
		//translation = Eigen::Vector3f(-0.279263f, 0.366249f, - 4.01858f);

		// andrei - surrender
		translation = Eigen::Vector3f(0.0404971f, 0.416566f, -3.69404f);

		//OptimizeShapeFromJoints2D(image_filename, translation, thetas, betas);

		PoseEulerCoefficients eulers;

		// jake - tpose
		// betas = { -0.287861f, -3.47998f, 3.58785f, -1.76725f, 2.16237f, 0.757985f, -1.80657f, -0.411921f, 3.06527f, -0.830053f };

		// andrei - surrender
		betas = { 0.0631728f, 0.463201f, 1.29514f, -0.267553f, -0.250164f, 0.171496f, -0.185418f, 0.0712007f, 0.116954f, -0.326455f };

		//OptimizePose(image_filename, translation, betas, eulers);
	}
}