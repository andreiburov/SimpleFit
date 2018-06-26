#include "Optimizer.h"
#include "Image.h"

namespace smpl
{
	Optimizer::Optimizer(Configuration& configuration, const Generator& generate, const std::vector<float>& tracked_joints) :
		generate_(generate), project_(configuration.intrinsics),
		coco_regress_(configuration.coco_regressor, COCO_JOINT_COUNT), smpl_regress_(configuration.smpl_regressor, JOINT_COUNT),
		tracked_joints_(tracked_joints)
	{
		dshape_.resize(COCO_JOINT_COUNT * BETA_COUNT);
		{
			const std::vector<float3>& shapedirs = generate_.GetShapeDirs();
			for (uint m = 0; m < COCO_JOINT_COUNT; m++)
			{
				Eigen::VectorXf regressor_m = coco_regress_.GetRow(m);
				for (uint j = 0; j < BETA_COUNT; j++)
				{
					Eigen::Vector3f temp(0, 0, 0);
					for (uint i = 0; i < VERTEX_COUNT; i++)
					{
						temp(0) += regressor_m(i) * shapedirs[i*BETA_COUNT + j].x;
						temp(1) += regressor_m(i) * shapedirs[i*BETA_COUNT + j].y;
						temp(2) += regressor_m(i) * shapedirs[i*BETA_COUNT + j].z;
					}
					dshape_[m*BETA_COUNT + j] = temp;
				}
			}
		}
	}

	void Optimizer::OptimizeExtrinsics(const std::string& image_filename, const Body& body, Eigen::Vector3f& scaling, Eigen::Vector3f& translation)
	{
		float learning_rate = 1e-7f;
		float energy = 1000.f;
		float epsilon = 10;
		uint count = 0;

		for (uint iteration = 0; iteration < 1000; iteration++)
			/*while (energy > epsilon)*/
		{
			Image image(640, 480);
			//body.Draw(image, project_.GetIntrinsics(), scaling, translation);

			energy = 0.f;

			Eigen::Vector3f dscaling(0.f, 0.f, 0.f);
			Eigen::Vector3f dtranslation(0.f, 0.f, 0.f);

			Joints joints = coco_regress_(body.vertices);

			for (uint i = 0; i < COCO_JOINT_COUNT; i++)
			{
				Eigen::Vector3f joint = joints.col(i);
				Eigen::Vector3f transformed_joint = Eigen::Scaling(scaling) * joint + translation;
				Eigen::Vector2f projection = project_(transformed_joint);
				Eigen::Vector2f error = Eigen::Vector2f(projection(0) - tracked_joints_[2 * i], projection(1) - tracked_joints_[2 * i + 1]);

				energy += error.squaredNorm();
				//dscaling += Eigen::Scaling(joint) * project_.derivative(transformed_joint) * error * 2.f;
				dtranslation += project_.derivative(transformed_joint) * error * 2.f;
			}

			image.SavePNG("projection.png");

			//scaling -= learning_rate * dscaling;
			translation -= learning_rate * dtranslation;

			if (count % 100 == 0)
			{
				Image image(image_filename.c_str(), 640, 480);
				//body.Draw(image, project_.GetIntrinsics(), scaling, translation);
				image.SavePNG(std::string("projections/").append("0extrinsics_").append(std::to_string(count)).append(".png").c_str());
				std::cout << "Iteration: " << count << std::endl;
				std::cout << "Energy: " << energy << std::endl;
				std::cout << "Scaling: " << std::endl << scaling << std::endl;
				std::cout << "Translation:" << std::endl << translation << std::endl << std::endl;
			}
			count++;
		}
	}

	void Optimizer::OptimizeShape(const std::string& image_filename, const Eigen::Vector3f& scaling, const Eigen::Vector3f& translation, const PoseEulerCoefficients& thetas, ShapeCoefficients& betas)
	{
		float learning_rate = 1e-3f;
		float energy = 1000.f;
		float epsilon = 10;
		uint count = 0;

		for (uint iteration = 0; iteration < 1000; iteration++)
			/*while (energy > epsilon)*/
		{
			Body body = generate_(betas, thetas);

			energy = 0.f;
			float dbetas[BETA_COUNT] = { 0 };

			Joints joints = coco_regress_(body.vertices);

			for (uint m = 0; m < COCO_JOINT_COUNT; m++)
			{
				Eigen::Vector3f joint = joints.col(m);
				Eigen::Vector3f transformed_joint = Eigen::Scaling(scaling) * joint + translation;
				Eigen::Vector2f projection = project_(transformed_joint);
				Eigen::Vector2f error = Eigen::Vector2f(projection(0) - tracked_joints_[2 * m], projection(1) - tracked_joints_[2 * m + 1]);

				energy += error.squaredNorm();

				for (uint j = 0; j < BETA_COUNT; j++)
				{
					dbetas[j] += 2.f * error.transpose() * project_.derivative(transformed_joint).transpose() * Eigen::Scaling(scaling) * dshape_[m*BETA_COUNT + j];
				}
			}

			for (uint j = 0; j < BETA_COUNT; j++)
			{
				betas[j] -= learning_rate * dbetas[j];
			}

			if (count % 100 == 0)
			{
				Image image(640, 480);
				Image::Draw3D(image, project_.GetIntrinsics(), scaling, translation, WHITE, body.vertices);
				Image::Draw3D(image, project_.GetIntrinsics(), scaling, translation, BLUE, 2, Joints2Vector(joints));

				image.SavePNG(std::string("ShapeReconstructionTemp/").append(std::to_string(count)).append(".png"));

				//Image image(image_filename.c_str(), 640, 480);
				////body.Draw(image, project_.GetIntrinsics(), scaling, translation);
				//image.SavePNG(std::string("projections/").append("1shape_").append(std::to_string(count)).append(".png").c_str());
				
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

	void Optimizer::ComputeSkinning(const PoseEulerCoefficients& thetas, const Joints& smpl_joints,
		Eigen::Matrix4f(&palette)[JOINT_COUNT]) const
	{
		palette[0] = EulerSkinningXYZ(thetas[0].x, thetas[0].y, thetas[0].z, smpl_joints.col(0)(0), smpl_joints.col(0)(1), smpl_joints.col(0)(2));
		for (uint i = 1; i < JOINT_COUNT; i++)
		{
			palette[i] = palette[PARENT_INDEX[i]] * EulerSkinningXYZ(thetas[i].x, thetas[i].y, thetas[i].z, smpl_joints.col(i)(0), smpl_joints.col(i)(1), smpl_joints.col(i)(2));
		}
	}

	void Optimizer::ComputeSkinningLastDerivatives(const PoseEulerCoefficients& thetas, 
		const Joints& smpl_joints, 
		Eigen::Matrix4f (&palette)[JOINT_COUNT],
		Eigen::Matrix4f (&dskinning)[JOINT_COUNT * 3]) const
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

	void Optimizer::ComputeSkinningDerivatives(const PoseEulerCoefficients& thetas, const Joints& smpl_joints,
		Eigen::Matrix4f* dskinning) const
	{
		Eigen::Matrix4f palette[JOINT_COUNT];
		ComputeSkinningDerivatives(thetas, smpl_joints, palette, dskinning);
	}

	void Optimizer::ComputeSkinningDerivatives(const PoseEulerCoefficients& thetas,
		const Joints& joints,
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

			body parts ------->
			theta0.x
			theta0.y
			theta0.z
			theta1.x
			...
		
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

	Joints Optimizer::RegressJoints(const Body& body, const JOINT_TYPE& joint_type) const
	{
		switch (joint_type)
		{
		case SMPL:
			return smpl_regress_(body.vertices);
		case COCO:
			return coco_regress_(body.vertices);
		default:
			return Joints();
		}
	}

	void Optimizer::OptimizePose(const std::string& image_filename, const Eigen::Vector3f& scaling, const Eigen::Vector3f& translation, const ShapeCoefficients& betas, PoseEulerCoefficients& thetas)
	{
		float learning_rate = 1e-5f;
		float energy = 1000.f;
		float epsilon = 10;
		uint count = 0;

		std::vector<Skin> skins = generate_.GetSkins();

		for (uint iteration = 0; iteration < 1000; iteration++)
			/*while (energy > epsilon)*/
		{
			Body body = generate_(betas, thetas);

			energy = 0.f;
			float dthetas[THETA_COUNT * 3] = { 0 };

			Joints smpl_joints = smpl_regress_(body.vertices);

			// precompute skinning derivatives
			Matrix3x4f dskinning[JOINT_COUNT * 3];
			{
				Eigen::Matrix4f palette[JOINT_COUNT];

				// parent initialization
				{
					palette[0] = EulerSkinningXYZ(thetas[0].x, thetas[0].y, thetas[0].z, smpl_joints.col(0)(0), smpl_joints.col(0)(1), smpl_joints.col(0)(2));
					dskinning[0] = EulerSkinningXYZDerivativeToAlpha(thetas[0].x, thetas[0].y, thetas[0].z, smpl_joints.col(0)(0), smpl_joints.col(0)(1), smpl_joints.col(0)(2)).block<3, 4>(0, 0);
					dskinning[1] = EulerSkinningXYZDerivativeToBeta(thetas[0].x, thetas[0].y, thetas[0].z, smpl_joints.col(0)(0), smpl_joints.col(0)(1), smpl_joints.col(0)(2)).block<3, 4>(0, 0);
					dskinning[2] = EulerSkinningXYZDerivativeToGamma(thetas[0].x, thetas[0].y, thetas[0].z, smpl_joints.col(0)(0), smpl_joints.col(0)(1), smpl_joints.col(0)(2)).block<3, 4>(0, 0);
				}

				for (uint i = 1; i < JOINT_COUNT; i++)
				{
					palette[i] = palette[PARENT_INDEX[i]] * EulerSkinningXYZ(thetas[i].x, thetas[i].y, thetas[i].z, smpl_joints.col(i)(0), smpl_joints.col(i)(1), smpl_joints.col(i)(2));
					dskinning[i * 3 + 0] = (palette[PARENT_INDEX[i]]
						* EulerSkinningXYZDerivativeToAlpha(thetas[i].x, thetas[i].y, thetas[i].z, smpl_joints.col(i)(0), smpl_joints.col(i)(1), smpl_joints.col(i)(2))).block<3, 4>(0, 0);
					dskinning[i * 3 + 1] = (palette[PARENT_INDEX[i]]
						* EulerSkinningXYZDerivativeToBeta(thetas[i].x, thetas[i].y, thetas[i].z, smpl_joints.col(i)(0), smpl_joints.col(i)(1), smpl_joints.col(i)(2))).block<3, 4>(0, 0);
					dskinning[i * 3 + 2] = (palette[PARENT_INDEX[i]]
						* EulerSkinningXYZDerivativeToGamma(thetas[i].x, thetas[i].y, thetas[i].z, smpl_joints.col(i)(0), smpl_joints.col(i)(1), smpl_joints.col(i)(2))).block<3, 4>(0, 0);
				}
			}

			Joints coco_joints = coco_regress_(body.vertices);

			for (uint m = 0; m < COCO_JOINT_COUNT; m++)
			{
				Eigen::Vector3f joint = coco_joints.col(m);
				Eigen::Vector3f transformed_joint = Eigen::Scaling(scaling) * joint + translation;
				Eigen::Vector2f projection = project_(transformed_joint);
				Eigen::Vector2f error = Eigen::Vector2f(projection(0) - tracked_joints_[2 * m], projection(1) - tracked_joints_[2 * m + 1]);

				energy += error.squaredNorm();

				Eigen::Vector3f dpose_alpha(0, 0, 0);
				Eigen::Vector3f dpose_beta(0, 0, 0);
				Eigen::Vector3f dpose_gamma(0, 0, 0);

#pragma omp parallel for
				for (uint i = 0; i < VERTEX_COUNT; i++)
				{
					dpose_alpha = (
						skins[i].weight.x * dskinning[skins[i].joint_index.x * 3] +
						skins[i].weight.y * dskinning[skins[i].joint_index.y * 3] +
						skins[i].weight.z * dskinning[skins[i].joint_index.z * 3] +
						skins[i].weight.w * dskinning[skins[i].joint_index.w * 3]) * body.deformed_template[i].ToEigen().homogeneous();

					dpose_beta = (
						skins[i].weight.x * dskinning[skins[i].joint_index.x * 3 + 1] +
						skins[i].weight.y * dskinning[skins[i].joint_index.y * 3 + 1] +
						skins[i].weight.z * dskinning[skins[i].joint_index.z * 3 + 1] +
						skins[i].weight.w * dskinning[skins[i].joint_index.w * 3 + 1]) * body.deformed_template[i].ToEigen().homogeneous();

					dpose_gamma = (
						skins[i].weight.x * dskinning[skins[i].joint_index.x * 3 + 2] +
						skins[i].weight.y * dskinning[skins[i].joint_index.y * 3 + 2] +
						skins[i].weight.z * dskinning[skins[i].joint_index.z * 3 + 2] +
						skins[i].weight.w * dskinning[skins[i].joint_index.w * 3 + 2]) * body.deformed_template[i].ToEigen().homogeneous();
				}

				for (uint j = 0; j < THETA_COUNT; j++)
				{
					dthetas[j * 3 + 0] += 2.f * error.transpose() * project_.derivative(transformed_joint).transpose() * Eigen::Scaling(scaling) * dpose_alpha;
					dthetas[j * 3 + 1] += 2.f * error.transpose() * project_.derivative(transformed_joint).transpose() * Eigen::Scaling(scaling) * dpose_beta;
					dthetas[j * 3 + 2] += 2.f * error.transpose() * project_.derivative(transformed_joint).transpose() * Eigen::Scaling(scaling) * dpose_gamma;
				}
			}

			for (uint j = 0; j < THETA_COUNT * 3; j++)
			{
				thetas(j) -= learning_rate * dthetas[j];
			}

			if (count % 100 == 0)
			{
				Image image(image_filename.c_str(), 640, 480);
				//body.Draw(image, project_.GetIntrinsics(), scaling, translation);
				image.SavePNG(std::string("projections/").append("2pose_").append(std::to_string(count)).append(".png").c_str());
				std::cout << "Iteration: " << count << std::endl;
				std::cout << "Energy: " << energy << std::endl;
				std::cout << "Thetas: ";
				for (uint j = 0; j < THETA_COUNT * 3; j++)
					std::cout << thetas(j) << " ";
				std::cout << std::endl;
			}
			count++;
		}
	}

	void Optimizer::OptimizePoseFromSmplJoints2D(const JOINT_TYPE& joint_type, const ShapeCoefficients& betas,
		const Eigen::Vector3f& scaling, const Eigen::Vector3f& translation, 
		PoseEulerCoefficients& thetas)
	{
		float learning_rate = 1e-4f;
		float energy = 1000.f;
		float epsilon = 1e-5f;
		uint count = 0;

		const std::vector<Skin>& skins = generate_.GetSkins();

		std::vector<std::vector<int> > active_joints_sets = {
			//{HIP_CENTER, STOMACH}, 
			{ HIP_RIGHT, HIP_LEFT, BACKBONE, CHEST },
			{ KNEE_RIGHT, KNEE_LEFT, PECK_RIGHT, PECK_LEFT, SHOULDER_CENTER },
			{ ANKLE_RIGHT, ANKLE_LEFT, CHIN, SHOULDER_RIGHT, SHOULDER_LEFT },
			{ FOOT_RIGHT, FOOT_LEFT, ELBOW_RIGHT, ELBOW_LEFT },
			{ WRIST_RIGHT, WRIST_LEFT },
			{ HAND_RIGHT, HAND_LEFT }
		};

		for (auto& active_joints : active_joints_sets)
		{
			std::cout << "NEW ACTIVE JOINT SET" << std::endl;
			for (uint iteration = 0; iteration < 1000; iteration++)
				/*while (energy > epsilon)*/
			{
				Body body = generate_(betas, thetas);

				energy = 0.f;
				float dthetas[THETA_COUNT * 3] = { 0 };
				Joints joints = smpl_regress_(body.vertices);
				Eigen::Matrix4f* dskinning = new Eigen::Matrix4f[JOINT_COUNT * JOINT_COUNT * 3];
				ComputeSkinningDerivatives(thetas, joints, dskinning);

				for (auto& m : active_joints)
				{
					Eigen::Vector3f joint = joints.col(m);
					Eigen::Vector3f transformed_joint = Eigen::Scaling(scaling) * joint + translation;
					Eigen::Vector2f projection = project_(transformed_joint);
					Eigen::Vector2f error = Eigen::Vector2f(
						projection(0) - tracked_joints_[2 * m], 
						projection(1) - tracked_joints_[2 * m + 1]);

					energy += error.squaredNorm();

					Eigen::VectorXf regressor_m = smpl_regress_.GetRow(m);

					Eigen::Vector3f djoint_alpha(0, 0, 0);
					Eigen::Vector3f djoint_beta(0, 0, 0);
					Eigen::Vector3f djoint_gamma(0, 0, 0);

					for (uint k = 0; k < JOINT_COUNT; k++) // Update dthetas
					{
#pragma omp parallel for
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

						dthetas[ALPHA(k)] += 2.f * error.transpose() * project_.derivative(transformed_joint).transpose() * djoint_alpha;
						dthetas[BETA(k)] += 2.f * error.transpose() * project_.derivative(transformed_joint).transpose() * djoint_beta;
						dthetas[GAMMA(k)] += 2.f * error.transpose() * project_.derivative(transformed_joint).transpose() * djoint_gamma;
					}
				}

				for (auto& m : active_joints)
				{
					thetas(ALPHA(PARENT_INDEX[m])) -= learning_rate * dthetas[ALPHA(PARENT_INDEX[m])];
					thetas(BETA(PARENT_INDEX[m])) -= learning_rate * dthetas[BETA(PARENT_INDEX[m])];
					thetas(GAMMA(PARENT_INDEX[m])) -= learning_rate * dthetas[GAMMA(PARENT_INDEX[m])];
				}

				if (count % 100 == 0)
				{
					Image image(640, 480);
					Image::Draw3D(image, project_.GetIntrinsics(), scaling, translation, WHITE, body.vertices);
					Image::Draw3D(image, project_.GetIntrinsics(), scaling, translation, BLUE, 2, Joints2Vector(joints));

					std::vector<float3> active_joint_coordinates;
					active_joint_coordinates.reserve(10);
					for (auto&m : active_joints)
					{
						active_joint_coordinates.push_back(float3(joints.col(m)));
					}

					Image::Draw3D(image, project_.GetIntrinsics(), scaling, translation, GREEN, 2, active_joint_coordinates);
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

				if (energy < epsilon)
					break;
			}
		}
	}

	void Optimizer::OptimizePoseFromSmplJoints3D(const JOINT_TYPE& joint_type, const ShapeCoefficients& betas, PoseEulerCoefficients& thetas)
	{
		float learning_rate = 1e-1f;
		float energy = 1000.f;
		float epsilon = 1e-5f;
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
			for (uint iteration = 0; iteration < 1000; iteration++)
				/*while (energy > epsilon)*/
			{
				Body body = generate_(betas, thetas);

				energy = 0.f;
				float dthetas[THETA_COUNT * 3] = { 0 };
				Joints joints = smpl_regress_(body.vertices);
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

					Eigen::VectorXf regressor_m = smpl_regress_.GetRow(m);

					Eigen::Vector3f djoint_alpha(0, 0, 0);
					Eigen::Vector3f djoint_beta(0, 0, 0);
					Eigen::Vector3f djoint_gamma(0, 0, 0);

					for (uint k = 0; k < JOINT_COUNT; k++) // Update dthetas
					{
#pragma omp parallel for
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
					thetas(ALPHA(PARENT_INDEX[m])) -= learning_rate * dthetas[ALPHA(PARENT_INDEX[m])];
					thetas(BETA(PARENT_INDEX[m])) -= learning_rate * dthetas[BETA(PARENT_INDEX[m])];
					thetas(GAMMA(PARENT_INDEX[m])) -= learning_rate * dthetas[GAMMA(PARENT_INDEX[m])];
				}

				if (count % 100 == 0)
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

				if (energy < epsilon)
					break;
			}
		}
	}

	void Optimizer::operator()(const std::string& image_filename, ShapeCoefficients& betas, PoseAxisAngleCoefficients& thetas, Eigen::Vector3f& scaling, Eigen::Vector3f& translation)
	{
		Body body = generate_(betas, thetas);
		//body.Dump("new.obj");

		//OptimizeExtrinsics(image_filename, body, scaling, translation);

		//scaling = Eigen::Vector3f(1.32409f, 1.42544f, 0.997468f);
		// jake - tpose
		//translation = Eigen::Vector3f(-0.279263f, 0.366249f, - 4.01858f);

		// andrei - surrender
		translation = Eigen::Vector3f(0.0404971f, 0.416566f, -3.69404f);

		//OptimizeShape(image_filename, scaling, translation, thetas, betas);

		PoseEulerCoefficients eulers;

		// jake - tpose
		// betas = { -0.287861f, -3.47998f, 3.58785f, -1.76725f, 2.16237f, 0.757985f, -1.80657f, -0.411921f, 3.06527f, -0.830053f };

		// andrei - surrender
		betas = { 0.0631728f, 0.463201f, 1.29514f, -0.267553f, -0.250164f, 0.171496f, -0.185418f, 0.0712007f, 0.116954f, -0.326455f, };

		OptimizePose(image_filename, scaling, translation, betas, eulers);
	}
}