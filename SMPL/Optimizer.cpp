#include "Optimizer.h"

namespace smpl
{
	Optimizer::Optimizer(Configuration& configuration, const Generator& generate, const std::vector<float>& tracked_joints) :
		generate_(generate), project_(configuration.intrinsics),
		coco_regress_(configuration.coco_regressor, COCO_JOINT_COUNT), smpl_regress_(configuration.smpl_regressor, JOINT_COUNT),
		tracked_joints_(tracked_joints)
	{
		dshape_.reserve(COCO_JOINT_COUNT * BETA_COUNT);
		{
			std::vector<float3> shapedirs = generate_.GetShapeDirs();
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
			body.Draw(image, project_.GetIntrinsics(), scaling, translation);

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
				body.Draw(image, project_.GetIntrinsics(), scaling, translation);
				image.SavePNG(std::string("projections/").append("0extrinsics_").append(std::to_string(count)).append(".png").c_str());
				std::cout << "Iteration: " << count << std::endl;
				std::cout << "Energy: " << energy << std::endl;
				std::cout << "Scaling: " << std::endl << scaling << std::endl;
				std::cout << "Translation:" << std::endl << translation << std::endl << std::endl;
			}
			count++;
		}
	}

	void Optimizer::OptimizeShape(const std::string& image_filename, const Eigen::Vector3f& scaling, const Eigen::Vector3f& translation, const PoseAxisAngleCoefficients& thetas, ShapeCoefficients& betas)
	{
		float learning_rate = 1e-3f;
		float energy = 1000.f;
		float epsilon = 10;
		uint count = 0;

		for (uint iteration = 0; iteration < 1000; iteration++)
			/*while (energy > epsilon)*/
		{
			std::cout << "Generate new body " << count << std::endl;
			Body body = generate_(betas, thetas);
			std::cout << "Start optimization " << count << std::endl;

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

			if (count % 5 == 0)
			{
				Image image(image_filename.c_str(), 640, 480);
				body.Draw(image, project_.GetIntrinsics(), scaling, translation);
				image.SavePNG(std::string("projections/").append("1shape_").append(std::to_string(count)).append(".png").c_str());
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
					palette[0] = EulerSkinning(thetas[0].x, thetas[0].y, thetas[0].z, smpl_joints.col(0)(0), smpl_joints.col(0)(1), smpl_joints.col(0)(2));
					dskinning[0] = EulerTruncatedSkinningDerivativeToAlpha(thetas[0].x, thetas[0].y, thetas[0].z, smpl_joints.col(0)(0), smpl_joints.col(0)(1), smpl_joints.col(0)(2));
					dskinning[1] = EulerTruncatedSkinningDerivativeToBeta(thetas[0].x, thetas[0].y, thetas[0].z, smpl_joints.col(0)(0), smpl_joints.col(0)(1), smpl_joints.col(0)(2));
					dskinning[2] = EulerTruncatedSkinningDerivativeToGamma(thetas[0].x, thetas[0].y, thetas[0].z, smpl_joints.col(0)(0), smpl_joints.col(0)(1), smpl_joints.col(0)(2));
				}

				for (uint i = 1; i < JOINT_COUNT; i++)
				{
					palette[i] = palette[PARENT_INDEX[i]] * EulerSkinning(thetas[i].x, thetas[i].y, thetas[i].z, smpl_joints.col(i)(0), smpl_joints.col(i)(1), smpl_joints.col(i)(2));
					dskinning[i * 3 + 0] = (palette[PARENT_INDEX[i]]
						* EulerSkinningDerivativeToAlpha(thetas[i].x, thetas[i].y, thetas[i].z, smpl_joints.col(i)(0), smpl_joints.col(i)(1), smpl_joints.col(i)(2))).block<3, 4>(0, 0);
					dskinning[i * 3 + 1] = (palette[PARENT_INDEX[i]]
						* EulerSkinningDerivativeToBeta(thetas[i].x, thetas[i].y, thetas[i].z, smpl_joints.col(i)(0), smpl_joints.col(i)(1), smpl_joints.col(i)(2))).block<3, 4>(0, 0);
					dskinning[i * 3 + 2] = (palette[PARENT_INDEX[i]]
						* EulerSkinningDerivativeToGamma(thetas[i].x, thetas[i].y, thetas[i].z, smpl_joints.col(i)(0), smpl_joints.col(i)(1), smpl_joints.col(i)(2))).block<3, 4>(0, 0);
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
				body.Draw(image, project_.GetIntrinsics(), scaling, translation);
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