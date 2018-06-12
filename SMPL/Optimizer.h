#pragma once

#include "Utils.h"
#include "Definitions.h"
#include "Projector.h"
#include "Generator.h"

namespace smpl
{
	class Optimizer
	{
	public:

		struct Configuration
		{
			Configuration(const std::string& configuration_path)
			{
				ReadSparseMatrixFile(configuration_path + std::string("/coco_regressor.txt"), joint_regressor);
			}

			SparseMatrix joint_regressor;

			// taken from calibration (Utilities project)
			float intrinsics[9] = {
				-8.6140702901296027e+02f, 0.f, 3.2324049091782535e+02f,
				0.f, 8.3086714228541780e+02f, 2.5605035868808250e+02f,
				0.f, 0.f, 1.f
			};
		};

		Optimizer(Configuration& configuration, const Generator& generate, const std::vector<float>& tracked_joints) :
			generate_(generate), project_(configuration.intrinsics), 
			regress_(configuration.joint_regressor, COCO_JOINT_COUNT), tracked_joints_(tracked_joints)
		{
			dshape_.reserve(COCO_JOINT_COUNT * BETA_COUNT);
			{
				std::vector<float3> shapedirs = generate_.GetShapeDirs();
				for (uint m = 0; m < COCO_JOINT_COUNT; m++)
				{
					Eigen::VectorXf regressor_m = regress_.GetRow(m);
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

		void OptimizeExtrinsics(const Body& body, Eigen::Vector3f& scaling, Eigen::Vector3f& translation)
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

				Joints joints = regress_(body.vertices);

				for (uint i = 0; i < COCO_JOINT_COUNT; i++)
				{
					Eigen::Vector3f joint = joints.col(i);
					Eigen::Vector3f transformed_joint = Eigen::Scaling(scaling) * joint + translation;
					Eigen::Vector2f projection = project_(transformed_joint);
					Eigen::Vector2f error = Eigen::Vector2f(projection(0) - tracked_joints_[2*i], projection(1) - tracked_joints_[2*i + 1]);
					
					energy += error.squaredNorm();
					//dscaling += Eigen::Scaling(joint) * project_.derivative(transformed_joint) * error * 2.f;
					dtranslation += project_.derivative(transformed_joint) * error * 2.f;
				}

				image.SavePNG("projection.png");

				//scaling -= learning_rate * dscaling;
				translation -= learning_rate * dtranslation;

				if (count % 100 == 0)
				{
					std::cout << "Iteration: " << count << std::endl;
					std::cout << "Energy: " << energy << std::endl;
					std::cout << "Scaling: " << std::endl << scaling << std::endl;
					std::cout << "Translation:" << std::endl << translation << std::endl << std::endl;
				}
				count++;
			}
		}

		void OptimizeShape(const Eigen::Vector3f& scaling, const Eigen::Vector3f& translation, const PoseCoefficients& thetas, ShapeCoefficients& betas)
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
				float dbetas[10] = { 0 };

				Joints joints = regress_(body.vertices);

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
					Image image("test20.png", 640, 480);
					body.Draw(image, project_.GetIntrinsics(), scaling, translation);
					image.SavePNG(std::string("projections/").append(std::to_string(count)).append(".png").c_str());
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

		void OptimizePose(const Eigen::Vector3f& scaling, const Eigen::Vector3f& translation, const ShapeCoefficients& betas, PoseCoefficients& thetas)
		{
			//float learning_rate = 1e-3f;
			//float energy = 1000.f;
			//float epsilon = 10;
			//uint count = 0;

			//for (uint iteration = 0; iteration < 1000; iteration++)
			//	/*while (energy > epsilon)*/
			//{
			//	std::cout << "Generate new body " << count << std::endl;
			//	Body body = generate_(betas, thetas);
			//	std::cout << "Start optimization " << count << std::endl;

			//	energy = 0.f;
			//	float dbetas[10] = { 0 };

			//	Joints joints = regress_(body.vertices);

			//	for (uint m = 0; m < COCO_JOINT_COUNT; m++)
			//	{
			//		Eigen::Vector3f joint = joints.col(m);
			//		Eigen::Vector3f transformed_joint = Eigen::Scaling(scaling) * joint + translation;
			//		Eigen::Vector2f projection = project_(transformed_joint);
			//		Eigen::Vector2f error = Eigen::Vector2f(projection(0) - tracked_joints_[2 * m], projection(1) - tracked_joints_[2 * m + 1]);

			//		energy += error.squaredNorm();

			//		for (uint j = 0; j < BETA_COUNT; j++)
			//		{
			//			dbetas[j] += 2.f * error.transpose() * project_.derivative(transformed_joint).transpose() * Eigen::Scaling(scaling) * dshape_[m*BETA_COUNT + j];
			//		}
			//	}

			//	for (uint j = 0; j < BETA_COUNT; j++)
			//	{
			//		betas[j] -= learning_rate * dbetas[j];
			//	}

			//	if (count % 5 == 0)
			//	{
			//		Image image("test20.png", 640, 480);
			//		body.Draw(image, project_.GetIntrinsics(), scaling, translation);
			//		image.SavePNG(std::string("projections/").append(std::to_string(count)).append(".png").c_str());
			//		std::cout << "Iteration: " << count << std::endl;
			//		std::cout << "Energy: " << energy << std::endl;
			//		std::cout << "Betas: ";
			//		for (uint j = 0; j < BETA_COUNT; j++)
			//			std::cout << betas[j] << " ";
			//		std::cout << std::endl;
			//	}
			//	count++;
			//}
		}

		void operator()(ShapeCoefficients& betas, PoseCoefficients& thetas, Eigen::Vector3f& scaling, Eigen::Vector3f& translation)
		{
			Body body = generate_(betas, thetas);
			//body.Dump("new.obj");
			// betas
			float learning_rate = 1e-3f; 
			// translation
			//float learning_rate = 1e-7f;
			float energy = 1000.f;
			float epsilon = 10;
			uint count = 0;

			//OptimizeExtrinsics(body, scaling, translation);

			//scaling = Eigen::Vector3f(1.32409f, 1.42544f, 0.997468f);
			translation = Eigen::Vector3f(-0.279263f, 0.366249f, - 4.01858f);

			//OptimizeShape(scaling, translation, thetas, betas);

			betas = { -0.287861f, -3.47998f, 3.58785f, -1.76725f, 2.16237f, 0.757985f, -1.80657f, -0.411921f, 3.06527f, -0.830053f };
			OptimizePose(scaling, translation, betas, thetas);
		}

	private:
		const Generator generate_;
		const Projector project_;
		const JointRegressor regress_;
		const std::vector<float> tracked_joints_;
		std::vector<Eigen::Vector3f> dshape_;
	};
}