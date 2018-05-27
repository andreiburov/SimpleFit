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
		}

		void operator()(ShapeCoefficients& betas, PoseCoefficients& thetas, Eigen::Vector3f& scaling, Eigen::Vector3f& translation)
		{
			Body body = generate_(betas, thetas);
			//body.Dump("new.obj");
			float learning_rate = 1e-7f;
			float energy = 1000.f;
			float epsilon = 10;
			uint count = 0;
			
			/*for (uint iteration = 0; iteration < 100; iteration++)*/
			while (energy > epsilon)
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
					dscaling += Eigen::Scaling(joint) * project_.derivative(transformed_joint) * error * 2.f;
					dtranslation += project_.derivative(transformed_joint) * error * 2.f;
				}

				image.SavePNG("projection.png");

				scaling -= learning_rate * dscaling;
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

			// f(x) = f(a) - f'(a) * (x-a)
			// we will try to do least squares on tracked and estimated joints
			// for this one has to calculate derivative of projected smpl w.r.t. betas and thetas
		}

	private:
		const Generator generate_;
		const Projector project_;
		const JointRegressor regress_;
		const std::vector<float> tracked_joints_;
	};
}