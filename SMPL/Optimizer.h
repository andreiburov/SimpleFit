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
				ReadSparseMatrixFile(configuration_path + std::string("/coco_regressor.txt"), coco_regressor);
				ReadSparseMatrixFile(configuration_path + std::string("/smpl_regressor.txt"), smpl_regressor);
			}

			SparseMatrix coco_regressor;
			SparseMatrix smpl_regressor;

			// taken from calibration (Utilities project)
			float intrinsics[9] = {
				-8.6140702901296027e+02f, 0.f, 3.2324049091782535e+02f,
				0.f, 8.3086714228541780e+02f, 2.5605035868808250e+02f,
				0.f, 0.f, 1.f
			};
		};

		Optimizer(Configuration& configuration, const Generator& generate, const std::vector<float>& tracked_joints);

		void OptimizeExtrinsics(const std::string& image_filename, const Body& body, Eigen::Vector3f& scaling, Eigen::Vector3f& translation);

		void OptimizeShape(const std::string& image_filename, const Eigen::Vector3f& scaling, const Eigen::Vector3f& translation, const PoseAxisAngleCoefficients& thetas, ShapeCoefficients& betas);

		void OptimizePose(const std::string& image_filename, const Eigen::Vector3f& scaling, const Eigen::Vector3f& translation, const ShapeCoefficients& betas, PoseEulerCoefficients& thetas);

		void operator()(const std::string& image_filename, ShapeCoefficients& betas, PoseAxisAngleCoefficients& thetas, Eigen::Vector3f& scaling, Eigen::Vector3f& translation);

	private:
		const Generator generate_;
		const Projector project_;
		const JointRegressor coco_regress_;
		const JointRegressor smpl_regress_;
		const std::vector<float> tracked_joints_;
		std::vector<Eigen::Vector3f> dshape_;
	};
}