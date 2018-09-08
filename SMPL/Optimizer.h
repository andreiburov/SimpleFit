#pragma once

#include "Utils.h"
#include "Definitions.h"
#include "Projector.h"
#include "Generator.h"

#include <map>

namespace smpl
{
	constexpr uint ALPHA(int idx)
	{
		return idx*3;
	}

	constexpr uint BETA(int idx)
	{
		return idx*3 + 1;
	}

	constexpr uint GAMMA(int idx)
	{
		return idx*3 + 2;
	}

	class Optimizer
	{
	public:
		
		enum JOINT_TYPE
		{
			SMPL, COCO
		};

		struct Configuration
		{
			SparseMatrix coco_regressor;
			SparseMatrix smpl_regressor;

			float intrinsics[9];

			std::map<std::string, float> optimization_parameters;

			// taken from calibration (Utilities project)
			/*float intrinsics[9] = {
			-8.6140702901296027e+02f, 0.f, 3.2324049091782535e+02f,
			0.f, 8.3086714228541780e+02f, 2.5605035868808250e+02f,
			0.f, 0.f, 1.f
			};*/

			Configuration(const std::string& configuration_path)
			{
				ReadSparseMatrixFile(configuration_path + std::string("/coco_regressor.txt"), coco_regressor);
				ReadSparseMatrixFile(configuration_path + std::string("/smpl_regressor.txt"), smpl_regressor);
				std::ifstream intrinsics_file(configuration_path + std::string("/intrinsics.txt"));
				if (intrinsics_file.fail()) MessageBoxA(NULL, "File not found: intrinsics.txt", "Error", MB_OK);
				for (uint i = 0; i < 9; i++)
				{
					intrinsics_file >> intrinsics[i];
				}
				ReadOptimizationConfiguration(configuration_path + std::string("/optimization_configuration.txt"));
			}

			void ReadOptimizationConfiguration(const std::string& filename);
		};


		Optimizer(Configuration& configuration, const Generator& generate, const std::vector<float>& tracked_joints);

		void OptimizeExtrinsics(const std::string& image_filename, const Body& body, Eigen::Vector3f& translation);

		void OptimizeShapeFromJoints2D(const JOINT_TYPE& joint_type, const std::string& image_filename, 
			const Eigen::Vector3f& translation, const PoseEulerCoefficients& thetas,
			ShapeCoefficients& betas);

		void OptimizePoseFromJoints2D(const JOINT_TYPE& joint_type, const std::string& image_filename, 
			const Eigen::Vector3f& translation, const ShapeCoefficients& betas,
			PoseEulerCoefficients& thetas);

		void OptimizePoseFromSmplJoints3D(const ShapeCoefficients& betas, PoseEulerCoefficients& thetas);

		void ReconstructCamera(const std::string& image_filename, Eigen::Vector3f& translation,
			ShapeCoefficients& betas, PoseEulerCoefficients& thetas, uint& count);

		// regularized translation, theta1, ... , thetaN, betas 
		float ReconstructTotal(const std::string& image_filename, Eigen::Vector3f& translation,
			ShapeCoefficients& betas, PoseEulerCoefficients& thetas, uint& count);

		// translation, theta0, regularized betas
		void ReconstructCoarse(const std::string& image_filename, Eigen::Vector3f& translation,
			ShapeCoefficients& betas, PoseEulerCoefficients& thetas, uint& count);

		// regularized translation, theta1, ... , thetaN, betas 
		void ReconstructFine(const std::string& image_filename, Eigen::Vector3f& translation,
			ShapeCoefficients& betas, PoseEulerCoefficients& thetas, uint& count);

		void Reconstruct(const std::string& image_filename, Eigen::Vector3f& translation,
			ShapeCoefficients& betas, PoseEulerCoefficients& thetas, uint& count);
		
		void ComputeShapeDerivativesAllJointTypes();
		
		void ComputeShapeDerivatives(const JOINT_TYPE& joint_type,
			std::vector<Eigen::Vector3f>& dshape);
		
		void ComputeSkinning(const PoseEulerCoefficients& thetas, const Joints& smpl_joints,
			Eigen::Matrix4f(&palette)[JOINT_COUNT]) const;

		void ComputeSkinningLastDerivatives(const PoseEulerCoefficients& thetas, const Joints& smpl_joints, 
			Eigen::Matrix4f (&palette)[JOINT_COUNT], Eigen::Matrix4f (&dskinning)[JOINT_COUNT * 3]) const;

		void ComputeSkinningDerivatives(const PoseEulerCoefficients& thetas, const Joints& smpl_joints,
			Eigen::Matrix4f(&palette)[JOINT_COUNT], Eigen::Matrix4f* dskinning) const;

		void ComputeSkinningDerivatives(const PoseEulerCoefficients& thetas, const Joints& smpl_joints,
			Eigen::Matrix4f* dskinning) const;

		void ComputeJacobianAndError(Eigen::MatrixXf& jacobian, Eigen::VectorXf& error, 
			const Body& body, const Joints& coco_joints, const Joints& smpl_joint_, 
			const Eigen::Vector3f& translation, const ShapeCoefficients& betas,
			const PoseEulerCoefficients& thetas, int active_set, 
			float shape_prior_weight, float pose_prior_weight);

		void Log(const std::string& image_filename, const Body& body,
			const Joints& reconstruction_joints, const Joints& smpl_joints,
			const Eigen::Vector3f& translation, int active_set, int count) const;

		Joints RegressJoints(const Body& body, const JOINT_TYPE& joint_type) const;

		void operator()(const std::string& image_filename, ShapeCoefficients& betas, 
			PoseAxisAngleCoefficients& thetas, Eigen::Vector3f& translation);

	private:
		const std::map<std::string, float> optimization_parameters;
		const Generator generate_;
		const Projector project_;
		const JointRegressor coco_regress_;
		const JointRegressor smpl_regress_;
		const std::vector<float> tracked_joints_;
		std::vector<Eigen::Vector3f> dshape_coco_;
		std::vector<Eigen::Vector3f> dshape_smpl_;

		float learning_rate_ = 1e-7f;
		uint iterations_ = 1000;
		uint log_every_ = 100;
		std::vector<std::vector<int> > checked_joint_sets_smpl_;
		std::vector<std::vector<int> > movable_joint_sets_smpl_;
		std::vector<std::vector<int> > checked_joint_sets_coco_;
		std::vector<std::vector<int> > movable_joint_sets_coco_;
		std::vector< int > active_betas_;
	};
}