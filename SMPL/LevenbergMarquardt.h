#pragma once
#include "Definitions.h"
#include <cereal/cereal.hpp>
#include <cereal/archives/json.hpp>
#include <fstream>

class LevenbergMarquardt
{
public:

	struct Configuration
	{
		Configuration(const std::string& config_path)
		{
			std::ifstream in(config_path);
			if (!in) MessageBoxA(nullptr,
				std::string("File not found: ").append(config_path).c_str(),
				"Error", MB_OK);
			cereal::JSONInputArchive archive(in);

			Serialize(archive);
		}

		template<class Archive>
		void Serialize(Archive& archive)
		{
			archive(
				CEREAL_NVP(lambda_min),
				CEREAL_NVP(alpha),
				CEREAL_NVP(beta),
				CEREAL_NVP(lambda)
			);
		}

		float lambda_min = 0.1f;
		float alpha = 1.8f;
		float beta = 0.6f;
		float lambda = 2.0f;
	};

	LevenbergMarquardt(const int unknowns, const bool logging_on) :
		logging_on_(logging_on), unknowns_(unknowns), 
        JtJ_(Eigen::MatrixXf::Zero(unknowns, unknowns)),
        JtF_(Eigen::VectorXf::Zero(unknowns)),
        parameters_(Eigen::VectorXf::Zero(unknowns))
	{
	}

	LevenbergMarquardt(const int unknowns, const bool logging_on, const Configuration& config) :
        logging_on_(logging_on), unknowns_(unknowns),
        JtJ_(Eigen::MatrixXf::Zero(unknowns, unknowns)),
        JtF_(Eigen::VectorXf::Zero(unknowns)),
        parameters_(Eigen::VectorXf::Zero(unknowns)),
		lambda_min_(config.lambda_min), alpha_(config.alpha), beta_(config.beta), 
        lambda_(config.lambda)
	{
	}

	void operator()(const Eigen::MatrixXf& jacobian, const Eigen::VectorXf& error,
		const int residuals, const int iteration, Eigen::VectorXf& parameters);

private:
    const bool logging_on_;

	const int unknowns_;
    Eigen::MatrixXf JtJ_;
    Eigen::VectorXf JtF_;
    Eigen::VectorXf parameters_;
    float last_residual_error_ = 1e30f;
    
	const float lambda_min_ = 0.05f;
	const float alpha_ = 1.8f;		// increase lambda factor when error goes up
	const float beta_ = 0.6f;		// decrease lambda factor when error goes down
	float lambda_ = 2.0f;			// the linear step factor, the bigger the more GD ist the step, and smaller the stepsize
};