#pragma once
#include "Definitions.h"

class LevenbergMarquardt
{
public:
	LevenbergMarquardt(const int unknowns, const bool logging_on) :
		unknowns_(unknowns), logging_on_(logging_on), delta_old_(Eigen::VectorXf::Zero(unknowns))
	{
	}

	// true if minimized, false otherwise
	bool operator()(const Eigen::MatrixXf& jacobian, const Eigen::VectorXf& error,
		const int residuals, const int iteration, Eigen::VectorXf& delta);

private:
	const int unknowns_;
	Eigen::VectorXf delta_old_;

	const float MINF = -100000.f;
	const bool logging_on_;

	const float lambda_min_ = 0.1f;
	const float alpha_ = 1.8f; // increase lambda factor when error goes down
	const float beta_ = 0.6f; // decrease lambda factor when error goes up
	float lambda_ = 2.0f; // the linear step factor
	float last_residual_error_ = MINF;
};