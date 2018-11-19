#include "LevenbergMarquardt.h"

bool LevenbergMarquardt::operator()(const Eigen::MatrixXf& jacobian,
	const Eigen::VectorXf& error, const int residuals, const int iteration,
	Eigen::VectorXf& delta)
{
	Eigen::MatrixXf Jt = jacobian.transpose();
	Eigen::MatrixXf JtJ = Jt * jacobian;
	Eigen::VectorXf JtF = Jt * error;
	Eigen::MatrixXf JtJ_diag = JtJ.diagonal().asDiagonal();

	float current_residual_error = error.squaredNorm();
	if (logging_on_)
	std::cout << "Error " << current_residual_error << std::endl;
	
	if (last_residual_error_ == MINF ||
		(last_residual_error_ >= current_residual_error))
	{
		JtJ += lambda_ * JtJ_diag;

		Eigen::ConjugateGradient<Eigen::MatrixXf, Eigen::Lower | Eigen::Upper> cg;
		cg.compute(JtJ);
		delta = cg.solve(JtF);

		delta_old_ = delta;
		last_residual_error_ = current_residual_error;

		lambda_ *= beta_;
		if (lambda_ < lambda_min_) lambda_ = lambda_min_;
		return true;
	}
	else if (last_residual_error_ < current_residual_error)
	{
		delta = delta_old_;
		lambda_ *= alpha_;
		return false;
	}

	return false;
}