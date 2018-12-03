#include "LevenbergMarquardt.h"

void LevenbergMarquardt::operator()(const Eigen::MatrixXf& jacobian,
	const Eigen::VectorXf& error, const int residuals, const int iteration,
	Eigen::VectorXf& parameters)
{
    float current_residual_error = error.squaredNorm();
    if (logging_on_)
        std::cout << "Total error: " << current_residual_error << std::endl;

    if (current_residual_error < last_residual_error_)
    {
        last_residual_error_ = current_residual_error;
        parameters_ = parameters;
        Eigen::MatrixXf Jt = jacobian.transpose();
        JtJ_ = Jt * jacobian;
        JtF_ = Jt * error;

        Eigen::MatrixXf JtJ_damped(JtJ_);
        JtJ_damped += lambda_ * JtJ_.diagonal().asDiagonal();
        lambda_ *= beta_; // bigger step less GD
        if (lambda_ < lambda_min_) lambda_ = lambda_min_;

        Eigen::ConjugateGradient<Eigen::MatrixXf, Eigen::Lower | Eigen::Upper> cg;
        cg.compute(JtJ_damped);
        Eigen::VectorXf delta = cg.solve(JtF_);

        parameters -= delta;
    }
    else
    {
        Eigen::MatrixXf JtJ_damped(JtJ_);
        JtJ_damped += lambda_ * JtJ_.diagonal().asDiagonal();
        lambda_ *= alpha_;

        Eigen::ConjugateGradient<Eigen::MatrixXf, Eigen::Lower | Eigen::Upper> cg;
        cg.compute(JtJ_damped);
        Eigen::VectorXf delta = cg.solve(JtF_);

        parameters = parameters_ - delta;
    }

    std::cout << "Lambda: " << lambda_ << std::endl;
}