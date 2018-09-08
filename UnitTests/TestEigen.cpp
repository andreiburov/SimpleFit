#include <catch.hpp>
#include <Eigen/Eigen>
#include <iostream>

TEST_CASE("Initialize Matrix")
{
	Eigen::MatrixXf m(6, 2);

	Eigen::MatrixXf sm1(3, 2);
	sm1 << Eigen::Vector3f(1, 2, 3), Eigen::Vector3f(4,5,6);

	Eigen::MatrixXf sm2(3, 2);
	sm2 << Eigen::Vector3f(1, 2, 3), Eigen::Vector3f(4, 5, 6);
		
	m << sm1, sm2;

	std::cout << "Hello" << std::endl;
}