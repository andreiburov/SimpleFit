#include <SMPL.h>
#include <catch.hpp>

using namespace smpl;

TEST_CASE("Project - Intrinsics")
{
	std::string model = "../Model/";
	Generator generator(model);
	Projector projector(model);

	Body body = generator();
	Eigen::Vector3f translation(0.f, 0.1f, -4.2f);

	Image image;
	Image::Draw3D(image, Pixel::White(), projector, translation, body.vertices);

	image.SavePNG("body.png");
}

Eigen::Matrix4f CalculateNDC(float width, float height, float near_, float far_, bool is_rhs_)
{
	// map a clipping space cube to directx ndc space
	Eigen::Matrix4f ndc(Eigen::Matrix4f::Zero());

	// 0 x 0 ; width x height -> -1 x -1 ; 1 x 1
	ndc(0, 0) = 2.f / width;
	ndc(0, 2) = -1.f;
	ndc(1, 1) = 2.f / height;
	ndc(1, 2) = -1.f;

	// RHS
	if (is_rhs_)
	{
		float range = far_ / (far_ - near_);
		ndc(2, 2) = range;
		ndc(2, 3) = near_ * range;
		ndc(3, 2) = 1.f;
		ndc *= -1.f; // changing the sign for clip test
	}
	else
	{
		float range = far_ / (far_ - near_);
		ndc(2, 2) = range;
		ndc(2, 3) = -near_ * range;
		ndc(3, 2) = 1.f;
	}

	return ndc;
}

Eigen::Matrix4f Intrinsics(bool is_rhs)
{
	Eigen::Matrix4f intrinsics(Eigen::Matrix4f::Identity());
	intrinsics(0, 0) = 861.4070f;
	intrinsics(0, 2) = 323.2405f;
	intrinsics(1, 1) = 830.8671f;
	intrinsics(1, 2) = 256.0504f;
	intrinsics(3, 3) = 1.f;

	if (is_rhs)
	{
		intrinsics(0, 0) *= -1.f;
		intrinsics(1, 1) *= -1.f;
	}

	return intrinsics;
}

Eigen::Matrix4f Ndc(float width, float height, bool is_rhs)
{
	Eigen::Matrix4f ndc =
		CalculateNDC(width, height, 0.1f, 10.f, is_rhs);

	return ndc;
}

TEST_CASE("Project - DirectX")
{
	std::string model = "../Model/";
	Generator generator(model);
	Projector projector(model);

	bool is_rhs = projector.IsRhs() ? true : false;

	Body body = generator();
	Eigen::Vector3f translation(0.f, 0.1f, -4.2f);

	Eigen::Matrix4f view = projector.CalculateView(translation);
	Eigen::Matrix4f proj = projector.DirectXProjection(IMAGE_WIDTH, IMAGE_HEIGHT);
	Eigen::Matrix4f intr = Intrinsics(is_rhs);
	Eigen::Matrix4f ndc = Ndc((float)IMAGE_WIDTH, (float)IMAGE_HEIGHT, is_rhs);

	for (int j = 0; j < VERTEX_COUNT; j++)
	{
		Eigen::Vector4f m = body.vertices[j].ToEigen().homogeneous();
		Eigen::Vector4f v = view * m;
		Eigen::Vector4f i = intr * v;

		Eigen::Vector4f d = i / i(2); // divide by z
		float xx = d(0);
		float yy = IMAGE_HEIGHT - d(1);

		Eigen::Vector4f n = ndc * i;
		Eigen::Vector4f h = n / n(3);

		Eigen::Vector4f dx = proj * v;
		Eigen::Vector4f hx = dx / dx(3);

		// viewport transform
		float x = (h(0) + 1) * IMAGE_WIDTH * 0.5f;
		float y = (1 - h(1)) * IMAGE_HEIGHT * 0.5f;
		
		float xxx = (hx(0) + 1) * IMAGE_WIDTH * 0.5f;
		float yyy = (1 - hx(1)) * IMAGE_HEIGHT * 0.5f;

		float w = x / y;
	}

	Image image;
	Image::Draw3D(image, Pixel::White(), projector, translation, body.vertices);

	image.SavePNG("body.png");
}