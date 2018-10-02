#include <catch.hpp>
#include <EulerAngles.h>
#include <SMPL.h>

using namespace smpl;

Eigen::Matrix4f CreateNDC(float left, float right, float bottom, float top, float _near, float _far)
{
	// map a clipping space cube to directx ndc space
	float reciprocal_width = 1.0f / (right - left);
	float reciprocal_height = 1.0f / (top - bottom);
	float reciprocal_range = 1.0f / (_far - _near);

	Eigen::Matrix4f NDC(Eigen::Matrix4f::Zero());
	NDC(0, 0) = reciprocal_width + reciprocal_width;
	NDC(1, 1) = reciprocal_height + reciprocal_height;
	NDC(2, 2) = reciprocal_range;
	NDC(0, 3) = -(left + right) * reciprocal_width;
	NDC(1, 3) = -(top + bottom) * reciprocal_height;
	NDC(2, 3) = -reciprocal_range * _near;
	NDC(3, 3) = 1.f;
	return NDC;
}

Eigen::Matrix4f CreateProjection(const Eigen::Matrix3f& calibration, float _near, float _far)
{
	Eigen::Matrix4f projection(Eigen::Matrix4f::Zero());
	// intrinsics in pixel width (the unit does matter)
	projection(0, 0) = 861.407029f;
	projection(0, 2) = 323.240491f;
	projection(1, 1) = 830.867142f; 
	projection(1, 2) = 256.050359f;
	projection(2, 2) = 1.f;
	projection(3, 2) = 1.f;
	// we decided not to preserve the depth info, near and far not used
	return projection;
}

Eigen::Matrix4f CreateView()
{
	Eigen::Matrix4f view(Eigen::Matrix4f::Identity());
	// mesh is facing from us, rotate 180 around y
	view(0, 0) = -1;
	view(1, 1) = 1;
	view(2, 2) = -1;
	// mesh should be put at distance and up the y
	view(1, 3) = 0.2f;
	view(2, 3) = 4.f;
	return view;
}

namespace silhouette_reconstruction
{
	TEST_CASE("Silhouette2")
	{
		ShapeCoefficients shape;
		PoseEulerCoefficients pose;
		//shape << std::string("-1 -5");

		Optimizer::Configuration configuration("../Model");
		Projector project(configuration.intrinsics);
		Generator generate(Generator::Configuration(std::string("../Model")));

		Eigen::Matrix4f view(CreateView());

		float _near = 0.001f;
		float _far = 10.f;

		Eigen::Matrix4f projection(
			CreateNDC(0, IMAGE_WIDTH, 0, IMAGE_HEIGHT, _near, _far)*
			CreateProjection(project.GetIntrinsics(), _near, _far));

		SilhouetteMaker silhouette_maker(generate(shape, pose));	
		Image input = silhouette_maker(generate(shape, pose), view, projection);

		// Find correspondances of the silhouette boundaries
		// - calculate normals of the body
		// - project body and the normals
		// - detect vertices on the boudary
		// - do bresenham marching to find the correspondances

		// Run the reconstruction using the energy defined as a sum of quadratic differences between the correspondances
		system("pause");
	}

	TEST_CASE("Silhouette")
	{
		Image input("input.png");
		Image model("model.png");

		BYTE a = 0U;
		BYTE b = 255U;



		//SilhouetteOptimizer silhouette_optimizer;
		//silhouette_optimizer.FindCorrespondances(input, model);

		// Find correspondances of the silhouette boundaries
		// - calculate normals of the body
		// - project body and the normals
		// - detect vertices on the boudary
		// - do bresenham marching to find the correspondances

		// Run the reconstruction using the energy defined as a sum of quadratic differences between the correspondances
		system("pause");
	}
}