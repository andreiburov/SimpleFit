#include <catch.hpp>
#include <SMPL.h>

using namespace smpl;

TEST_CASE("Print Configuration")
{
	Reconstruction::Configuration configuration;
	configuration.iterations_ = 10;
	configuration.output_path_ = "JointsSyntheticShape0";
	configuration.translation_ = { 3, 2, 1 } ;
	configuration.Dump("Model/reconstruction.json");
}

TEST_CASE("Load Configuration")
{
	Reconstruction::Configuration configuration("Model");
	std::cout << configuration.output_path_ << std::endl;
}

struct Configuration
{
	bool with_normals;
	std::string output_path;
	float precision;

	Configuration() {}

	Configuration(
		bool with_normals,
		std::string output_path,
		float precision
	) :
		with_normals(with_normals),
		output_path(output_path),
		precision(precision)
	{}

	template<class Archive>
	void Serialize(Archive& archive)
	{
		archive(
			CEREAL_NVP(with_normals),
			CEREAL_NVP(output_path),
			CEREAL_NVP(precision)
		);
	}

	void Dump(const std::string& path)
	{
		std::ofstream out(path);
		cereal::JSONOutputArchive archive(out);

		Serialize(archive);
	}
};

//TEST_CASE("Print Configuration")
//{
//	Configuration configuration(true, "hello", 0.5f);
//
//	/*std::ofstream out(std::string("reconstruction.json"));
//	cereal::JSONOutputArchive archive(out);
//	configuration.Serialize(archive);*/
//	configuration.Dump("reconstruction.json");
//}

TEST_CASE("Jacobian Joints From Shape")
{
	Generator generator(Generator::Configuration(std::string("../Model")));
	Projector projector(Projector::Configuration(std::string("../Model")));
	SilhouetteEnergy silhouette_optimizer(generator, projector);

	Eigen::Vector3f translation(0.f, 0.2f, 4.0f);
	ShapeCoefficients input_betas;
	PoseEulerCoefficients input_thetas;
	input_betas[0] = -1.f;
	input_betas[1] = -5.f;
	Body input_body = generator(input_betas, input_thetas);
	Silhouette input_silhouette = silhouette_optimizer.Infer("", translation, input_betas, input_thetas);
	input_silhouette.GetImage().SavePNG("input_silhouette.png");

	ShapeCoefficients model_betas;
	PoseEulerCoefficients model_thetas;
	Body model_body = generator(model_betas, model_thetas);
	Silhouette model_silhouette = silhouette_optimizer.Infer("", translation, model_betas, model_thetas);
	model_silhouette.GetImage().SavePNG("model_silhouette.png");

	Correspondences correspondences = silhouette_optimizer.FindCorrespondences(input_silhouette.GetImage(),
		model_silhouette.GetImage(), model_silhouette.GetNormals());

	std::vector<float3> dshape(VERTEX_COUNT * BETA_COUNT);
	generator.ComputeBodyFromShapeJacobian(dshape);

	const int residuals = static_cast<int>(correspondences.input_border.size() * 2);
	Eigen::MatrixXf dsillhouette_shape(residuals, BETA_COUNT);
	silhouette_optimizer.ComputeSilhouetteFromShapeJacobian(model_body, dshape, translation,
		model_silhouette, correspondences, residuals, dsillhouette_shape);

	// print model silhouette border before the update
	{
		Image image;
		for (auto& p : correspondences.model_border)
		{
			image(p.x, p.y) = BLUE;
		}
		image.SavePNG("model_border_before.png");
	}

	for (int m = 0; m < residuals; m += 2)
	{
		float x = static_cast<float>(correspondences.model_border[m / 2].x);
		float y = static_cast<float>(correspondences.model_border[m / 2].y);

		for (int j = 0; j < BETA_COUNT; j++)
		{
			x += input_betas[j] * dsillhouette_shape(m, j);
			y += input_betas[j] * dsillhouette_shape(m + 1, j);
		}

		correspondences.model_border[m / 2].x = static_cast<int>(x);
		correspondences.model_border[m / 2].y = static_cast<int>(y);
	}

	// print model silhouette border after the update
	{
		Image image;
		for (auto& p : correspondences.model_border)
		{
			image(p.x, p.y) = BLUE;
		}
		image.SavePNG("model_border_after.png");
	}
}