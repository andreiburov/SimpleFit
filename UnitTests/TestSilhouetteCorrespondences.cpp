//#include <direct.h>  
//#include <stdlib.h>
//#include <thread>
#include <catch.hpp>
#include <SMPL.h>

using namespace smpl;

TEST_CASE("Find Correspondences")
{
	Generator generator(Generator::Configuration(std::string("../Model")));
	Projector projector(Projector::Configuration(std::string("../Model")));
	SilhouetteRenderer renderer(generator(true));
	SilhouetteEnergy silhouette_energy(generator, projector, renderer, 35, 5);

	Eigen::Vector3f translation(0.f, 0.2f, 4.0f);
	ShapeCoefficients input_betas;
	PoseEulerCoefficients input_thetas;
	input_betas << std::string("-1 -5");
	Silhouette input = silhouette_energy.Infer(translation, input_betas, input_thetas);

	ShapeCoefficients model_betas;
	PoseEulerCoefficients model_thetas;
	Silhouette model = silhouette_energy.Infer(translation, model_betas, model_thetas);

	Correspondences correspondences = silhouette_energy.FindCorrespondences(input.GetImage(), model.GetImage(), model.GetNormals());

	Image test("TestSilhouettes/correspondences.png");
	REQUIRE(correspondences.image == test);
}

TEST_CASE("Prune Correspondences")
{
	Generator generator(Generator::Configuration(std::string("../Model")));
	Projector projector(Projector::Configuration(std::string("../Model")));
	SilhouetteRenderer renderer(generator(true));
	SilhouetteEnergy silhouette_energy(generator, projector, renderer, 35, 5);

	Eigen::Vector3f translation(0.f, 0.2f, 4.0f);
	ShapeCoefficients input_betas;
	PoseEulerCoefficients input_thetas;
	input_thetas[SHOULDER_RIGHT].z = 1.f;
	Silhouette input = silhouette_energy.Infer(translation, input_betas, input_thetas);
	input.GetImage().SavePNG("input.png");

	ShapeCoefficients model_betas;
	PoseEulerCoefficients model_thetas;
	Silhouette model = silhouette_energy.Infer(translation, model_betas, model_thetas);
	input.GetImage().SavePNG("model.png");

	Correspondences correspondences = silhouette_energy.FindCorrespondences(input.GetImage(), model.GetImage(), model.GetNormals());
	correspondences.image.SavePNG("pure_correspondences.png");
	silhouette_energy.PruneCorrepondences(input.GetImage(), model.GetImage(), model.GetNormals(), correspondences);
	correspondences.image.SavePNG("pruned_correspondences.png");
}