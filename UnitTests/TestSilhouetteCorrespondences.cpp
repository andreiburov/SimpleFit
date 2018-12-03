#include <catch.hpp>
#include <SMPL.h>
#include "Common.h"

using namespace smpl;

TEST_CASE("Find Correspondences")
{
	TestConfiguration test;
	int ray_dist, pdx;

	SECTION("0") { test = TestConfiguration("Confs/find_correspondences0.json"); ray_dist = 35; pdx = 5; }
	SECTION("1") { test = TestConfiguration("Confs/find_correspondences1.json"); ray_dist = 50; pdx = 5; }
	SECTION("2") { test = TestConfiguration("Confs/find_correspondences2.json"); ray_dist = 65; pdx = 5; }

	Generator generator(Generator::Configuration(test.model_path));
	Projector projector(Projector::Configuration(test.model_path));
	SilhouetteRenderer renderer(generator(true));
	SilhouetteEnergy silhouette_energy(generator, projector, renderer, ray_dist, pdx);

	Eigen::Vector3f translation(test.translation);
	ShapeCoefficients input_betas(test.betas), betas;
	PoseEulerCoefficients input_thetas(test.thetas), thetas;

	Silhouette input_silhouette = silhouette_energy.Infer(translation, input_betas, input_thetas);
	Silhouette silhouette = silhouette_energy.Infer(translation, betas, thetas);

	Correspondences correspondences = silhouette_energy
		.FindCorrespondences(input_silhouette.GetImage(), silhouette.GetImage(), silhouette.GetNormals());

	correspondences.image.SavePNG(test.output_path + "correspondences.png");
}

TEST_CASE("Prune Correspondences")
{
	TestConfiguration test;
	int ray_dist, pdx;

	SECTION("0") { test = TestConfiguration("Confs/find_correspondences0.json"); ray_dist = 35; pdx = 5; }
	SECTION("1") { test = TestConfiguration("Confs/find_correspondences1.json"); ray_dist = 50; pdx = 5; }
	SECTION("2") { test = TestConfiguration("Confs/find_correspondences2.json"); ray_dist = 65; pdx = 5; }

	Generator generator(Generator::Configuration(test.model_path));
	Projector projector(Projector::Configuration(test.model_path));
	SilhouetteRenderer renderer(generator(true));
	SilhouetteEnergy silhouette_energy(generator, projector, renderer, ray_dist, pdx);

	Eigen::Vector3f translation(test.translation);
	ShapeCoefficients input_betas(test.betas), betas;
	PoseEulerCoefficients input_thetas(test.thetas), thetas;

	Silhouette input_silhouette = silhouette_energy.Infer(translation, input_betas, input_thetas);
	Silhouette silhouette = silhouette_energy.Infer(translation, betas, thetas);

	Correspondences correspondences = silhouette_energy
		.FindCorrespondences(input_silhouette.GetImage(), silhouette.GetImage(), silhouette.GetNormals());
	
	correspondences.image.SavePNG(test.output_path + "correspondences.png");
	silhouette_energy.PruneCorrepondences(input_silhouette.GetImage(), 
		silhouette.GetImage(), silhouette.GetNormals(), correspondences);
	correspondences.image.SavePNG(test.output_path + "pruned.png");
}

TEST_CASE("Find Correspondences By Normals")
{
	TestConfiguration test;
	int ray_dist, pdx;

	SECTION("0") { test = TestConfiguration("Confs/find_correspondences0.json"); ray_dist = 70; pdx = 5; }
	/*SECTION("1") { test = TestConfiguration("Confs/find_correspondences1.json"); ray_dist = 50; pdx = 5; }
	SECTION("2") { test = TestConfiguration("Confs/find_correspondences2.json"); ray_dist = 65; pdx = 5; }*/

	Generator generator(Generator::Configuration(test.model_path));
	Projector projector(Projector::Configuration(test.model_path));
	SilhouetteRenderer renderer(generator(true));
	SilhouetteEnergy silhouette_energy(generator, projector, renderer, ray_dist, pdx);

	Eigen::Vector3f translation(test.translation);
	ShapeCoefficients input_betas(test.betas), betas;
	PoseEulerCoefficients input_thetas(test.thetas), thetas;

	Silhouette input_silhouette = silhouette_energy.Infer(translation, input_betas, input_thetas);
	Silhouette silhouette = silhouette_energy.Infer(translation, betas, thetas);

	Correspondences correspondences = silhouette_energy
		.FindCorrespondencesPruned(input_silhouette.GetImage(), silhouette.GetImage(), silhouette.GetNormals(), test.output_path);
}