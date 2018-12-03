#pragma once
#include "Definitions.h"
#include "Utils.h"
#include "Image.h"
#include <fstream>
#include <tuple>
#include <set>
#include <cereal/cereal.hpp>
#include <cereal/archives/json.hpp>

namespace smpl
{
	struct Correspondences
	{
		Correspondences(
			const Image& image, const std::vector<Point<int>>& model_border,
			const std::vector<Point<int>>& input_border, std::vector<Point<float>>&  distance) :
			image(image), model_border(model_border),
			input_border(input_border), distance(distance)
		{
		}

		Image image;
		std::vector<Point<int>> model_border;
		std::vector<Point<int>> input_border;
		std::vector<Point<float>> distance;
	};

	class SilhouetteCorrespondencesFinder
	{
		public:

        struct Configuration
        {
            Configuration(const std::string& model_path)
            {
                const std::string config_filename(model_path + "silhouette_correspondences.json");
                std::ifstream config_file(config_filename);
                if (!config_file)
                    MessageBoxA(NULL, std::string("File not found: ")
                    .append(config_filename).c_str(), "Error", MB_OK);
                cereal::JSONInputArchive archive(config_file);

                Serialize(archive);

                if (prune_crotch)
                {
                    const std::string crotch_filename(model_path + "crotch_indices.txt");
                    std::ifstream crotch_file(crotch_filename);
                    if (!crotch_file)
                        MessageBoxA(NULL, std::string("File not found: ")
                        .append(crotch_filename).c_str(), "Error", MB_OK);
         
                    int i;
                    while (crotch_file >> i) crotch_indices.emplace(i);
                }

                if (prune_hand)
                {
                    const std::string hand_filename(model_path + "hand_indices.txt");
                    std::ifstream hand_file(hand_filename);
                    if (!hand_file)
                        MessageBoxA(NULL, std::string("File not found: ")
                            .append(hand_filename).c_str(), "Error", MB_OK);

                    int i;
                    while (hand_file >> i) hand_indices.emplace(i);
                }

                if (prune_feet)
                {
                    const std::string feet_filename(model_path + "feet_indices.txt");
                    std::ifstream feet_file(feet_filename);
                    if (!feet_file)
                        MessageBoxA(NULL, std::string("File not found: ")
                            .append(feet_filename).c_str(), "Error", MB_OK);

                    int i;
                    while (feet_file >> i) feet_indices.emplace(i);
                }
            }

            template<class Archive>
            void Serialize(Archive& archive)
            {
                archive(
                    CEREAL_NVP(ray_distance),
                    CEREAL_NVP(input_derivative_half_dx),
                    CEREAL_NVP(prune_head),
                    CEREAL_NVP(prune_crotch),
                    CEREAL_NVP(prune_hand),
                    CEREAL_NVP(prune_feet)
                );
            }

            int ray_distance;
            int input_derivative_half_dx;
            bool prune_head;
            bool prune_crotch;
            bool prune_hand;
            bool prune_feet;
            std::set<int> crotch_indices;
            std::set<int> hand_indices;
            std::set<int> feet_indices;
        };

        SilhouetteCorrespondencesFinder(const Configuration& conf) :
            crotch_indices_(std::move(conf.crotch_indices)),
            hand_indices_(std::move(conf.hand_indices)),
            feet_indices_(std::move(conf.feet_indices)),
            ray_distance_(conf.ray_distance), hdx_(conf.input_derivative_half_dx),
            prune_head_(conf.prune_head), prune_crotch_(conf.prune_crotch),
            prune_hand_(conf.prune_hand), prune_feet_(conf.prune_feet)
        {
        }

        Correspondences operator()(
            const Image& input,
            const Image& model,
            const std::vector<float4>& model_normals,
            const std::vector<float4>& model_indices,
            const std::string& output_path) const;

        private:

        bool IsBorderingPixelBlack(
            const Image& image, int i, int j) const;

        std::vector<Point<int>> DetectSilhouetteBorder(
            const Image& silhouette, Image& contour, bool fill_border) const;

        void LogCorrespondences(
            const std::vector<Point<int>>& model_correspondences,
            const std::vector<Point<int>>& input_correspondences,
            const Image& input_contour,
            Image& log_correspondences) const;

        void Prune(
            const std::vector<float4>& model_indices,
            std::vector<Point<int>>& model_correspondences,
            std::vector<Point<int>>& input_correspondences,
            std::vector<Point<float>>& distances
        ) const;

        Point<int> Bresenham(
            std::function<bool(int, int)> stop_condition,
            const Point<int>& p0, const Point<int>& p1,
            Image& model, bool painted) const;

        std::tuple<Point<int>, Point<int>>
        BresenhamNext(
            std::function<bool(int, int)> stop_condition,
            const Point<int>& p0, const Point<int>& p1,
            Image& log_marching, bool painted) const;

        std::tuple<Point<int>, Point<float>>
        Marching(
            const std::function<bool(int, int)>& stop_condition,
            const Point<int>& model_point,
            const Point<float>& model_normal, int max_dist,
            const Image& input_silhouette,
            bool logging_on,
            Image& log_marching,
            Image& log_input_derivatives) const;

        void AddCorrespondence(
            std::function<bool(int, int)> stop_condition,
            const Point<int>& model_point,
            const Point<float>& model_normal, int max_dist,
            const Image& input_silhouette,
            std::vector<Point<int>>& model_correspondence,
            std::vector<Point<int>>& input_correspondence,
            std::vector<Point<float>>& distance,
            bool logging_on,
            Image& log_marching,
            Image& log_input_derivatives) const;

        const int ray_distance_;
        const int hdx_;
        const bool prune_head_;
        const bool prune_crotch_;
        const bool prune_hand_;
        const bool prune_feet_;
        const std::set<int> crotch_indices_;
        const std::set<int> hand_indices_;
        const std::set<int> feet_indices_;
	};
}