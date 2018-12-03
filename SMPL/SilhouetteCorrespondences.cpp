#include "SilhouetteCorrespondences.h"

namespace smpl
{
    Correspondences SilhouetteCorrespondencesFinder::operator()(
        const Image& input_silhouette,
        const Image& model_silhouette,
        const std::vector<float4>& model_normals,
        const std::vector<float4>& model_indices,
        const std::string& output_path) const
    {
        bool logging_on = true;
        if (output_path.empty()) logging_on = false;

        Image model_contour, input_contour;
        
        std::vector<Point<int>> model_border = 
        DetectSilhouetteBorder(model_silhouette, model_contour, true);
        DetectSilhouetteBorder(input_silhouette, input_contour, false);

        // model border points that have input correspondences
        std::vector<Point<int>> model_correspondences;
        std::vector<Point<int>> input_correspondences;
        std::vector<Point<float>> distances;

        Image log_marching(model_contour),
            log_input_normals(input_contour);

        auto stop_condition = [&input_contour](int x, int y) { return (input_contour(x, y).IsWhite()); };
        for (auto& point : model_border)
        {
            float4 normal4 = model_normals[point.y*IMAGE_WIDTH + point.x];
            Point<float> normal = Point<float>(normal4.x, -normal4.y).normalized();
            AddCorrespondence(stop_condition, point, normal, ray_distance_,
                input_silhouette, model_correspondences, input_correspondences, distances,
                logging_on, log_marching, log_input_normals);
        }

        Image log_correspondences(model_contour);
        LogCorrespondences(model_correspondences, input_correspondences, input_contour, log_correspondences);
        
        Prune(model_indices, model_correspondences, input_correspondences, distances);
        Image log_pruned(model_contour);
        LogCorrespondences(model_correspondences, input_correspondences, input_contour, log_pruned);

        if (logging_on)
        {
            input_contour.SavePNG(output_path + "input_contour.png");
            model_contour.SavePNG(output_path + "model_contour.png");
            log_correspondences.SavePNG(output_path + "correspondences.png");
            log_pruned.SavePNG(output_path + "pruned.png");

            for (int y = 0; y < IMAGE_HEIGHT; y++)
                for (int x = 0; x < IMAGE_WIDTH; x++)
                {
                    if (model_contour(x, y).IsWhite())
                    {
                        log_marching(x, y) = Pixel::Yellow();
                    }

                    if (input_contour(x, y).IsWhite())
                    {
                        log_marching(x, y) = Pixel::Red();
                    }
                }

            log_marching.SavePNG(output_path + "marching.png");
            log_input_normals.SavePNG(output_path + "input_normals.png");
        }

        return Correspondences(log_pruned, model_correspondences, input_correspondences, distances);
    }

    void SilhouetteCorrespondencesFinder::LogCorrespondences(
        const std::vector<Point<int>>& model_correspondences,
        const std::vector<Point<int>>& input_correspondences,
        const Image& input_contour,
        Image& log_correspondences) const
    {
        auto stop_on_endpoint = [](int x, int y) { return false; };
        for (int i = 0; i < model_correspondences.size(); i++)
            Bresenham(stop_on_endpoint, model_correspondences[i], 
                input_correspondences[i], log_correspondences, true);

        for (int y = 0; y < IMAGE_HEIGHT; y++)
            for (int x = 0; x < IMAGE_WIDTH; x++)
                if (input_contour(x, y).IsWhite())
                    log_correspondences(x, y) = Pixel::Yellow();
    }

    void SilhouetteCorrespondencesFinder::Prune(
        const std::vector<float4>& model_indices,
        std::vector<Point<int>>& model_correspondences,
        std::vector<Point<int>>& input_correspondences,
        std::vector<Point<float>>& distances
    ) const
    {
        const int size = static_cast<int>(model_correspondences.size());
        assert(size == input_correspondences.size());

        std::vector<int> filtered_indices;
        filtered_indices.reserve(size);

        for (int i = 0; i < size; i++)
        {
            Point<int> point = model_correspondences[i];
            float4 indices = model_indices[point.y*IMAGE_WIDTH + point.x];
            int i0 = (int)indices[0], i1 = (int)indices[1], i2 = (int)indices[2];
            bool prune_head(prune_head_);
            bool prune_crotch(prune_crotch_);
            bool prune_hand(prune_hand_);
            bool prune_feet(prune_feet_);

            if (prune_head_)
                prune_head = false;

            if (prune_crotch_ &&
                crotch_indices_.find(i0) == crotch_indices_.end() &&
                crotch_indices_.find(i1) == crotch_indices_.end() &&
                crotch_indices_.find(i2) == crotch_indices_.end())
                prune_crotch = false;

            if (prune_hand_ &&
                hand_indices_.find(i0) == hand_indices_.end() &&
                hand_indices_.find(i1) == hand_indices_.end() &&
                hand_indices_.find(i2) == hand_indices_.end())
                prune_hand = false;

            if (prune_feet_ &&
                feet_indices_.find(i0) == feet_indices_.end() &&
                feet_indices_.find(i1) == feet_indices_.end() &&
                feet_indices_.find(i2) == feet_indices_.end())
                prune_feet = false;

            if (!prune_head && !prune_crotch && !prune_hand && !prune_feet)
                filtered_indices.push_back(i);
        }

        const int new_size = (int)filtered_indices.size();

        std::vector<Point<int>> mc_filtered; mc_filtered.reserve(new_size);
        std::vector<Point<int>> ic_filtered; ic_filtered.reserve(new_size);
        std::vector<Point<float>> d_filtered; d_filtered.reserve(new_size);

        for (const auto& i : filtered_indices)
        {
            mc_filtered.push_back(model_correspondences[i]);
            ic_filtered.push_back(input_correspondences[i]);
            d_filtered.push_back(distances[i]);
        }

        model_correspondences = mc_filtered;
        input_correspondences = ic_filtered;
        distances = d_filtered;
    }

    bool SilhouetteCorrespondencesFinder::IsBorderingPixelBlack(
        const Image& image, int i, int j) const
    {
        if (image(std::max(0, i - 1), std::max(0, j - 1)).IsBlack() ||
            image(i, std::max(0, j - 1)).IsBlack() ||
            image(std::min(IMAGE_WIDTH - 1, i + 1), std::max(0, j - 1)).IsBlack() ||

            image(std::max(0, i - 1), j).IsBlack() ||
            image(std::min(IMAGE_WIDTH - 1, i + 1), j).IsBlack() ||

            image(std::max(0, i - 1), std::min(IMAGE_HEIGHT - 1, j + 1)).IsBlack() ||
            image(i, std::min(IMAGE_HEIGHT - 1, j + 1)).IsBlack() ||
            image(std::min(IMAGE_WIDTH - 1, i + 1), std::min(IMAGE_HEIGHT - 1, j + 1)).IsBlack()
            )
            return true;
        else
            return false;
    }

    std::vector<Point<int>>
    SilhouetteCorrespondencesFinder::DetectSilhouetteBorder(
        const Image& silhouette, Image& contour, bool fill_border) const
    {
        std::vector<Point<int>> border;
        if (fill_border) border.reserve(3000);

        for (int j = 0; j < IMAGE_HEIGHT; j++)
        {
            for (int i = 0; i < IMAGE_WIDTH; i++)
            {
                if (!silhouette(i, j).IsBlack())
                    if (IsBorderingPixelBlack(silhouette, i, j))
                    {
                        contour(i, j) = Pixel::White();
                        if (fill_border) border.emplace_back(i, j);
                    }
                    else contour(i, j) = Pixel::Black();
            }
        }

        return border;
    }

    Point<int> SilhouetteCorrespondencesFinder::Bresenham(
        std::function<bool(int, int)> stop_condition,
        const Point<int>& p0, const Point<int>& p1,
        Image& model, bool painted) const
    {
        if (!p1.IsDefined()) return Point<int>();

        // ensure integer coordinates
        int x0 = p0[0];
        int y0 = p0[1];
        int x1 = p1[0];
        int y1 = p1[1];

        // compute deltas and update directions
        float dx = abs(static_cast<float>(x1 - x0));
        int sx = x0 < x1 ? 1 : -1;
        float dy = -abs(static_cast<float>(y1 - y0));
        int sy = y0 < y1 ? 1 : -1;
        float err = dx + dy;
        float e2 = 0; // error value e_xy

        // set initial coordinates
        int x = x0;
        int y = y0;

        // start loop to set nPixels
        int nPixels = static_cast<int>(std::max(dx, -dy));
        for (int i = 0; i < nPixels; ++i)
        {
            if (x < 0 || y < 0 || x >= IMAGE_WIDTH || y >= IMAGE_HEIGHT) return Point<int>();
            if (painted) model(x, y) = Pixel::Blue();
            if (stop_condition(x, y)) return Point<int>(x, y);

            // update error
            e2 = 2 * err;
            // update coordinates depending on the error
            if (e2 > dy) { err += dy; x += sx; } /* e_xy+e_x > 0 */
            if (e2 < dx) { err += dx; y += sy; } /* e_xy+e_y < 0 */
        }

        return Point<int>();
    }

    std::tuple<Point<int>, Point<int>>
    SilhouetteCorrespondencesFinder::BresenhamNext(std::function<bool(int, int)> stop_condition,
            const Point<int>& p0, const Point<int>& p1,
            Image& log_marching, bool painted) const
    {
        if (!p1.IsDefined()) return std::make_tuple(Point<int>(), Point<int>());

        // ensure integer coordinates
        int x0 = p0[0];
        int y0 = p0[1];
        int x1 = p1[0];
        int y1 = p1[1];

        // compute deltas and update directions
        float dx = abs(static_cast<float>(x1 - x0));
        int sx = x0 < x1 ? 1 : -1;
        float dy = -abs(static_cast<float>(y1 - y0));
        int sy = y0 < y1 ? 1 : -1;
        float err = dx + dy;
        float e2 = 0; // error value e_xy

        // set initial coordinates
        int x = x0;
        int y = y0;

        // start loop to set nPixels
        int nPixels = static_cast<int>(std::max(dx, -dy));
        for (int i = 0; i < nPixels; ++i)
        {
            if (x < 0 || y < 0 || x >= IMAGE_WIDTH || y >= IMAGE_HEIGHT)
                return std::make_tuple(Point<int>(), Point<int>());
            if (painted) log_marching(x, y) = Pixel::Blue();

            Point<int> correspondence(x, y);

            // update error
            e2 = 2 * err;
            // update coordinates depending on the error
            if (e2 > dy) { err += dy; x += sx; } /* e_xy+e_x > 0 */
            if (e2 < dx) { err += dx; y += sy; } /* e_xy+e_y < 0 */

            if (stop_condition(correspondence.x, correspondence.y))
                return std::make_tuple(correspondence, Point<int>(x, y));
        }

        return std::make_tuple(Point<int>(), Point<int>());
    }

    std::tuple<Point<int>, Point<float>>
    SilhouetteCorrespondencesFinder::Marching(
            const std::function<bool(int, int)>& stop_condition,
            const Point<int>& model_point,
            const Point<float>& model_normal, int max_dist,
            const Image& input_silhouette,
            bool logging_on,
            Image& log_marching,
            Image& log_input_derivatives) const
    {
        Point<int> endpoint(
            static_cast<int>(model_point[0] + max_dist * model_normal[0]),
            static_cast<int>(model_point[1] + max_dist * model_normal[1]));

        Point<int> current, next(model_point);

        while (true)
        {
            std::tie(current, next) = BresenhamNext(stop_condition, next, endpoint, log_marching, logging_on);

            if (!current.IsDefined()) break;
            else
            {
                // Derivatives in FreeImage space, inverted to point out of the silhouette
                // Taking step to because the contours can have a width of 2
                float dx = (input_silhouette(current.x + hdx_, current.y).GrayScale()
                    - input_silhouette(current.x - hdx_, current.y).GrayScale()) / 2.f;
                float dy = (input_silhouette(current.x, current.y + hdx_).GrayScale()
                    - input_silhouette(current.x, current.y - hdx_).GrayScale()) / 2.f;
                Point<float> input_normal = Point<float>(-dx, -dy).normalized();

                if (logging_on)
                {
                    const int radius = 5;
                    Point<int> log_end(static_cast<int>(current.x + radius * input_normal.x),
                        static_cast<int>(current.y + radius * input_normal.y));
                    auto stop_on_endpoint = [](int x, int y) { return false; };
                    Bresenham(stop_on_endpoint, current, log_end, log_input_derivatives, true);
                }

                const float threshold = 0.2f;
                if (model_normal.dot(input_normal) > threshold)
                {
                    Point<float> distance(
                        static_cast<float>(model_point[0] - current[0]),
                        static_cast<float>(model_point[1] - current[1]));
                    return std::make_tuple(current, distance);
                }
            }
        }

        return std::make_tuple(Point<int>(), Point<float>());
    }

    void SilhouetteCorrespondencesFinder::AddCorrespondence(
        std::function<bool(int, int)> stop_condition,
        const Point<int>& model_point,
        const Point<float>& model_normal, int max_dist,
        const Image& input_silhouette,
        std::vector<Point<int>>& model_correspondence,
        std::vector<Point<int>>& input_correspondence,
        std::vector<Point<float>>& distance,
        bool logging_on,
        Image& log_marching,
        Image& log_input_normals) const
    {
        Point<int> c1, c2;		// input_point
        Point<float> d1, d2;	// distances

        std::tie(c1, d1) = Marching(stop_condition, model_point, model_normal,
            max_dist, input_silhouette, logging_on, log_marching, log_input_normals);
        std::tie(c2, d2) = Marching(stop_condition, model_point, model_normal,
            -max_dist, input_silhouette, logging_on, log_marching, log_input_normals);

        if (!c1.IsDefined() && !c2.IsDefined()) return;
        if (c1.IsDefined() && !c2.IsDefined())
        {
            model_correspondence.push_back(model_point);
            input_correspondence.push_back(c1);
            distance.push_back(d1);
        }
        if (!c1.IsDefined() && c2.IsDefined())
        {
            model_correspondence.push_back(model_point);
            input_correspondence.push_back(c2);
            distance.push_back(d2);
        }
        if (d1.Norm() < d2.Norm())
        {
            model_correspondence.push_back(model_point);
            input_correspondence.push_back(c1);
            distance.push_back(d1);
        }
        else
        {
            model_correspondence.push_back(model_point);
            input_correspondence.push_back(c2);
            distance.push_back(d2);
        }
    }
}