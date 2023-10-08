#ifndef RANSACLIB_RANSACLIB_HYBRID_UTILS_H_
#define RANSACLIB_RANSACLIB_HYBRID_UTILS_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>
#include <unordered_set>

namespace ransac_lib {
    namespace utils 
    {
        /// If there are cameras with no matches, the range for this camera would be (i, i-1), in this case, we set inlier ratio to 0
        template <typename T>
        void check_inlier_ratios(std::vector<T>& inlier_ratios, const std::vector<std::pair<int, int>>* ranges)
        {
            double sum_inlier_ratios = 0.0;
            for (size_t i = 0; i < ranges->size(); ++i)
            {
                if ((*ranges)[i].first > (*ranges)[i].second)
                    inlier_ratios[i] = 0.0;
                sum_inlier_ratios += inlier_ratios[i];
            }

            if (sum_inlier_ratios == 0.0)
                std::cerr << " ERROR: All inlier ratios are zero " << std::endl;
        }


        template <typename Model>
        struct RejectedModelStats
        {
            uint32_t iteration;
            std::vector<int> minimal_sample;
            Model rejected_model;
            Model best_model;
            double best_score;
            std::vector<uint8_t> selected_cameras;
            int selected_solver;

            // These are to be filled later in eval script
            // points 
            // gt model ... homography or R t
            // etc

            Eigen::Matrix<double, 2, 5> q_points, db_points, q_pps, db_pps;

            Eigen::Matrix<double, 3, 5> q, p, c;
            std::unordered_multiset<int> activated_cameras;

            Eigen::Matrix3d R_gt;
            Eigen::Vector3d c_gt;

            Eigen::Matrix3d R_est;
            Eigen::Vector3d c_est;

            int max_count, num_solutions;
            bool decomposed, is_valid;

            std::vector<Model> estimated_models;

            bool best_had_worse_score;
            double score_of_best;
            double min_score;

            RejectedModelStats(uint32_t it, std::vector<int> ms, Model&& rm, Model bm, double bs, std::vector<uint8_t>&& sc, int ss, int max_c, int num_s, bool dc, bool iv, 
                std::vector<Model> em, bool bhws, double sob, double mins)
                : iteration(it), minimal_sample(std::move(ms)), rejected_model(std::move(rm)), best_model(std::move(bm)),
                  best_score(bs), selected_cameras(std::move(sc)), selected_solver(ss),
                  max_count(max_c), num_solutions(num_s), decomposed(dc), is_valid(iv), estimated_models(em), best_had_worse_score(bhws),
                  score_of_best(sob), min_score(mins) {}
        };


        // added for proposed hybrid ransac
        struct BestModelUpdateStats
        {
            uint32_t iteration;
            float ransac_score;
            float orientation_error;
            float position_error;
            //add inlier_ratios
            std::vector<uint8_t> selected_cameras;

            BestModelUpdateStats(uint32_t it, float rs, float oe, float pe, std::vector<uint8_t>&& sc)
                : iteration(it), ransac_score(rs), orientation_error(oe), position_error(pe), selected_cameras(std::move(sc)) {}
        };
    }  // namespace utils
}  // namespace ransac_lib

#endif  // RANSACLIB_RANSACLIB_HYBRID_UTILS_H_