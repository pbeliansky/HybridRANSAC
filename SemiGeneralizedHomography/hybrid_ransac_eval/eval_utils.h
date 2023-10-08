#ifndef RANSACLIB_RANSACLIB_EVAL_UTILS_H_
#define RANSACLIB_RANSACLIB_EVAL_UTILS_H_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <regex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <ransac_solvers/ransac_solvers_common.h>
#include <hybrid_ransac_eval/data_handling.h>


namespace ransac_lib {
    namespace utils
    {
        template <typename S = ransac_lib::PHybridRansacStatistics<solvers::GenHomography>>
        void report_progress(S& ransac_stats, int num_ransac_inliers, std::chrono::duration<double>& elapsed_seconds,
            double& average_ransac_time, int kNumQuery, solvers::GenHomography& best_model, QueryData& query_data,
            std::vector<double>& orientation_errors, std::vector<double>& position_errors, std::string name, solvers::GenHomography& rejected_model)
        {
            // Compute the errors
            average_ransac_time += elapsed_seconds.count() / static_cast<double>(kNumQuery);
            Eigen::Vector3d N = best_model.N;
            Eigen::Matrix3d R_gt(query_data.q);
            Eigen::Vector3d c_gt = query_data.t; 

            Eigen::Vector3d c_est = best_model.c;
            Eigen::AngleAxisd aax(R_gt * best_model.R);
            double rot_error = aax.angle() * 180.0 / M_PI; 

            orientation_errors.push_back(rot_error);

            double pos_error = (c_est - c_gt).norm();
            position_errors.push_back(pos_error);
        }


        void accumulate_scores(std::vector<double>& avg_scores, std::vector<double>& scores, std::vector<size_t>& counts)
        {
            // assert matching size
            for (size_t i = 0; i < scores.size(); ++i)
            {
                avg_scores[i] += scores[i];
                counts[i]++;
            }
        }

        void concatenate_scores(std::vector<std::vector<double>>& con_scores, std::vector<double>& scores)
        {
            // assert matching size
            scores.resize(con_scores.size(), scores.back());

            for (size_t i = 0; i < scores.size(); ++i)
            {
                con_scores[i].push_back(scores[i]);
            }
        }

        void mean_median_scores(std::vector<std::vector<double>>& con_scores, std::vector<double>& mean_scores, std::vector<double>& median_scores)
        {
            for (size_t i = 0; i < con_scores.size(); ++i)
            {
                if (con_scores[i].size() > 0)
                {
                    median_scores[i] = ComputeMedian(&(con_scores[i]), &(mean_scores[i]));
                }
            }
        }

        void average_scores(std::vector<double>& avg_scores, std::vector<size_t>& counts)
        {
            // assert matching size
            for (size_t i = 0; i < avg_scores.size(); ++i)
            {
                avg_scores[i] /= counts[i];
            }
        }

        void output_scores(std::vector<double>& scores, std::string name, char* arg)
        {
            std::ofstream File(arg + name + ".txt");

            for (auto&& s : scores)
                File << s << "\n";

            File.close();
        }


        void output_stats_for_1_iteration_queries(std::vector<double>& orientation_errors, std::vector<double>& position_errors,
            std::vector<double>& min_scores, std::vector<double>& best_m_scores, char* arg)
        {
            output_scores(orientation_errors, "_orientation_errors_", arg);

            output_scores(position_errors, "_position_errors_", arg);

            output_scores(min_scores, "_min_scores_", arg);

            output_scores(best_m_scores, "_best_m_scores_", arg);
        }


        void output_stats_from_each_query(std::vector<std::vector<ransac_lib::utils::BestModelUpdateStats>>& stats, uint32_t max_iteration, char* arg)
        {
            uint32_t num_of_queries = stats.size();
            std::vector<uint32_t> current_idx(num_of_queries, 0);

            std::vector<double> scores(num_of_queries);
            std::vector<double> mean_scores(max_iteration);
            std::vector<double> median_scores(max_iteration);

            std::vector<double> orientation_errors(num_of_queries);
            std::vector<double> mean_orientation_errors(max_iteration);
            std::vector<double> median_orientation_errors(max_iteration);

            std::vector<double> position_errors(num_of_queries);
            std::vector<double> mean_position_errors(max_iteration);
            std::vector<double> median_position_errors(max_iteration);

            // Go through all iterations and compute mean and median
            for (uint32_t i = 0; i < max_iteration; ++i)
            {
                if (i % 100 == 0)
                    std::cout << "i:   " << i << "   max_iteration:   " << max_iteration << std::endl;

                // Get current stats for iteration i for each of the queries
                for (int q = 0; q < num_of_queries; ++q)  // kNumQuery is type int
                {
                    if (stats[q].size())
                    {
                        // Index out of bounds
                        if (current_idx[q] >= stats[q].size())
                            std::cerr << " ERROR: (current_idx[q] >= stats[q].size()),  q:   " << q << "   stats.size():   " << stats.size() << "   stats[q].size():   " << stats[q].size() << std::endl;

                        // If the current index is of the last best model found for query q
                        else if (current_idx[q] + 1 == stats[q].size())
                        {
                            scores[q] = stats[q][current_idx[q]].ransac_score;
                            orientation_errors[q] = stats[q][current_idx[q]].orientation_error;
                            position_errors[q] = stats[q][current_idx[q]].position_error;
                        }

                        // Since there are another best models after this one, we can chceck whether the next one doesn't start at this iteration
                        else if (stats[q][current_idx[q] + 1].iteration <= i)
                        {
                            scores[q] = stats[q][++current_idx[q]].ransac_score;
                            orientation_errors[q] = stats[q][current_idx[q]].orientation_error;
                            position_errors[q] = stats[q][current_idx[q]].position_error;
                        }

                        // There are another best models after this one, but they don't start at this iteration
                        else
                        {
                            scores[q] = stats[q][current_idx[q]].ransac_score;
                            orientation_errors[q] = stats[q][current_idx[q]].orientation_error;
                            position_errors[q] = stats[q][current_idx[q]].position_error;
                        }
                    }
                }

                median_scores[i] = ComputeMedian(&(scores), &(mean_scores[i]));
                median_orientation_errors[i] = ComputeMedian(&(orientation_errors), &(mean_orientation_errors[i]));
                median_position_errors[i] = ComputeMedian(&(position_errors), &(mean_position_errors[i]));
            }


            output_scores(mean_scores, "_mean_scores_", arg);
            output_scores(median_scores, "_median_scores_", arg);

            output_scores(mean_orientation_errors, "_mean_orientation_errors_", arg);
            output_scores(median_orientation_errors, "_median_orientation_errors_", arg);

            output_scores(mean_position_errors, "_mean_position_errors_", arg);
            output_scores(median_position_errors, "_median_position_errors_", arg);
        }
    }  // namespace utils
}  // namespace ransac_lib

#endif  // RANSACLIB_RANSACLIB_EVAL_UTILS_H_