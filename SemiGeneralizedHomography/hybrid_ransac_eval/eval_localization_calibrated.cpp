#define _USE_MATH_DEFINES

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

#include <RansacLib/proposed_hybrid_ransac.h>
#include <RansacLib/inlier_sampling.h>
#include <RansacLib/sampling.h>
#include <RansacLib/inlier_sampling_two_cameras.h>
#include <RansacLib/random_sampling_two_cameras.h>

#include <ransac_solvers/hybrid_H50_ransac_solver.h>
#include <ransac_solvers/ransac_solvers_common.h>

#include "eval_utils.h"
#include "data_handling.h"


template<class GenHomography, class GenHomographies, class sampler>
int run_ransac(const ransac_lib::PHybridRansacOptions& options, const solvers::HybridH50RansacSolver& solver,
                ransac_lib::PHybridRansacStatistics<GenHomography>& ransac_stats, GenHomography& best_model,
                int kNumMatches, Eigen::Quaterniond& q, Eigen::Vector3d& t, const std::vector<double>& gt_inlier_ratios,
                std::chrono::duration<double>& elapsed_seconds)
{
    // Create RANSAC
    ransac_lib::ProposedHybridRansac<GenHomography, GenHomographies, solvers::HybridH50RansacSolver, sampler> ransac;

    auto ransac_start = std::chrono::system_clock::now();

    int num_ransac_inliers = 0;
    if (kNumMatches >= 10) 
    {
        num_ransac_inliers = ransac.EstimateModel(options, solver, &best_model, &ransac_stats, q, t, gt_inlier_ratios);
    }

    auto ransac_end = std::chrono::system_clock::now();
    elapsed_seconds = ransac_end - ransac_start;

    return num_ransac_inliers;
}

int main(int argc, char** argv) 
{
    //_CrtSetDbgFlag(_CRTDBG_CHECK_ALWAYS_DF);
    using solvers::GenHomography;
    using solvers::GenHomographies;
    using namespace ransac_lib::utils;

    std::cout << " usage: " << argv[0] << " query_list colmap_dir match_dir "
        << "pairs_file inlier_threshold_2D outfile solver_type num_iters sampler_type early_model_rejection " << std::endl;

    std::cout << argc;

    if (argc < 18) 
        return -1;


    std::string experiment = argv[11];  // This could be enum
    int experiment_param = atoi(argv[12]);
    double min_inl = atof(argv[13]);
    double max_inl = atof(argv[14]);
    double noise = atof(argv[15]);
    bool modify_data = (bool)atoi(argv[16]);

    int inliers_init = atoi(argv[17]);  // 0 = default, 1 = oracle
    int solver_type = atoi(argv[7]); // 0 = hybrid_solver, 1 = sH5_3
    int sampler_type = atoi(argv[9]); // 0 = hybrid_sampling, 1 = random_sampling, 2 = inlier_sampling
    int early_model_rejection = atoi(argv[10]); // 0 = false, 1 = true


    std::cout << argv[3] << " Modifying data " << (bool)modify_data << "   Using solver " << solver_type << "   Using sampler " << sampler_type << std::endl;
    

    Queries query_data;
    std::vector<ColmapImage> images;
    std::unordered_map<std::string, int> map_image_name_to_idx;
    std::unordered_map<int, int> map_cam_id_to_idx;
    pairs_u_map pairs;
    solvers::Cameras h_cameras;
    std::vector<ColmapCamera> cameras;

    std::string match_dir(argv[3]);

    const double kErrorThresh = atof(argv[5]);

    if (LoadData(query_data, images, map_image_name_to_idx, map_cam_id_to_idx, pairs, h_cameras, cameras, argv) == -1)
        return -1;

    const int kNumQuery = static_cast<int>(query_data.size());
    std::cout << " Found " << kNumQuery << " query images " << std::endl;
    
    std::vector<double> orientation_errors, position_errors;
    std::vector<double> min_scores, best_m_scores;

    std::vector<std::vector<ransac_lib::utils::BestModelUpdateStats>> stats_for_each_query(kNumQuery);
    std::vector<uint32_t> n_iterations(kNumQuery);

    // Random engine to shuffle inlier ratios
    auto rng = std::default_random_engine{};
    rng.seed(42);

    double average_ransac_time = 0.0;
    int num_7scenes_threshold = 0;
    double camera_threshold = 0.0;
    if (argc >= 11) {
        camera_threshold = atof(argv[10]);
    }

    uint32_t max_n_iterations = 0;

    int failed = 0;
    int rejected_count = 0;
    int not_decomposed_count = 0;
    int not_valid_count = 0;
    int eq_count = 0;
    int eq_nv_count = 0;
    int eq_nd_count = 0;
    int eq_nt_count = 0;
    int best_had_worse_score_count = 0;

    // Run RANSAC for each querry
    std::cout << argv[3] << " starts the queries" << std::endl;
    for (int i = 0; i < kNumQuery; ++i)
    {
        if (i % 50 == 0)
            std::cout << experiment << " " << experiment_param << " " << min_inl << " " << max_inl << " " << noise << " " << 
                " " << solver_type<<" "<<sampler_type << " at query " << i << "/" << kNumQuery << std::endl;

        solvers::Matches2D2DWithCamID matches2D2D;
        int num_valid = 0;
        std::vector<std::pair<int, int>> match_ranges;
        double q_fx = query_data[i].camera.parameters[0];
        double q_fy = q_fx;

        std::vector<std::pair< Eigen::Vector2d, Eigen::Vector2d>> points2D_q_db;
        const std::vector<std::string>& pairs_i = pairs[query_data[i].name];

        std::vector<double> inlier_ratios(pairs_i.size());

        // Set up the inlier ratios based on the selected experiment
        if (experiment == "dist")
        {
            std::fill(inlier_ratios.begin(), inlier_ratios.end(), min_inl);

            // get camera distances
            std::vector<size_t> dist_indexes;
            GetCameraDistances(dist_indexes, pairs_i, query_data[i].name, map_image_name_to_idx, map_cam_id_to_idx, images, h_cameras);

            // set camera at experiment_param-th distance from query camera (and the next one) to max_inl, all other cameras are min_inl
            // assert experiment_param < size - 1
            inlier_ratios[dist_indexes[experiment_param]] = max_inl;
            inlier_ratios[dist_indexes[experiment_param + 1]] = max_inl;
        }
        else if (experiment == "uniform")
        {
            // do python's linspace
            double d = (max_inl - min_inl) / (double)(pairs_i.size() - 1);
            for (size_t j = 0; j < pairs_i.size(); ++j)
            {
                inlier_ratios[j] = min_inl + d * j;
            }

            // shuffle the vector to create random assignment of uniformly distributed inlier ratios 
            std::shuffle(std::begin(inlier_ratios), std::end(inlier_ratios), rng);
        }
        else if (experiment == "dist_uniform")
        {
            // get camera distances
            std::vector<size_t> dist_indexes;
            GetCameraDistances(dist_indexes, pairs_i, query_data[i].name, map_image_name_to_idx, map_cam_id_to_idx, images, h_cameras);

            // experiment_param denotes whether the smallest inlier ratio should be assigned to closest or the farthest camera
            bool reverse = experiment_param;
            if (reverse)
                std::reverse(std::begin(dist_indexes), std::end(dist_indexes));

            // do python's linspace
            double d = (max_inl - min_inl) / (double)(pairs_i.size() - 1);
            for (size_t j = 0; j < pairs_i.size(); ++j)
            {
                inlier_ratios[dist_indexes[j]] = min_inl + d * j;
            }
        }
        else if (experiment == "good_bad")
        {
            // do python's linspace
            std::fill(inlier_ratios.begin(), inlier_ratios.end(), min_inl);

            for (size_t j = 0; j < experiment_param; ++j)
            {
                inlier_ratios[j] = max_inl;
            }

            // shuffle the vector to create random assignment of uniformly distributed inlier ratios 
            std::shuffle(std::begin(inlier_ratios), std::end(inlier_ratios), rng);
        }
        else
        {
            std::cerr << "  ERROR: Unknown experiment " << std::endl;
        }


        std::vector<int> activated_cams;
        std::vector<int> selected_images;
        size_t idx = 0;
        for (const std::string& p : pairs_i)
        {
            Points2D points2d_q;
            Points2D points2d_db;
            if (!LoadMatches2D2D(match_dir, query_data[i].name, p, &points2d_q, &points2d_db)) 
            {
                std::cerr << "  ERROR: Could not load matches for " << query_data[i].name
                    << " and " << p << std::endl;
                continue;
            }

            const int kImgIdx = map_image_name_to_idx[p];
            const int kCamIdx = map_cam_id_to_idx[images[kImgIdx].camera_id];

            // add noise and outliers
            if (modify_data)
                ModifyMatches(matches2D2D, noise, inlier_ratios[idx++], rng, points2d_q, points2d_db);

            PrepareMatches(match_ranges, matches2D2D, selected_images, points2d_q, points2d_db, points2D_q_db,
                query_data[i], cameras[kCamIdx], h_cameras[kImgIdx], q_fx, q_fy, kErrorThresh, kImgIdx, num_valid);
        }

        const int kNumMatches = static_cast<int>(matches2D2D.size());

        if (kNumMatches < 5 || num_valid < 2) 
        {
            orientation_errors.push_back(std::numeric_limits<double>::max());
            position_errors.push_back(std::numeric_limits<double>::max());
            continue;
        }


        ransac_lib::PHybridRansacOptions options;
        options.min_num_iterations_ = static_cast<uint32_t>(atoi(argv[8]));
        options.max_num_iterations_ = static_cast<uint32_t>(atoi(argv[8]));
        options.squared_inlier_threshold_ = kErrorThresh * kErrorThresh;
        options.inliers_init_ = inliers_init;

        std::random_device rand_dev;
        options.random_seed_ = rand_dev();

        // Assuming single focal length!
        solvers::HybridH50RansacSolver solver(q_fx, q_fy, h_cameras, matches2D2D,
            options.squared_inlier_threshold_, match_ranges, solver_type, early_model_rejection);

        /// Create variables for RANSAC
        ransac_lib::PHybridRansacStatistics<GenHomography> ransac_stats;

        GenHomography best_model;
        std::chrono::duration<double> elapsed_seconds;
        int num_ransac_inliers;

        // Run the RANSAC with the selected sampler and solver
        if (sampler_type == 1) //  1 = random_sampling
        {
            if (solver_type == 0)   // 0 = hybrid_solver
                num_ransac_inliers = run_ransac<GenHomography, GenHomographies, ransac_lib::UniformSampling<solvers::HybridH50RansacSolver> >
                                        (options, solver, ransac_stats, best_model, kNumMatches, query_data[i].q, query_data[i].t, inlier_ratios, elapsed_seconds);

            else if (solver_type == 1)  // 1 = sH5_3
                num_ransac_inliers = run_ransac<GenHomography, GenHomographies, ransac_lib::RandomSamplingFromTwo<solvers::HybridH50RansacSolver> >
                                        (options, solver, ransac_stats, best_model, kNumMatches, query_data[i].q, query_data[i].t, inlier_ratios, elapsed_seconds);
        }
        else if (sampler_type == 2) // 2 = inlier_sampling
        {
            if (solver_type == 0)   // 0 = hybrid_solver
                num_ransac_inliers = run_ransac<GenHomography, GenHomographies, ransac_lib::InlierSampling<solvers::HybridH50RansacSolver> >
                                        (options, solver, ransac_stats, best_model, kNumMatches, query_data[i].q, query_data[i].t, inlier_ratios, elapsed_seconds);

            else if (solver_type == 1)  // 1 = sH5_3
                num_ransac_inliers = run_ransac<GenHomography, GenHomographies, ransac_lib::InlierSamplingFromTwo<solvers::HybridH50RansacSolver> >
                                        (options, solver, ransac_stats, best_model, kNumMatches, query_data[i].q, query_data[i].t, inlier_ratios, elapsed_seconds);
        }
        else    // default ....... (sampler_type == 0)       // 0 = hybrid_sampling
        {
            num_ransac_inliers = run_ransac<GenHomography, GenHomographies, ransac_lib::ProposedHybridSampling<solvers::HybridH50RansacSolver> >
                (options, solver, ransac_stats, best_model, kNumMatches, query_data[i].q, query_data[i].t, inlier_ratios, elapsed_seconds);
        }


        GenHomography rejected_model;
        rejected_model = best_model;

        report_progress(ransac_stats, num_ransac_inliers, elapsed_seconds,
            average_ransac_time, kNumQuery, best_model, query_data[i],
            orientation_errors, position_errors, "RANSAC", rejected_model);

        if ((position_errors.back() < 0.05) && (orientation_errors.back() < 5.0)) 
            ++num_7scenes_threshold;

        if (ransac_stats.num_iterations > max_n_iterations) 
            max_n_iterations = ransac_stats.num_iterations;

        n_iterations[i] = ransac_stats.num_iterations;
        failed += ransac_stats.failed_iterations;
        stats_for_each_query[i] = std::move(ransac_stats.stats_after_new_best_model);
    }

    // Report stats after finishing all the querries
    std::cout << argv[3] << " Finished all the queries, failed iterations: " << failed << std::endl;
    
    std::cout << " rejected_count " << rejected_count << std::endl;
    std::cout << " not_decomposed_count " << not_decomposed_count << std::endl;
    std::cout << " not_valid_count " << not_valid_count << std::endl;
    std::cout << " eq_count " << eq_count << std::endl;
    std::cout << " eq_nv_count " << eq_nv_count << std::endl;
    std::cout << " eq_nd_count " << eq_nd_count << std::endl;
    std::cout << " eq_not_transposed_count " << eq_nt_count << std::endl;
    std::cout << " best_had_worse_score_count " << best_had_worse_score_count << std::endl;

    std::cout << " kNumQuery " << kNumQuery << std::endl;

    output_stats_from_each_query(stats_for_each_query, max_n_iterations, argv[6]);

    std::cout << std::endl << std::endl;
    std::cout << "/////////////////////" << std::endl;
    std::cout << "// Statistics" << std::endl;

    double mean_pos, mean_orient;
    double median_pos = ComputeMedian(&position_errors, &mean_pos);
    double median_orient = ComputeMedian(&orientation_errors, &mean_orient);


    std::cout << "// average RANSAC time: " << average_ransac_time << std::endl;
    std::cout << std::endl << std::endl;
    std::cout << "// median position error: " << median_pos << std::endl;
    std::cout << "// median orientation error: " << median_orient << std::endl;
    std::cout << "// mean position error: " << mean_pos << std::endl;
    std::cout << "// mean orientation error: " << mean_orient << std::endl;
    std::cout << std::endl << std::endl;


    double ratio = 100.0 * static_cast<double>(num_7scenes_threshold) / static_cast<double>(kNumQuery);
    std::cout << "// within 7 Scenes threshold: " << ratio << " % " << std::endl;
    std::ofstream ofs(argv[6], std::ios::out);
    if (!ofs.is_open()) {
        std::cerr << " ERROR: Cannot write to " << argv[6] << std::endl;
        return -1;
    }
    ofs << median_pos << " / " << mean_pos << " & " << median_orient << " / " << mean_orient << " & " << average_ransac_time << " & " << ratio << std::endl;
    ofs.close();

    return 0;
}












