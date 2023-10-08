#ifndef RANSACLIB_RANSACLIB_PROPOSED_HYBRID_RANSAC_H_
#define RANSACLIB_RANSACLIB_PROPOSED_HYBRID_RANSAC_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

#include <RansacLib/proposed_hybrid_sampling.h>
#include <RansacLib/utils.h>
#include <RansacLib/hybrid_utils.h>

namespace ransac_lib {

class PHybridRansacOptions {
 public:
     PHybridRansacOptions()
      : min_num_iterations_(100u),
        max_num_iterations_(10000u),
        success_probability_(0.9999),
        squared_inlier_threshold_(1.0),
        random_seed_(0u),
        early_model_rejection_(true) {}
  uint32_t min_num_iterations_;
  uint32_t max_num_iterations_;
  double success_probability_;
  double squared_inlier_threshold_;
  unsigned int random_seed_;
  bool early_model_rejection_;
  int inliers_init_;
};


template <class Model>
struct PHybridRansacStatistics {
  uint32_t num_iterations;
  int best_num_inliers;
  double best_model_score;
  double inlier_ratio;
  std::vector<int> inlier_indices;

  std::vector<double> inlier_ratios;
  std::vector<utils::BestModelUpdateStats> stats_after_new_best_model;

  int failed_iterations;
};

template <class Model>
class PHybridRansacBase {
 protected:
  void ResetStatistics(PHybridRansacStatistics<Model>* statistics) const {
      PHybridRansacStatistics<Model>& stats = *statistics;
    stats.best_num_inliers = 0;
    stats.best_model_score = std::numeric_limits<double>::max();
    stats.num_iterations = 0u;
    stats.inlier_ratio = 0.0;
    stats.inlier_indices.clear();
    stats.inlier_ratios.clear();
    stats.stats_after_new_best_model.clear();
    stats.failed_iterations = 0;
  }
};


template <class Model, class ModelVector, class Solver, class Sampler = ProposedHybridSampling<Solver> >
class ProposedHybridRansac : public PHybridRansacBase<Model> {
 public:
  // Estimates a model using a given solver. Notice that the solver contains all data.
  // Returns the number of inliers.
  int EstimateModel(const PHybridRansacOptions& options, const Solver& solver,
                    Model* best_model, PHybridRansacStatistics<Model>* statistics, Eigen::Quaterniond& q, Eigen::Vector3d& t, const std::vector<double>& gt_inlier_ratios) const
  {
    this->ResetStatistics(statistics); 
    PHybridRansacStatistics<Model>& stats = *statistics;

    if (options.inliers_init_ == 0)
    {
        // This way even cameras with no matches get inlier ratio 1, it si later checked. Besides, there should not be a camera with no matches.
        stats.inlier_ratios.resize(solver.get_num_of_cams(), 1.0);
    }
    else if (options.inliers_init_ == 1)
    {
        stats.inlier_ratios = gt_inlier_ratios;
    }
    else
    {
        std::cerr << "  ERROR: Unknown inlier initialization " << std::endl;
    }

    utils::check_inlier_ratios(stats.inlier_ratios, solver.get_match_ranges());

    // Sanity check: No need to run RANSAC if there are not enough data
    // points.
    const int kMinSampleSize = solver.min_sample_size();
    const int kNumData = solver.num_data();
    if (kMinSampleSize > kNumData || kMinSampleSize <= 0) 
    {
        std::cout << " Not enough data points " << std::endl;
        return 0;
    }

    // Initializes variables, etc.
    Sampler sampler(options.random_seed_, solver);
    std::mt19937 rng;
    rng.seed(options.random_seed_);

    uint32_t max_num_iterations =
        std::max(options.max_num_iterations_, options.min_num_iterations_);


    const double kSqrInlierThresh = options.squared_inlier_threshold_;

    Model best_minimal_model;
    double best_min_model_score = std::numeric_limits<double>::max();

    std::vector<int> minimal_sample(kMinSampleSize);
    ModelVector estimated_models;


    // Prepare statistics
    stats.stats_after_new_best_model.reserve(max_num_iterations);


    int failed_iterations = 0;
    // Runs sampling
    for (stats.num_iterations = 0u; stats.num_iterations < max_num_iterations;
         ++stats.num_iterations) 
    {
        std::vector<uint8_t> selected_cameras;
        sampler.Sample(&minimal_sample, stats.inlier_ratios, selected_cameras);

        Model rejected;  
        int max_count = 0;
        int num_solutions;
        bool decomposed = true;
        bool is_valid = true;

        std::vector<double> scores_in_solver; 
        std::vector< std::vector< Eigen::Matrix3d>> Hs_vector;

        int selected_solver = 0;    

        // MinimalSolver returns the number of estimated models.
        const int kNumEstimatedModels = solver.MinimalSolver(minimal_sample, &estimated_models, rejected, selected_solver, max_count, num_solutions, decomposed, is_valid, 
            scores_in_solver, Hs_vector);

        if (kNumEstimatedModels <= 0)
        {
            failed_iterations++;
            continue;
        }

        // Finds the best model among all estimated models.
        double best_local_score = std::numeric_limits<double>::max();
        int best_local_model_id = 0;
        GetBestEstimatedModelId(solver, estimated_models, kNumEstimatedModels,
                                kSqrInlierThresh, &best_local_score,
                                &best_local_model_id);

        // Updates the best model found so far.
        if (best_local_score < best_min_model_score)
        {
            // New best model estimated from inliers found. Stores this model
            // and runs local optimization.
            best_min_model_score = best_local_score;
            best_minimal_model = estimated_models[best_local_model_id];

            // Updates the best model.
            UpdateBestModel(best_min_model_score, best_minimal_model,
                &(stats.best_model_score), best_model);

            Eigen::Matrix3d R_gt(q);
            Eigen::Vector3d c_gt = t;

            Eigen::Vector3d c_est = best_model->c;
            Eigen::AngleAxisd aax(R_gt * best_model->R);
            double rot_error = aax.angle() * 180.0 / M_PI;
            double pos_error = (c_est - c_gt).norm();

            stats.stats_after_new_best_model.emplace_back(stats.num_iterations, static_cast<double>(best_min_model_score), 
                                                        static_cast<double>(rot_error), static_cast<double>(pos_error),
                                                        std::move(selected_cameras));

            if (options.inliers_init_ == 0)
            {
                stats.best_num_inliers = GetInliers(
                    solver, *best_model, kSqrInlierThresh, &(stats.inlier_indices), stats.inlier_ratios);

                // Camera without matches could have been assigned non-zero probability
                utils::check_inlier_ratios(stats.inlier_ratios, solver.get_match_ranges());
            }
            else if (options.inliers_init_ == 1)
            {
                stats.best_num_inliers = GetInliers(
                    solver, *best_model, kSqrInlierThresh, &(stats.inlier_indices));
            }


            stats.inlier_ratio = static_cast<double>(stats.best_num_inliers) /
                static_cast<double>(kNumData);
            max_num_iterations = utils::NumRequiredIterations(
                stats.inlier_ratio, 1.0 - options.success_probability_,
                kMinSampleSize, options.min_num_iterations_,
                options.max_num_iterations_);
        }
    }

    stats.failed_iterations = failed_iterations;

    return stats.best_num_inliers;
  }

 protected:
  void GetBestEstimatedModelId(const Solver& solver, const ModelVector& models,
                               const int num_models,
                               const double squared_inlier_threshold,
                               double* best_score, int* best_model_id) const {
    *best_score = std::numeric_limits<double>::max();
    *best_model_id = 0;
    for (int m = 0; m < num_models; ++m) {
      double score = std::numeric_limits<double>::max();
      ScoreModel(solver, models[m], squared_inlier_threshold, &score);

      if (score < *best_score) {
        *best_score = score;
        *best_model_id = m;
      }
    }
  }

  void ScoreModel(const Solver& solver, const Model& model,
                  const double squared_inlier_threshold, double* score) const {
    const int kNumData = solver.num_data();
    *score = 0.0;
    for (int i = 0; i < kNumData; ++i) {
      double squared_error = solver.EvaluateModelOnPoint(model, i);
      *score += ComputeScore(squared_error, squared_inlier_threshold);
    }
  }

  // MSAC (top-hat) scoring function.
  inline double ComputeScore(const double squared_error,
                             const double squared_error_threshold) const {
    return std::min(squared_error, squared_error_threshold);
  }

  int GetInliers(const Solver& solver, const Model& model,
      const double squared_inlier_threshold,
      std::vector<int>* inliers) const
  {
      const int kNumData = solver.num_data();
      int num_inliers = 0;
      if (inliers == nullptr) {
          for (int i = 0; i < kNumData; ++i)
          {
              double squared_error = solver.EvaluateModelOnPoint(model, i);
              if (squared_error < squared_inlier_threshold)
              {
                  ++num_inliers;
              }
          }
      }
      else {
          inliers->clear();
          for (int i = 0; i < kNumData; ++i) {
              double squared_error = solver.EvaluateModelOnPoint(model, i);
              if (squared_error < squared_inlier_threshold)
              {
                  ++num_inliers;
                  inliers->push_back(i);
              }
          }
      }

      return num_inliers;
  }

  // Version saving inlier ratios
  int GetInliers(const Solver& solver, const Model& model,
      const double squared_inlier_threshold,
      std::vector<int>* inliers, std::vector<double>& inlier_ratios) const
  {
      const std::vector<std::pair<int, int>>& ranges = *(solver.get_match_ranges());
      int cam = 0;

      // Setting small probability for all matches - in case there will be no inliers
      // The cameras with no matches need to have probability set zero later - RANSAC does this inside EstimateModel
      std::fill(inlier_ratios.begin(), inlier_ratios.end(), 0.0001);

      const int kNumData = solver.num_data();
      int num_inliers = 0;

      if (inliers != nullptr) 
      {
            inliers->clear();
      }
      for (int i = 0; i < kNumData; ++i)
      {
            double squared_error = solver.EvaluateModelOnPoint(model, i);
            if (squared_error < squared_inlier_threshold)
            {
                // in case some cameras have no inliers
                while (ranges[cam].second < i)
                {
                    cam++;
                }

                ++inlier_ratios[cam]; // previously set to zero
                ++num_inliers;
                if (inliers != nullptr)
                {
                    inliers->push_back(i);
                }
            }
      }

      // Divide by the number of matches from each camera
      for (size_t i = 0; i < inlier_ratios.size(); ++i)
      {
          inlier_ratios[i] /= static_cast<double>(ranges[i].second - ranges[i].first + 1);  // Not checking for zeros - cameras without matches are fixed later
      }
      return num_inliers;
  }

  

  inline void UpdateBestModel(const double score_curr, const Model& m_curr,
                              double* score_best, Model* m_best) const {
    if (score_curr < *score_best) {
      *score_best = score_curr;
      *m_best = m_curr;
    }
  }
};

}  // namespace ransac_lib

#endif  // RANSACLIB_RANSACLIB_PROPOSED_HYBRID_RANSAC_H_
