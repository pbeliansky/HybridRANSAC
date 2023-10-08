#ifndef RANSACLIB_RANSACLIB_PROPOSED_HYBRID_SAMPLING_H_
#define RANSACLIB_RANSACLIB_PROPOSED_HYBRID_SAMPLING_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <random>
#include <vector>

#include <random>


#include <RansacLib/hybrid_utils.h>

namespace ransac_lib {

    // Implements uniform sampling for RANSAC.
    template <class Solver>
    class ProposedHybridSampling {
    public:
        ProposedHybridSampling(const unsigned int random_seed, const Solver& solver)
            : num_data_(solver.num_data()), sample_size_(solver.min_sample_size()) 
        {
            rng_.seed(random_seed);
            solver.copy_match_ranges(&data_ranges_); 
        }

        // Basic strategy to start with
        // Choosing cameras based on the inlier ratios. We are choosing 5 cameras, i-th camera with probability P[i].
        // We can modify P when we choose 2 or 3 matches from the same camera, as choosing another match from the same camera will reduce the number of possible solvers.
        void Sample(std::vector<int>* random_sample, const std::vector<double>& inlier_ratios, std::vector<uint8_t>& selected_cams)
        {

            std::vector<int> cams(inlier_ratios.size());
            std::vector<double> P(inlier_ratios);

            selected_cams.resize(5);
            int last_max = 0;

            for (size_t i = 0; i < 5; ++i)
            {
                auto max_it = std::max_element(cams.begin(), cams.end());

                if (*max_it < 2)
                {
                    selected_cams[i] = ChooseCam(P);
                    cams[selected_cams[i]]++;
                }
                else if (*max_it == 2)
                {
                    if (last_max != 2)
                    {
                        last_max = 2;
                        /////////   Update probabilities, using Priors etc.
                        /////////    P = 
                    }
                    selected_cams[i] = ChooseCam(P);
                    cams[selected_cams[i]]++;
                }
                else if (*max_it == 3)
                {
                    if (last_max != 3)
                    {
                        last_max = 3;
                        P[std::distance(cams.begin(), max_it)] = 0.0;
                        /////////   Update probabilities, using Priors etc.
                        /////////    P = 
                    }

                    selected_cams[i] = ChooseCam(P);
                    cams[selected_cams[i]]++;
                }
            }
            random_sample->clear();
            random_sample->reserve(5);

            for (size_t i = 0; i < cams.size(); ++i)
            {
                if (cams[i] == 0)
                    continue;
                DrawHybridSample(cams[i], data_ranges_[i].first, data_ranges_[i].second, random_sample);
            }

        }


        // Choosing camera based on probabilities in P
        int ChooseCam(const std::vector<double>& P)
        {
            double sum_P = std::accumulate(P.begin(), P.end(), 0.0);

            real_distr_.param(std::uniform_real_distribution<double>::param_type(0, sum_P));

            const double kProb = real_distr_(rng_);
            double current_prob = 0.0;
            for (int i = 0; i < P.size(); ++i) {
                if (P[i] == 0.0) continue;

                current_prob += P[i];
                if (kProb <= current_prob) return i;
            }
            return -1;
        }

    protected:
        // Draws a minimal sample of size sample_size. Function doesn't clear random_sample vector
        void DrawHybridSample(int sample_size, int start, int end, std::vector<int>* random_sample)
        {
            std::vector<int>& sample = *random_sample;
            uniform_dstr_.param(
                std::uniform_int_distribution<int>::param_type(start, end));

            for (int i = 0; i < sample_size; ++i)
            {
                sample.push_back(-1);

                bool found = true;
                while (found)
                {
                    found = false;
                    sample.back() = uniform_dstr_(rng_);

                    for (int j = 0; j < sample.size() - 1; ++j)
                    {
                        if (sample[j] == sample.back())
                        {
                            found = true;
                            break;
                        }
                    }
                }
            }
        }

        // The random number generator used by RANSAC.
        std::mt19937_64 rng_;
        std::uniform_int_distribution<int> uniform_dstr_;
        // The number of data points.
        int num_data_;
        // The size of a sample.
        int sample_size_;

        std::vector<std::pair<int, int>> data_ranges_;
        std::uniform_real_distribution<double> real_distr_;
    };
}  // namespace ransac_lib

#endif  // RANSACLIB_RANSACLIB_PROPOSED_HYBRID_SAMPLING_H_
