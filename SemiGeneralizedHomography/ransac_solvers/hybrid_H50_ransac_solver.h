#ifndef HYBRID_H50_H50_RANSAC_SOLVER_H_
#define HYBRID_H50_H50_RANSAC_SOLVER_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include "ransac_solvers_common.h"

namespace solvers {
    bool DecomposeHomography(const Eigen::Matrix3d& H,
        std::vector<Eigen::Matrix3d>* Rs,
        std::vector<Eigen::Vector3d>* ts,
        std::vector<Eigen::Vector3d>* ns);

    // Wraps the H50 solver into a solver class that can be used by RANSAC lib.
    class HybridH50RansacSolver {
    public:
        HybridH50RansacSolver(double fx, double fy, const Cameras& cameras,
            const Matches2D2DWithCamID& matches,
            double squared_threshold, const std::vector<std::pair<int, int>>& match_ranges, int solver_type = 0, bool early_model_rejection = true);

        inline int min_sample_size() const { return 5; }

        inline int non_minimal_sample_size() const { return 10; }

        inline int num_data() const { return num_matches_; }

        virtual int MinimalSolver(const std::vector<int>& sample, GenHomographies* poses, 
            GenHomography& rejected, int& selected_solver, int& max_count, int& num_solutions, bool& decomposed, bool& is_valid, 
            std::vector<double>& scores, std::vector< std::vector< Eigen::Matrix3d>>& Hs_vector) const;

        // Evaluates the pose on the i-th data point.
        double EvaluateModelOnPoint(const GenHomography& pose, int i) const;

        int NonMinimalSolver(const std::vector<int>& sample, GenHomography* pose) const
        {
            std::cerr << " NonMinimalSolver - Not implemented !!!" << std::endl;
            return 0;
        }

        // Linear least squares solver.
        void LeastSquares(const std::vector<int>& sample, GenHomography* pose) const
        {
            std::cerr << " LeastSquares - Not implemented !!!" << std::endl;
            return;
        }

        bool DecomposeGenHomography(const std::vector<int>& inlier_ids,
            GenHomography* pose) const;


        // Taken from RansacLib
        inline double ComputeTriangulationError(const GenHomography& pose,
            int i) const {
            const int kCamIdx = matches_[i].camera_id;
            // Computes the closest point on one line to the second one, then checks
            // if the point is in front of both cameras.
            // See https://en.wikipedia.org/wiki/Skew_lines#Nearest_Points for details.
            Eigen::Vector3d p1 = pose.c;
            Eigen::Vector3d d1 = pose.R * matches_[i].p2D_query.homogeneous();
            d1.normalize();
            Eigen::Vector3d p2 = cameras_[kCamIdx].c;
            Eigen::Vector3d d2 = matches_[i].ref_ray_dir;
            d2.normalize();

            Eigen::Vector3d n = d1.cross(d2);
            n.normalize();
            Eigen::Vector3d n1 = d1.cross(n);
            n1.normalize();
            Eigen::Vector3d n2 = d2.cross(n);
            n2.normalize();

            Eigen::Vector3d c1 = p1 + (p2 - p1).dot(n2) / (d1.dot(n2)) * d1;

            Eigen::Vector3d c2 = p2 + (p1 - p2).dot(n1) / (d2.dot(n1)) * d2;

            double error = std::numeric_limits<double>::max();

            if ((d1.dot(c1 - p1) < 0.0) || (d2.dot(c2 - p2) < 0.0)) {
                return std::numeric_limits<double>::max();
            }


            Eigen::Vector3d p = cameras_[kCamIdx].K * cameras_[kCamIdx].R * (c1 - cameras_[kCamIdx].c);
            if (p[2] < 0.0) return std::numeric_limits<double>::max();
            error = (p.hnormalized() - matches_[i].p2D_db).squaredNorm();
            p = pose.R.transpose() * (c2 - pose.c);
            if (p[2] < 0.0) return std::numeric_limits<double>::max();
            Eigen::Vector2d p_img = p.hnormalized() - matches_[i].p2D_query;
            p_img[0] *= f_x_;
            p_img[1] *= f_y_;
            error += p_img.squaredNorm();

            return error / 4.0;  // squared mean error.
        }

        inline void copy_match_ranges(std::vector<std::pair<int, int>>* match_ranges) const 
        {
            *match_ranges = match_ranges_;
        }

        inline size_t get_num_of_cams() const
        {
            return match_ranges_.size();
        }

        const std::vector<std::pair<int, int>>* get_match_ranges() const
        {
            return &match_ranges_;
        }


    protected:
        double f_x_;
        double f_y_;
        Matches2D2DWithCamID matches_;
        Cameras cameras_;
        int num_matches_;
        int num_cameras_;
        double squared_threshold_;
        int solver_type_;
        bool early_model_rejection_;

        std::vector<std::pair<int, int>> match_ranges_;
    };

}  // namespace solvers

#endif  // HYBRID_H50_H50_RANSAC_SOLVER_H_
