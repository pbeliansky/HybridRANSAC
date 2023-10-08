#include "hybrid_H50_ransac_solver.h"

#include <solvers/sh5_2/cpp/sh5_2.h>
#include <solvers/sh5_3/cpp/sh5_3_cf.h>


namespace solvers 
{
    HybridH50RansacSolver::HybridH50RansacSolver(double fx, double fy, const Cameras& cameras,
        const Matches2D2DWithCamID& matches,
        double squared_threshold, const std::vector<std::pair<int, int>>& match_ranges, int solver_type, bool early_model_rejection) // 0 = hybrid, 1 = sH5_3
    {
        f_x_ = fx;
        f_y_ = fy;
        matches_ = matches;
        cameras_ = cameras;
        num_matches_ = static_cast<int>(matches_.size());
        num_cameras_ = static_cast<int>(cameras_.size());
        squared_threshold_ = squared_threshold;
        solver_type_ = solver_type;
        early_model_rejection_ = early_model_rejection;

        // Used for sampling
        match_ranges_ = match_ranges;
    }

    int HybridH50RansacSolver::MinimalSolver(const std::vector<int>& sample,
        GenHomographies* poses, GenHomography& rejected, int& selected_solver, int& max_count, int& num_solutions, bool& decomposed, bool& is_valid, 
        std::vector<double>& scores, std::vector< std::vector< Eigen::Matrix3d>>& Hs_vector) const
    {
        poses->clear();

        std::unordered_multiset<int> activated_cameras;


        // ...todo
        Eigen::Matrix<double, 3, 5> q(3, 5), p(3, 5), c(3, 5);
        for (int i = 0; i < 5; ++i) 
        {
            q.col(i) = matches_[sample[i]].p2D_query.homogeneous();
            p.col(i) = matches_[sample[i]].ref_ray_dir;
            p.col(i).normalize();
            c.col(i) = cameras_[matches_[sample[i]].camera_id].c;
            activated_cameras.insert(matches_[sample[i]].camera_id);
        }


        std::vector<Eigen::Matrix3d> homographies_local;
        std::vector<Eigen::Vector3d> N_homographies_local;

        // Choosing the right solver
        if (solver_type_ == 0) // Hybrid solver
        {
            for (int i = 0; i < 5; ++i)
            {
                if (activated_cameras.count(matches_[sample[i]].camera_id) > max_count)
                    max_count = activated_cameras.count(matches_[sample[i]].camera_id);
            }

            if (max_count <= 2)
            {
                sh5_2::solver_sturm(q, p, c, &homographies_local, &N_homographies_local);
                selected_solver = 2;
            }
            else if (max_count == 3)
            {
                sh5_3::solver_closed_form(q, p, c, &homographies_local, &N_homographies_local);
                selected_solver = 3;
            }
            else
            {
                // std::cerr << " Unsuitable configuration ... " << max_count << std::endl;
                return -1;
            }
        }
        else if (solver_type_ == 1) // sH5_3
        {
            sh5_3::solver_closed_form(q, p, c, &homographies_local, &N_homographies_local);
            selected_solver = 3;
        }

        num_solutions = static_cast<int>(N_homographies_local.size());
        if (num_solutions == 0) 
            return 0;

        GenHomographies local_poses;

        if (early_model_rejection_) // The early rejection didn't work very well in our RANSAC, we don't use it in the experiments
        {
            int best_solution = 0;
            double best_score = std::numeric_limits<double>::max();
            std::vector<Eigen::Matrix3d> homographies;
            std::vector<Eigen::Vector3d> N_homographies;

            for (int i = 0; i < num_solutions; ++i) {
                double score = 0.0;
                for (int j = 0; j < 5; ++j) {
                    const int kIdx = sample[j];
                    const int kCamIdx = matches_[kIdx].camera_id;
                    Eigen::Matrix3d H;
                    H = cameras_[kCamIdx].K * cameras_[kCamIdx].R * (homographies_local[i] + cameras_[kCamIdx].c * N_homographies_local[i].transpose());
                    score += ((H * matches_[kIdx].p2D_query.homogeneous()).hnormalized() - matches_[kIdx].p2D_db).squaredNorm();
                }
                if (score < best_score) {
                    best_solution = i;
                    best_score = score;
                }
            }

            N_homographies.push_back(N_homographies_local[best_solution]);
            homographies.push_back(homographies_local[best_solution]);

            num_solutions = static_cast<int>(N_homographies.size());

            best_solution = -1;
            best_score = std::numeric_limits<double>::max();

            for (int i = 0; i < num_solutions; ++i) // num_solutions is always 1
            {
                GenHomography H_gen;
                H_gen.H = homographies[i];
                H_gen.N = N_homographies[i];

                rejected = H_gen;

                if (!DecomposeGenHomography(sample, &H_gen))
                {

                    rejected = H_gen;
                    decomposed = false;
                    continue;
                }
                double score = 0.0;
                bool valid = true;
                for (int j = 0; j < 5; ++j) {
                    double error = EvaluateModelOnPoint(H_gen, sample[j]);
                    if (error >= squared_threshold_) {
                        valid = false;
                        break;
                    }
                    score += error;
                }

                if (!valid)
                {
                    is_valid = false;
                    continue;
                }

                if (score < best_score) {
                    best_score = score;
                    best_solution = static_cast<int>(local_poses.size());
                }

                local_poses.push_back(H_gen);
            }

            if (best_solution >= 0) {
                poses->push_back(local_poses[best_solution]);
            }
        }
        else
        {
            scores.clear();
            for (int i = 0; i < num_solutions; ++i)
            {
                // compute score for each model
                double score = 0.0;
                std::vector< Eigen::Matrix3d> Hs;

                for (int j = 0; j < 5; ++j) {
                    const int kIdx = sample[j];
                    const int kCamIdx = matches_[kIdx].camera_id;
                    Eigen::Matrix3d H;
                    H = cameras_[kCamIdx].K * cameras_[kCamIdx].R * (homographies_local[i] + cameras_[kCamIdx].c * N_homographies_local[i].transpose());
                    score += ((H * matches_[kIdx].p2D_query.homogeneous()).hnormalized() - matches_[kIdx].p2D_db).squaredNorm();
                    Hs.push_back(H);
                }

                GenHomography H_gen;
                H_gen.H = homographies_local[i];
                H_gen.N = N_homographies_local[i]; 

                std::vector<Eigen::Matrix3d> Rs;
                std::vector<Eigen::Vector3d> ts;
                std::vector<Eigen::Vector3d> ns;

                Eigen::Vector3d N = H_gen.N;
                double N_norm = N.norm();
                N.normalize();
                
                solvers::DecomposeHomography(H_gen.H, &Rs, &ts, &ns);

                for (size_t j = 0; j < Rs.size(); ++j)
                {
                    GenHomography Hg = H_gen;

                    Hg.R = Rs[j];
                    Hg.c = ts[j] / N_norm;
                    
                    Hg.H = Hg.R + Hg.c * Hg.N.transpose();

                    poses->push_back(Hg);
                    scores.push_back(score);
                    Hs_vector.push_back(Hs);
                }
            }
        }        

        return static_cast<int>(poses->size());
    }


    // Evaluates the line on the i-th data point.
    double HybridH50RansacSolver::EvaluateModelOnPoint(const GenHomography& pose, int i) const 
    {
        double T_error = ComputeTriangulationError(pose, i);
        return T_error;
    }


    
    // Taken from RansacLib
    bool DecomposeHomography(const Eigen::Matrix3d& H,
        std::vector<Eigen::Matrix3d>* Rs,
        std::vector<Eigen::Vector3d>* ts,
        std::vector<Eigen::Vector3d>* ns) {
        Rs->clear();
        ts->clear();
        ns->clear();
        // The following code is based on The Robotics Toolbox for Matlab (RTB) and
        // the corresponding copyright note is replicated below.
        // Copyright (C) 1993-2011, by Peter I. Corke
        //
        // This file is part of The Robotics Toolbox for Matlab (RTB).
        //
        // RTB is free software: you can redistribute it and/or modify
        // it under the terms of the GNU Lesser General Public License as published by
        // the Free Software Foundation, either version 3 of the License, or
        // (at your option) any later version.
        //
        // RTB is distributed in the hope that it will be useful,
        // but WITHOUT ANY WARRANTY; without even the implied warranty of
        // MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        // GNU Lesser General Public License for more details.
        //
        // You should have received a copy of the GNU Leser General Public License
        // along with RTB.  If not, see <http://www.gnu.org/licenses/>.

        // normalize H so that the second singular value is one
        Eigen::JacobiSVD<Eigen::Matrix3d> svd1(H);
        Eigen::Matrix3d H2 = H / svd1.singularValues()[1];

        // compute the SVD of the symmetric matrix H'*H = VSV'
        Eigen::JacobiSVD<Eigen::Matrix3d> svd2(H2.transpose() * H2,
            Eigen::ComputeFullU | Eigen::ComputeFullV); //Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix3d V = svd2.matrixV();

        // ensure V is right-handed
        if (V.determinant() < 0.0) V *= -1.0;

        // get the squared singular values
        Eigen::Vector3d S = svd2.singularValues();
        double s1 = S[0];
        double s3 = S[2];

        Eigen::Vector3d v1 = V.col(0);
        Eigen::Vector3d v2 = V.col(1);
        Eigen::Vector3d v3 = V.col(2);

        // pure the case of pure rotation all the singular values are equal to 1
        if (fabs(s1 - s3) < 1e-14) {
            return false;
        }
        else {
            // compute orthogonal unit vectors
            Eigen::Vector3d u1 = (sqrt(1.0 - s3) * v1 + sqrt(s1 - 1.0) * v3) / sqrt(s1 - s3);
            Eigen::Vector3d u2 = (sqrt(1.0 - s3) * v1 - sqrt(s1 - 1.0) * v3) / sqrt(s1 - s3);

            Eigen::Matrix3d U1, W1, U2, W2;
            U1.col(0) = v2;
            U1.col(1) = u1;
            U1.col(2) = v2.cross(u1);

            W1.col(0) = H2 * v2;
            W1.col(1) = H2 * u1;
            W1.col(2) = (H2 * v2).cross(H2 * u1);

            U2.col(0) = v2;
            U2.col(1) = u2;
            U2.col(2) = v2.cross(u2);

            W2.col(0) = H2 * v2;
            W2.col(1) = H2 * u2;
            W2.col(2) = (H2 * v2).cross(H2 * u2);

            // compute the rotation matrices
            Eigen::Matrix3d R1 = W1 * U1.transpose();
            Eigen::Matrix3d R2 = W2 * U2.transpose();

            // build the solutions, discard those with negative plane normals
            // Compare to the original code, we do not invert the transformation.
            // Furthermore, we multiply t with -1.
            Eigen::Vector3d n = v2.cross(u1);
            ns->push_back(n);
            Rs->push_back(R1);
            Eigen::Vector3d t = -(H2 - R1) * n;
            ts->push_back(t);

            ns->push_back(-n);
            t = (H2 - R1) * n;
            Rs->push_back(R1);
            ts->push_back(t);


            n = v2.cross(u2);
            ns->push_back(n);
            t = -(H2 - R2) * n;
            Rs->push_back(R2);
            ts->push_back(t);

            ns->push_back(-n);
            t = (H2 - R2) * n;
            ts->push_back(t);
            Rs->push_back(R2);
        }
        return true;
    }


    // Taken from RansacLib
    bool HybridH50RansacSolver::DecomposeGenHomography(const std::vector<int>& inlier_ids, GenHomography* pose) const
    {
        // Finds the pose with most inliers.
        std::vector<Eigen::Matrix3d> Rs;
        std::vector<Eigen::Vector3d> ts;
        std::vector<Eigen::Vector3d> ns;
        DecomposeHomography(pose->H, &Rs, &ts, &ns);

        // int num_solutions = static_cast<int>(rotations.size());
        int num_solutions = static_cast<int>(Rs.size());
        if (num_solutions == 0) return false;

        const int kNumInliers = static_cast<int>(inlier_ids.size());

        int best_num_consistent = 0;
        double best_score = static_cast<double>(kNumInliers) * squared_threshold_; //std::numeric_limits<double>::max();
        int best_pose = -1;

        Eigen::Vector3d N = pose->N;
        double N_norm = N.norm();
        N.normalize();
        
        for (int i = 0; i < num_solutions; ++i) {
            if (Rs[i].determinant() < 0.0) {
                continue;
            }
            Eigen::Vector3d c = ts[i] / N_norm;

            int num_consistent = 0;
            // Performs a chirality check.
            double score = 0.0;
            for (int j = 0; j < kNumInliers; ++j) {
                const int kIdx = inlier_ids[j];
                const int kCamIdx = matches_[kIdx].camera_id;
                // Computes the closest point on one line to the second one, then checks
                // if the point is in front of both cameras.
                // See https://en.wikipedia.org/wiki/Skew_lines#Nearest_Points for details.
                // Eigen::Vector3d p1 = translations[i];
                Eigen::Vector3d p1 = c;
                Eigen::Vector3d d1 = Rs[i] * matches_[kIdx].p2D_query.homogeneous();
                d1.normalize();
                Eigen::Vector3d p2 = cameras_[kCamIdx].c;
                Eigen::Vector3d d2 = matches_[kIdx].ref_ray_dir;
                d2.normalize();

                Eigen::Vector3d n1 = d1.cross(d2);
                n1.normalize();
                Eigen::Vector3d n2 = d2.cross(n1);
                n2.normalize();

                Eigen::Vector3d c1 = p1 + (p2 - p1).dot(n2) / (d1.dot(n2)) * d1;

                double error = squared_threshold_;

                if ((d1.dot(c1 - p1) > 0.0) && (d2.dot(c1 - p2) > 0.0)) {
                    Eigen::Vector3d p = (cameras_[kCamIdx].K * cameras_[kCamIdx].R * (c1 - cameras_[kCamIdx].c));
                    if (p[2] > 0.0) {
                        error = std::min((p.hnormalized() - matches_[kIdx].p2D_db).squaredNorm(), error);
                        ++num_consistent;
                    }
                }

                score += error;
            }

            // if (num_consistent > best_num_consistent) {
            if (score < best_score) {
                best_num_consistent = num_consistent;
                best_pose = i;
                best_score = score;
            }
        }
        if (best_pose < 0) {
            pose->R = Eigen::Matrix3d::Identity();
            pose->c = Eigen::Vector3d(0.0, 0.0, 0.0);
            pose->N = pose->c;
        }
        else {
            pose->R = Rs[best_pose];
            pose->c = ts[best_pose] / N_norm;
        }
        pose->H = pose->R + pose->c * pose->N.transpose();
        return (best_pose >= 0);
    }

}  // namespace solvers
