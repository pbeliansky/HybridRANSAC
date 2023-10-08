// Copyright (c) 2020, Torsten Sattler
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of Torsten Sattler nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// author: Torsten Sattler, torsten.sattler.de@googlemail.com

#ifndef RANSAC_SOLVERS_COMMON_H_
#define RANSAC_SOLVERS_COMMON_H_

#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

//#include <ceres/ceres.h>

namespace solvers {

struct GenRelativePoseF {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  double focal;
};
typedef std::vector<GenRelativePoseF, Eigen::aligned_allocator<GenRelativePoseF>> GenRelativePosesF;

// Common definitions such as 2D-3D and 2D-2D matches.
struct Match2D3D {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector2d p2D;
  Eigen::Vector3d p3D;
};
typedef std::vector<Match2D3D, Eigen::aligned_allocator<Match2D3D>> Matches2D3D;

struct Match2D2D {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Represented by a 2D point in the query image and a 3D ray and ray origin
  // for the 2D point in the reference image.
  Eigen::Vector2d p2D_query;
  Eigen::Vector3d ref_ray_dir;
  Eigen::Vector3d ref_ray_orig;
};
typedef std::vector<Match2D2D, Eigen::aligned_allocator<Match2D2D>> Matches2D2D;

struct GenHomography {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix3d H;
  Eigen::Vector3d N;
  double focal;

  // Individual homographies, one for each camera in the generalized camera.
  // The homographies transform from the query image into the cameras of the
  // generalized camera.
  std::vector<Eigen::Matrix3d> homographies;

  // Individual essential matrices.
  std::vector<Eigen::Matrix3d> essentials;

  // Pose of the pinhole camera, transforms from the pinhole to the generalized
  // camera coordinates.
  Eigen::Matrix3d R;
  Eigen::Vector3d c;
};
typedef std::vector<GenHomography, Eigen::aligned_allocator<GenHomography>> GenHomographies;

struct Match2D2DWithCamID {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Represented by a 2D point in the query image and a 3D ray and ray origin
  // for the 2D point in the reference image. The origin of the ray is the center
  // of the camera in the generalized camera coordinate system.
  Eigen::Vector2d p2D_query;
  Eigen::Vector3d ref_ray_dir;
  // 2D feature position in the generalized camera cameras.
  Eigen::Vector2d p2D_db;
  int camera_id;
};
typedef std::vector<Match2D2DWithCamID, Eigen::aligned_allocator<Match2D2DWithCamID>> Matches2D2DWithCamID;

struct Camera {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // The orientation and position of the camera with respect to the generalized
  // camera.
  Eigen::Matrix3d R;
  Eigen::Vector3d c;
  Eigen::Matrix3d K;
};
typedef std::vector<Camera, Eigen::aligned_allocator<Camera>> Cameras;


// Non-linear optimization.
struct ReprojectionError {
  ReprojectionError(double x, double y, double x_db, double y_db,
                    const Eigen::Matrix3d& K_, const Eigen::Matrix3d& R_,
                    const Eigen::Vector3d& c_)
      : p2D_q_x(x),
        p2D_q_y(y),
        p2D_db_x(x_db),
        p2D_db_y(y_db) {
    K = K_;
    R = R_;
    c = c_;
  }

  // template <typename T>
  // bool operator()(const T* const data, T* residuals) const {
  //   // The last three entries correspond to the N part of the homography.
  //   Eigen::Matrix<T, 3, 1> N;
  //   N[0] = data[9];
  //   N[1] = data[10];
  //   N[2] = data[11];
  //   N /= N.norm();
  //   T intercept = data[12];

  //   Eigen::Matrix<T, 3, 3> H;
  //   H << data[0], data[1], data[2], data[3], data[4], data[5], data[6],
  //        data[7], data[8];

  //   Eigen::Matrix<T, 3, 3> H_cam = K.template cast<T>() * R.template cast<T>() * (H + c.template cast<T>() * N.transpose()*intercept);
  //   Eigen::Matrix<T, 3, 1> p;
  //   p << static_cast<T>(p2D_q_x), static_cast<T>(p2D_q_y), static_cast<T>(1.0);
  //   Eigen::Matrix<T, 3, 1> p2 = H_cam * p;

  //   T x_proj = p2[0] / p2[2];
  //   T y_proj = p2[1] / p2[2];

  //   residuals[0] = static_cast<T>(p2D_db_x) - x_proj;
  //   residuals[1] = static_cast<T>(p2D_db_y) - y_proj;

  //   return true;
  // }

  template <typename T>
  bool operator()(const T* const data, T* residuals) const {
    // The first four entries encode the rotation matrix as a quaternion, the
    // next three entries correspond to the translation, the last three entries
    // correspond to N.
    Eigen::Quaternion<T> q(data[0], data[1], data[2], data[3]);
    q.normalize();
    Eigen::Matrix<T, 3, 3> R_H(q);
    Eigen::Matrix<T, 3, 1> t;
    t << data[4], data[5], data[6];

    Eigen::Matrix<T, 3, 1> N;
    N << data[7], data[8], data[9];

    // Recreates the homography.
    Eigen::Matrix<T, 3, 3> H;
    H = R_H + t * N.transpose();

    Eigen::Matrix<T, 3, 3> H_cam = K.template cast<T>() * R.template cast<T>() * (H + c.template cast<T>() * N.transpose());
    Eigen::Matrix<T, 3, 1> p;
    p << static_cast<T>(p2D_q_x), static_cast<T>(p2D_q_y), static_cast<T>(1.0);
    Eigen::Matrix<T, 3, 1> p2 = H_cam * p;

    T x_proj = p2[0] / p2[2];
    T y_proj = p2[1] / p2[2];

    residuals[0] = static_cast<T>(p2D_db_x) - x_proj;
    residuals[1] = static_cast<T>(p2D_db_y) - y_proj;

    return true;
  }

  //// Factory function
  //static ceres::CostFunction* CreateCost(const double x, const double y,
  //                                       const double x_db, const double y_db,
  //                                       const Eigen::Matrix3d& K_,
  //                                       const Eigen::Matrix3d& R_,
  //                                       const Eigen::Vector3d& c_) {
  //  return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 10>(
  //      new ReprojectionError(x, y, x_db, y_db, K_, R_, c_)));
  //}

  // Assumes that the measurement is centered around the principal point.
  double p2D_q_x;
  double p2D_q_y;
  double p2D_db_x;
  double p2D_db_y;
  Eigen::Matrix3d K;
  Eigen::Matrix3d R;
  Eigen::Vector3d c;
};

struct ReprojectionTriangulationError {
  ReprojectionTriangulationError(double x, double y, double x_db, double y_db,
                                 // const Eigen::Matrix3d& K_,
                                 const Eigen::Matrix3d& R_,
                                 const Eigen::Vector3d& c_)
      : p2D_q_x(x),
        p2D_q_y(y),
        p2D_db_x(x_db),
        p2D_db_y(y_db) {
    // K = K_;
    R = R_;
    c = c_;
  }

  template <typename T>
  bool operator()(const T* const data, T* residuals) const {
    // The first four entries encode the rotation matrix as a quaternion, the
    // next three entries correspond to the translation, the last three entries
    // correspond to N.
    Eigen::Quaternion<T> q(data[0], data[1], data[2], data[3]);
    q.normalize();
    Eigen::Matrix<T, 3, 3> R_H(q);
    Eigen::Matrix<T, 3, 1> t;
    t << data[4], data[5], data[6];

    // Eigen::Matrix<T, 3, 1> N;
    // N << data[7], data[8], data[9];

    Eigen::Matrix<T, 3, 1> p_q, p_db;
    p_q << static_cast<T>(p2D_q_x), static_cast<T>(p2D_q_y), static_cast<T>(1.0);
    // p_db << static_cast<T>(p2D_db_x / K(0, 0)),
            // static_cast<T>(p2D_db_y / K(1, 1)), static_cast<T>(1.0);
    p_db << static_cast<T>(p2D_db_x), static_cast<T>(p2D_db_y), static_cast<T>(1.0);

    // Computes the closest point on one line to the second one, then checks
    // if the point is in front of both cameras.
    // See https://en.wikipedia.org/wiki/Skew_lines#Nearest_Points for details.
    Eigen::Matrix<T, 3, 1> p1 = t;
    Eigen::Matrix<T, 3, 1> d1 = R_H * p_q;
    d1.normalize();
    Eigen::Matrix<T, 3, 1> p2 = c.template cast<T>();
    Eigen::Matrix<T, 3, 1> d2 = R.transpose().template cast<T>() * p_db;
    d2.normalize();

    Eigen::Matrix<T, 3, 1> n = d1.cross(d2);
    n.normalize();
    Eigen::Matrix<T, 3, 1> n1 = d1.cross(n);
    n1.normalize();
    Eigen::Matrix<T, 3, 1> n2 = d2.cross(n);
    n2.normalize();
    
    Eigen::Matrix<T, 3, 1> c1 = p1 + (p2 - p1).dot(n2) / (d1.dot(n2)) * d1;
    Eigen::Matrix<T, 3, 1> c2 = p2 + (p1 - p2).dot(n1) / (d2.dot(n1)) * d2;

    // c1 = static_cast<T>(0.5) * (c1 + c2); 

    // Eigen::Matrix<T, 3, 1> proj = K.template cast<T>() * R.template cast<T>() * (c1 - c.template cast<T>());
    Eigen::Matrix<T, 3, 1> proj = R.template cast<T>() * (c1 - c.template cast<T>());

    T x_proj = proj[0] / proj[2];
    T y_proj = proj[1] / proj[2];

    residuals[0] = static_cast<T>(p2D_db_x) - x_proj;
    residuals[1] = static_cast<T>(p2D_db_y) - y_proj;

    proj = R_H.transpose().template cast<T>() * (c2 - t);
    x_proj = proj[0] / proj[2];
    y_proj = proj[1] / proj[2];
    residuals[2] = static_cast<T>(p2D_q_x) - x_proj;
    residuals[3] = static_cast<T>(p2D_q_y) - y_proj;


    // Eigen::Matrix<T, 3, 1> proj2 = K.template cast<T>() * R_H.transpose() * (c2 - t);

    // T x_proj2 = proj2[0] / proj2[2];
    // T y_proj2 = proj2[1] / proj2[2];
    // std::cout << K(0, 0) << " " << K(1, 1) << std::endl;
    // std::cout << static_cast<double>(x_proj2) << " " << static_cast<double>(y_proj2) << std::endl;

    // residuals[0] = static_cast<T>(p2D_q_x * K(0, 0)) - x_proj2;
    // residuals[1] = static_cast<T>(p2D_q_y * K(1, 1)) - y_proj2;

    return true;
  }

  // Factory function
  //static ceres::CostFunction* CreateCost(const double x, const double y,
  //                                       const double x_db, const double y_db,
  //                                       // const Eigen::Matrix3d& K_,
  //                                       const Eigen::Matrix3d& R_,
  //                                       const Eigen::Vector3d& c_) {
  //  return (new ceres::AutoDiffCostFunction<ReprojectionTriangulationError, 4, 7>(
  //      // new ReprojectionTriangulationError(x, y, x_db, y_db, K_, R_, c_)));
  //      new ReprojectionTriangulationError(x, y, x_db, y_db, R_, c_)));
  //}

  // Assumes that the measurement is centered around the principal point.
  double p2D_q_x;
  double p2D_q_y;
  double p2D_db_x;
  double p2D_db_y;
  // Eigen::Matrix3d K;
  Eigen::Matrix3d R;
  Eigen::Vector3d c;
};

struct ReprojectionTriangulationErrorUncalibrated {
  ReprojectionTriangulationErrorUncalibrated(double x, double y, double x_db, double y_db,
                                 // const Eigen::Matrix3d& K_,
                                 const Eigen::Matrix3d& R_,
                                 const Eigen::Vector3d& c_)
      : p2D_q_x(x),
        p2D_q_y(y),
        p2D_db_x(x_db),
        p2D_db_y(y_db) {
    // K = K_;
    R = R_;
    c = c_;
  }

  template <typename T>
  bool operator()(const T* const data, T* residuals) const {
    // The first four entries encode the rotation matrix as a quaternion, the
    // next three entries correspond to the translation, the last three entries
    // correspond to N.
    Eigen::Quaternion<T> q(data[0], data[1], data[2], data[3]);
    q.normalize();
    Eigen::Matrix<T, 3, 3> R_H(q);
    Eigen::Matrix<T, 3, 1> t;
    t << data[4], data[5], data[6];
    // Last entry is the focal length.
    T focal = data[7];

    // Eigen::Matrix<T, 3, 1> N;
    // N << data[7], data[8], data[9];

    Eigen::Matrix<T, 3, 1> p_q, p_db;
    p_q << static_cast<T>(p2D_q_x) / focal, static_cast<T>(p2D_q_y) / focal, static_cast<T>(1.0);
    // p_db << static_cast<T>(p2D_db_x / K(0, 0)),
            // static_cast<T>(p2D_db_y / K(1, 1)), static_cast<T>(1.0);
    p_db << static_cast<T>(p2D_db_x), static_cast<T>(p2D_db_y), static_cast<T>(1.0);

    // Computes the closest point on one line to the second one, then checks
    // if the point is in front of both cameras.
    // See https://en.wikipedia.org/wiki/Skew_lines#Nearest_Points for details.
    Eigen::Matrix<T, 3, 1> p1 = t;
    Eigen::Matrix<T, 3, 1> d1 = R_H * p_q;
    d1.normalize();
    Eigen::Matrix<T, 3, 1> p2 = c.template cast<T>();
    Eigen::Matrix<T, 3, 1> d2 = R.transpose().template cast<T>() * p_db;
    d2.normalize();

    Eigen::Matrix<T, 3, 1> n = d1.cross(d2);
    n.normalize();
    Eigen::Matrix<T, 3, 1> n1 = d1.cross(n);
    n1.normalize();
    Eigen::Matrix<T, 3, 1> n2 = d2.cross(n);
    n2.normalize();
    
    Eigen::Matrix<T, 3, 1> c1 = p1 + (p2 - p1).dot(n2) / (d1.dot(n2)) * d1;
    Eigen::Matrix<T, 3, 1> c2 = p2 + (p1 - p2).dot(n1) / (d2.dot(n1)) * d2;

    // c1 = static_cast<T>(0.5) * (c1 + c2); 

    // Eigen::Matrix<T, 3, 1> proj = K.template cast<T>() * R.template cast<T>() * (c1 - c.template cast<T>());
    Eigen::Matrix<T, 3, 1> proj = R.template cast<T>() * (c1 - c.template cast<T>());

    T x_proj = proj[0] / proj[2];
    T y_proj = proj[1] / proj[2];

    residuals[0] = static_cast<T>(p2D_db_x) - x_proj;
    residuals[1] = static_cast<T>(p2D_db_y) - y_proj;

    proj = R_H.transpose().template cast<T>() * (c2 - t);
    x_proj = proj[0] / proj[2];
    y_proj = proj[1] / proj[2];
    residuals[2] = static_cast<T>(p2D_q_x) - x_proj;
    residuals[3] = static_cast<T>(p2D_q_y) - y_proj;


    // Eigen::Matrix<T, 3, 1> proj2 = K.template cast<T>() * R_H.transpose() * (c2 - t);

    // T x_proj2 = proj2[0] / proj2[2];
    // T y_proj2 = proj2[1] / proj2[2];
    // std::cout << K(0, 0) << " " << K(1, 1) << std::endl;
    // std::cout << static_cast<double>(x_proj2) << " " << static_cast<double>(y_proj2) << std::endl;

    // residuals[0] = static_cast<T>(p2D_q_x * K(0, 0)) - x_proj2;
    // residuals[1] = static_cast<T>(p2D_q_y * K(1, 1)) - y_proj2;

    return true;
  }

  // Factory function
  //static ceres::CostFunction* CreateCost(const double x, const double y,
  //                                       const double x_db, const double y_db,
  //                                       // const Eigen::Matrix3d& K_,
  //                                       const Eigen::Matrix3d& R_,
  //                                       const Eigen::Vector3d& c_) {
  //  return (new ceres::AutoDiffCostFunction<ReprojectionTriangulationErrorUncalibrated, 4, 8>(
  //      // new ReprojectionTriangulationError(x, y, x_db, y_db, K_, R_, c_)));
  //      new ReprojectionTriangulationErrorUncalibrated(x, y, x_db, y_db, R_, c_)));
  //}

  // Assumes that the measurement is centered around the principal point.
  double p2D_q_x;
  double p2D_q_y;
  double p2D_db_x;
  double p2D_db_y;
  // Eigen::Matrix3d K;
  Eigen::Matrix3d R;
  Eigen::Vector3d c;
};

// Epipolar error.
struct EpipolarError {
  EpipolarError(double x, double y, double x_db, double y_db,
                const Eigen::Matrix3d& K_, const Eigen::Matrix3d& R_,
                const Eigen::Vector3d& c_)
      : p2D_q_x(x),
        p2D_q_y(y),
        p2D_db_x(x_db),
        p2D_db_y(y_db) {
    K = K_;
    R = R_;
    c = c_;
  }

  template <typename T>
  bool operator()(const T* const data, T* residuals) const {
    // The first four entries encode the rotation matrix as a quaternion, the
    // next three entries correspond to the translation, the last three entries
    // correspond to N.
    Eigen::Quaternion<T> q(data[0], data[1], data[2], data[3]);
    q.normalize();
    Eigen::Matrix<T, 3, 3> R_H(q);
    Eigen::Matrix<T, 3, 1> t;
    t << data[4], data[5], data[6];

    Eigen::Matrix<T, 3, 1> N;
    N << data[7], data[8], data[9];

    // Recreates the homography.
    Eigen::Matrix<T, 3, 3> R_E = R.template cast<T>() * R_H;
    Eigen::Matrix<T, 3, 1> t_E = R.template cast<T>() (t - c.template cast<T>());

    Eigen::Matrix<T, 3, 3> t_E_cross;
    T zero = static_cast<T>(0.0);
    t_E_cross << zero, -t_E[2], t_E[1],
                 t_E[2], zero, -t_E[0],
                -t_E[1], t_E[0], zero;
    Eigen::Matrix<T, 3, 3> F;
    F.noalias() = t_E_cross * R_E;
    F.row(0) /= static_cast<T>(K(0, 0));
    F.row(1) /= static_cast<T>(K(1, 1));

    Eigen::Matrix<T, 3, 1> p, p2;
    p << static_cast<T>(p2D_q_x), static_cast<T>(p2D_q_y), static_cast<T>(1.0);
    p2 << static_cast<T>(p2D_db_x), static_cast<T>(p2D_db_y), static_cast<T>(1.0);
    Eigen::Matrix<T, 3, 1> l1 = F * p;
    Eigen::Matrix<T, 1, 3> l2 = p2 * F;
    residuals[0] = l1.dot(p2) * l1.dot(p2) / (l1.template head<2>().squaredNorm() + l2.template head<2>().squaredNorm());

    return true;
  }

  // Factory function
 /* static ceres::CostFunction* CreateCost(const double x, const double y,
                                         const double x_db, const double y_db,
                                         const Eigen::Matrix3d& K_,
                                         const Eigen::Matrix3d& R_,
                                         const Eigen::Vector3d& c_) {
    return (new ceres::AutoDiffCostFunction<ReprojectionError, 1, 10>(
        new ReprojectionError(x, y, x_db, y_db, K_, R_, c_)));
  }*/

  // Assumes that the measurement is centered around the principal point.
  double p2D_q_x;
  double p2D_q_y;
  double p2D_db_x;
  double p2D_db_y;
  Eigen::Matrix3d K;
  Eigen::Matrix3d R;
  Eigen::Vector3d c;
};


}  // namespace solvers

#endif  // RANSAC_SOLVERS_COMMON_H_
