#pragma once
#include <Eigen/Eigen>
#include "transform.h"
#include "parameterize_HN.h"
//#include "extract_poses.h"
#include "get_coeffs.h"
#include "sturm.h"
#include "extract_homographies.h"

namespace sh5_2
{
	inline int solver_sturm(Eigen::Matrix<double, 3, 5> q, Eigen::Matrix<double, 3, 5> p, 
		Eigen::Matrix<double, 3, 5> c, std::vector<Eigen::Matrix3d>* homographies, 
		std::vector<Eigen::Vector3d>* N_h, double tol = 1e-20)
	{
		Eigen::Matrix3d Rtransform1; Eigen::Matrix3d Rtransform2; Eigen::Vector3d shift;

		solver_common::transform(q, p, c, Rtransform1, Rtransform2, shift);

		auto M = parameterize_HN(q, p, c);

		auto coef = get_coeffs(M.data());

		double roots[5] = { 0 };

		int n_roots = pose_lib::sturm::bisect_sturm<5>(coef.data(), roots, tol);

		Eigen::Matrix<double, 9, Eigen::Dynamic, 0, 9, 6> vecs(9, n_roots);

		for (size_t i = 0; i < n_roots; ++i)
			vecs(8, i) = roots[i];

		vecs.block(0, 0, 8, n_roots) = (M.col(0) * vecs.row(8)).colwise() + M.col(1);

		homographies->clear();
		N_h->clear();
		solver_common::extract_homographies_sh5<9, 6>(vecs, Rtransform1, Rtransform2, shift, homographies, N_h);

		return n_roots;
	}
}