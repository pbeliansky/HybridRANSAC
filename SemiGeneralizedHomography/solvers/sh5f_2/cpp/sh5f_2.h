#pragma once
#include <Eigen/Eigen>
#include "transform.h"
#include "parameterize_HN.h"
//#include "extract_poses.h"
#include "get_coeffs.h"
#include "sturm.h"
#include "extract_homographies.h"

namespace sh5f_2
{
	int solver_sturm(Eigen::Matrix<double, 3, 5> q, Eigen::Matrix<double, 3, 5> p, Eigen::Matrix<double, 3, 5> c,
		std::vector<double>* fss, std::vector<Eigen::Matrix3d>* Hs, std::vector<Eigen::Vector3d>* Nss, double tol)
	{
		Eigen::Matrix3d Rtransform; Eigen::Matrix3d Rtransform2; Eigen::Vector3d shift; Eigen::Matrix3d Ktransform;

		solver_common::transform_focal(q, p, c, Rtransform, Rtransform2, shift, Ktransform);

		//std::cout << q << std::endl << std::endl << p << std::endl << std::endl << c << std::endl << std::endl << Rtransform << std::endl << std::endl << Rtransform2 << std::endl << std::endl << shift << std::endl << std::endl << Ktransform << std::endl;

		auto M = parameterize_HN(q, p, c);

		//std::cout << M << std::endl;

		auto coef = get_coeffs(M.data());

		double roots[5] = { 0 };

		int n_roots = pose_lib::sturm::bisect_sturm<5>(coef.data(), roots, tol);

		//for (size_t i = 0; i < 5; i++)
			//std::cout << roots[i]<<std::endl;

		Eigen::Matrix<double, 9, Eigen::Dynamic, 0, 9, 5> vecs(9, n_roots);

		for (size_t i = 0; i < n_roots; ++i)
			vecs(8, i) = roots[i];

		vecs.block(0, 0, 8, n_roots) = (M.col(0) * vecs.row(8)).colwise() + M.col(1);

		fss->clear();
		Hs->clear();
		Nss->clear();
		solver_common::extract_homographies_sh5f<9, 5>(vecs, Rtransform, Rtransform2, shift, Ktransform, fss, Hs, Nss);

		return n_roots;
	}
}