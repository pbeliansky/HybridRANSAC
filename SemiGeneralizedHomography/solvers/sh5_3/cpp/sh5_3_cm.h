#pragma once
#include <Eigen/Eigen>
#include "transform.h"
#include "parametrize_HN.h"
#include "get_coeffs.h"
#include "extract_homographies.h"


namespace sh5_3
{
	inline int solver_companion_matrix(Eigen::Matrix<double, 3, 5> q, Eigen::Matrix<double, 3, 5> p,
		Eigen::Matrix<double, 3, 5> c, std::vector<Eigen::Matrix3d>* homographies, 
		std::vector<Eigen::Vector3d>* N_h)
	{
		Eigen::Matrix3d Rtransform1; Eigen::Matrix3d Rtransform2; Eigen::Vector3d shift;

		solver_common::transform(q, p, c, Rtransform1, Rtransform2, shift);

		// Computing the coefficients
		// Estimate the scaled version of the H matrix
		auto M = parameterize_HN(q, p, c);

		Eigen::Matrix<double, 6, 1> h = (-M.block<8, 8>(0, 0)).lu().solve(M.block<8, 1>(0, 9)).block<6,1>(0,0);

		// Estimate the scaled version of the plane vector
		Eigen::Matrix<double, 2, 3> b;	//useless
		b << M.block<1, 3>(5, 6),
			M.block<1, 3>(7, 6);

		Eigen::Vector2d A;
		A << -M.block<1, 6>(5, 0) * h - M(5, 9),
			-M.block<1, 6>(7, 0) * h - M(7, 9);

		Eigen::Vector2d c1 = b.block<2, 2>(0, 0).lu().solve(A);

		Eigen::Vector2d c2 = (-b.block<2, 2>(0, 0)).lu().solve(b.col(2));

		double data[10];
		std::copy(c1.data(), c1.data() + 2, data);
		std::copy(c2.data(), c2.data() + 2, data + 2);
		std::copy(h.data(), h.data() + 6, data + 4);

		auto companion = get_companion(data);


		Eigen::EigenSolver<Eigen::Matrix<double, 3, 3>> es(companion, false);
		Eigen::MatrixXcd D = es.eigenvalues();

		int n_roots = 0;
		double roots[3];

		for (size_t i = 0; i < D.size(); ++i)
		{
			if (abs(D(i).imag()) < 1e-6)
				roots[n_roots++] = D(i).real();
		}


		Eigen::Matrix<double, 9, Eigen::Dynamic, 0, 9, 3> vecs(9, n_roots);

		for (size_t i = 0; i < n_roots; ++i)
		{
			vecs.block<6, 1>(0, i) = h;
			vecs(6, i) = data[0] + data[2] * roots[i];
			vecs(7, i) = data[1] + data[3] * roots[i];
			vecs(8, i) = roots[i];
		}

		homographies->clear();
		N_h->clear();
		solver_common::extract_homographies_sh5<9, 3>(vecs, Rtransform1, Rtransform2, shift, homographies, N_h);

		return n_roots;
	}
}