#pragma once
#include <chrono> 
#include <vector>
#include <iostream>

#include <Eigen/Eigen>

#include <solvers/sh4.5_3/cpp/sh45_3.h>
#include <solvers/sh4.5_3/cpp/sh45_3_o.h>
#include <solvers/sh4.5_3/cpp/sh45_3_d.h>

#include "util.h"

namespace test_sh45_3
{
	template <int config>
	void run_solver(Eigen::Matrix<double, 3, 5>& q, Eigen::Matrix<double, 3, 5>& p, Eigen::Matrix<double, 3, 5>& c, std::vector<Eigen::Matrix3d>& Hs, std::vector<Eigen::Vector3d>& Nss, std::vector<double>& fss, double sturm_tol = 1e-20)
	{
		if (config == 0)
			sh45_3::solver_eigen(q, p, c, &Hs, &Nss, 0);
		else if (config == 1)
			sh45_3::solver_eigen_optimized(q, p, c, &Hs, &Nss, 0);
		else if (config == 2)
			sh45_3::solver_danilevsky(q, p, c, &Hs, &Nss, 0, sturm_tol);
	}
}