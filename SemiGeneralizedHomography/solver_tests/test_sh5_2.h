#pragma once
#include <chrono> 
#include <vector>
#include <iostream>

#include <Eigen/Eigen>

#include <solvers/sh5_2/cpp/sh5_2.h>
#include <solvers/sh5_2/cpp/sh5_2_cm.h>

#include "util.h" 


namespace test_sh5_2
{
	template <int config>
	void run_solver(Eigen::Matrix<double, 3, 5>& q, Eigen::Matrix<double, 3, 5>& p, Eigen::Matrix<double, 3, 5>& c, std::vector<Eigen::Matrix3d>& Hs, std::vector<Eigen::Vector3d>& Nss, std::vector<double>& fss, double sturm_tol = 1e-20)
	{
		if (config == 0)
			sh5_2::solver_sturm(q, p, c, &Hs, &Nss, sturm_tol);
		else if (config == 1)
			sh5_2::solver_companion_matrix(q, p, c, &Hs, &Nss);
	}
}
#pragma once