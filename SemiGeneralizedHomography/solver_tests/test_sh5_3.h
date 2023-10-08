#pragma once
#include <chrono> 
#include <vector>
#include <iostream>

#include <Eigen/Eigen>

#include <solvers/sh5_3/cpp/sh5_3.h>
#include <solvers/sh5_3/cpp/sh5_3_cm.h>
#include <solvers/sh5_3/cpp/sh5_3_cf.h>

#include "util.h"


namespace test_sh5_3
{
	template <int config>
	void run_solver(Eigen::Matrix<double, 3, 5>& q, Eigen::Matrix<double, 3, 5>& p, Eigen::Matrix<double, 3, 5>& c, std::vector<Eigen::Matrix3d>& Hs, std::vector<Eigen::Vector3d>& Nss, std::vector<double>& fss, double sturm_tol = 1e-20)
	{
		if (config == 0)
			sh5_3::solver_sturm(q, p, c, &Hs, &Nss, sturm_tol);
		else if (config == 1)
			sh5_3::solver_closed_form(q, p, c, &Hs, &Nss);
		else if (config == 2)
			sh5_3::solver_companion_matrix(q, p, c, &Hs, &Nss);
	}
}
#pragma once