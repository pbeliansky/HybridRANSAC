#pragma once
#include <Eigen/Eigen>

namespace sh5f_3
{
	Eigen::Matrix<double, 4, 1> get_coeffs(const double* data)
	{
		Eigen::Matrix<double, 4, 1> C;

		C(0) = -pow(data[0], 3) * data[4] * data[6] - pow(data[0], 3) * data[5] * data[7] + pow(data[0], 3) * data[8] * data[9] + pow(data[0], 2) * data[1] * pow(data[4], 2) + pow(data[0], 2) * data[1] * pow(data[5], 2) - pow(data[0], 2) * data[1] * data[9] - data[0] * pow(data[1], 2) * data[4] * data[6] - data[0] * pow(data[1], 2) * data[5] * data[7] + data[0] * pow(data[1], 2) * data[8] * data[9] + pow(data[1], 3) * pow(data[4], 2) + pow(data[1], 3) * pow(data[5], 2) - pow(data[1], 3) * data[9];
		C(1) = -3 * pow(data[0], 2) * data[2] * data[4] * data[6] - 3 * pow(data[0], 2) * data[2] * data[5] * data[7] + 3 * pow(data[0], 2) * data[2] * data[8] * data[9] + pow(data[0], 2) * data[3] * pow(data[4], 2) + pow(data[0], 2) * data[3] * pow(data[5], 2) - pow(data[0], 2) * data[3] * data[9] - pow(data[0], 2) * data[4] * data[6] - pow(data[0], 2) * data[5] * data[7] - pow(data[0], 2) * data[8] + 2 * data[0] * data[1] * data[2] * pow(data[4], 2) + 2 * data[0] * data[1] * data[2] * pow(data[5], 2) - 2 * data[0] * data[1] * data[2] * data[9] - 2 * data[0] * data[1] * data[3] * data[4] * data[6] - 2 * data[0] * data[1] * data[3] * data[5] * data[7] + 2 * data[0] * data[1] * data[3] * data[8] * data[9] + data[0] * data[1] * pow(data[4], 2) + data[0] * data[1] * pow(data[5], 2) - data[0] * data[1] * pow(data[6], 2) - data[0] * data[1] * pow(data[7], 2) - data[0] * data[1] * pow(data[8], 2) + data[0] * data[1] - pow(data[1], 2) * data[2] * data[4] * data[6] - pow(data[1], 2) * data[2] * data[5] * data[7] + pow(data[1], 2) * data[2] * data[8] * data[9] + 3 * pow(data[1], 2) * data[3] * pow(data[4], 2) + 3 * pow(data[1], 2) * data[3] * pow(data[5], 2) - 3 * pow(data[1], 2) * data[3] * data[9] + pow(data[1], 2) * data[4] * data[6] + pow(data[1], 2) * data[5] * data[7] + pow(data[1], 2) * data[8];
		C(2) = -3 * data[0] * pow(data[2], 2) * data[4] * data[6] - 3 * data[0] * pow(data[2], 2) * data[5] * data[7] + 3 * data[0] * pow(data[2], 2) * data[8] * data[9] + 2 * data[0] * data[2] * data[3] * pow(data[4], 2) + 2 * data[0] * data[2] * data[3] * pow(data[5], 2) - 2 * data[0] * data[2] * data[3] * data[9] - 2 * data[0] * data[2] * data[4] * data[6] - 2 * data[0] * data[2] * data[5] * data[7] - 2 * data[0] * data[2] * data[8] - data[0] * pow(data[3], 2) * data[4] * data[6] - data[0] * pow(data[3], 2) * data[5] * data[7] + data[0] * pow(data[3], 2) * data[8] * data[9] + data[0] * data[3] * pow(data[4], 2) + data[0] * data[3] * pow(data[5], 2) - data[0] * data[3] * pow(data[6], 2) - data[0] * data[3] * pow(data[7], 2) - data[0] * data[3] * pow(data[8], 2) + data[0] * data[3] + data[1] * pow(data[2], 2) * pow(data[4], 2) + data[1] * pow(data[2], 2) * pow(data[5], 2) - data[1] * pow(data[2], 2) * data[9] - 2 * data[1] * data[2] * data[3] * data[4] * data[6] - 2 * data[1] * data[2] * data[3] * data[5] * data[7] + 2 * data[1] * data[2] * data[3] * data[8] * data[9] + data[1] * data[2] * pow(data[4], 2) + data[1] * data[2] * pow(data[5], 2) - data[1] * data[2] * pow(data[6], 2) - data[1] * data[2] * pow(data[7], 2) - data[1] * data[2] * pow(data[8], 2) + data[1] * data[2] + 3 * data[1] * pow(data[3], 2) * pow(data[4], 2) + 3 * data[1] * pow(data[3], 2) * pow(data[5], 2) - 3 * data[1] * pow(data[3], 2) * data[9] + 2 * data[1] * data[3] * data[4] * data[6] + 2 * data[1] * data[3] * data[5] * data[7] + 2 * data[1] * data[3] * data[8];
		C(3) = -pow(data[2], 3) * data[4] * data[6] - pow(data[2], 3) * data[5] * data[7] + pow(data[2], 3) * data[8] * data[9] + pow(data[2], 2) * data[3] * pow(data[4], 2) + pow(data[2], 2) * data[3] * pow(data[5], 2) - pow(data[2], 2) * data[3] * data[9] - pow(data[2], 2) * data[4] * data[6] - pow(data[2], 2) * data[5] * data[7] - pow(data[2], 2) * data[8] - data[2] * pow(data[3], 2) * data[4] * data[6] - data[2] * pow(data[3], 2) * data[5] * data[7] + data[2] * pow(data[3], 2) * data[8] * data[9] + data[2] * data[3] * pow(data[4], 2) + data[2] * data[3] * pow(data[5], 2) - data[2] * data[3] * pow(data[6], 2) - data[2] * data[3] * pow(data[7], 2) - data[2] * data[3] * pow(data[8], 2) + data[2] * data[3] + pow(data[3], 3) * pow(data[4], 2) + pow(data[3], 3) * pow(data[5], 2) - pow(data[3], 3) * data[9] + pow(data[3], 2) * data[4] * data[6] + pow(data[3], 2) * data[5] * data[7] + pow(data[3], 2) * data[8];
		
		return C;
	}

	Eigen::Matrix<double, 3, 3> get_companion(const double* data)
	{
		Eigen::Matrix<double, 3, 3> C;

		double c0 = -pow(data[0], 3) * data[4] * data[6] - pow(data[0], 3) * data[5] * data[7] + pow(data[0], 3) * data[8] * data[9] + pow(data[0], 2) * data[1] * pow(data[4], 2) + pow(data[0], 2) * data[1] * pow(data[5], 2) - pow(data[0], 2) * data[1] * data[9] - data[0] * pow(data[1], 2) * data[4] * data[6] - data[0] * pow(data[1], 2) * data[5] * data[7] + data[0] * pow(data[1], 2) * data[8] * data[9] + pow(data[1], 3) * pow(data[4], 2) + pow(data[1], 3) * pow(data[5], 2) - pow(data[1], 3) * data[9];
		double c1 = -3 * pow(data[0], 2) * data[2] * data[4] * data[6] - 3 * pow(data[0], 2) * data[2] * data[5] * data[7] + 3 * pow(data[0], 2) * data[2] * data[8] * data[9] + pow(data[0], 2) * data[3] * pow(data[4], 2) + pow(data[0], 2) * data[3] * pow(data[5], 2) - pow(data[0], 2) * data[3] * data[9] - pow(data[0], 2) * data[4] * data[6] - pow(data[0], 2) * data[5] * data[7] - pow(data[0], 2) * data[8] + 2 * data[0] * data[1] * data[2] * pow(data[4], 2) + 2 * data[0] * data[1] * data[2] * pow(data[5], 2) - 2 * data[0] * data[1] * data[2] * data[9] - 2 * data[0] * data[1] * data[3] * data[4] * data[6] - 2 * data[0] * data[1] * data[3] * data[5] * data[7] + 2 * data[0] * data[1] * data[3] * data[8] * data[9] + data[0] * data[1] * pow(data[4], 2) + data[0] * data[1] * pow(data[5], 2) - data[0] * data[1] * pow(data[6], 2) - data[0] * data[1] * pow(data[7], 2) - data[0] * data[1] * pow(data[8], 2) + data[0] * data[1] - pow(data[1], 2) * data[2] * data[4] * data[6] - pow(data[1], 2) * data[2] * data[5] * data[7] + pow(data[1], 2) * data[2] * data[8] * data[9] + 3 * pow(data[1], 2) * data[3] * pow(data[4], 2) + 3 * pow(data[1], 2) * data[3] * pow(data[5], 2) - 3 * pow(data[1], 2) * data[3] * data[9] + pow(data[1], 2) * data[4] * data[6] + pow(data[1], 2) * data[5] * data[7] + pow(data[1], 2) * data[8];
		double c2 = -3 * data[0] * pow(data[2], 2) * data[4] * data[6] - 3 * data[0] * pow(data[2], 2) * data[5] * data[7] + 3 * data[0] * pow(data[2], 2) * data[8] * data[9] + 2 * data[0] * data[2] * data[3] * pow(data[4], 2) + 2 * data[0] * data[2] * data[3] * pow(data[5], 2) - 2 * data[0] * data[2] * data[3] * data[9] - 2 * data[0] * data[2] * data[4] * data[6] - 2 * data[0] * data[2] * data[5] * data[7] - 2 * data[0] * data[2] * data[8] - data[0] * pow(data[3], 2) * data[4] * data[6] - data[0] * pow(data[3], 2) * data[5] * data[7] + data[0] * pow(data[3], 2) * data[8] * data[9] + data[0] * data[3] * pow(data[4], 2) + data[0] * data[3] * pow(data[5], 2) - data[0] * data[3] * pow(data[6], 2) - data[0] * data[3] * pow(data[7], 2) - data[0] * data[3] * pow(data[8], 2) + data[0] * data[3] + data[1] * pow(data[2], 2) * pow(data[4], 2) + data[1] * pow(data[2], 2) * pow(data[5], 2) - data[1] * pow(data[2], 2) * data[9] - 2 * data[1] * data[2] * data[3] * data[4] * data[6] - 2 * data[1] * data[2] * data[3] * data[5] * data[7] + 2 * data[1] * data[2] * data[3] * data[8] * data[9] + data[1] * data[2] * pow(data[4], 2) + data[1] * data[2] * pow(data[5], 2) - data[1] * data[2] * pow(data[6], 2) - data[1] * data[2] * pow(data[7], 2) - data[1] * data[2] * pow(data[8], 2) + data[1] * data[2] + 3 * data[1] * pow(data[3], 2) * pow(data[4], 2) + 3 * data[1] * pow(data[3], 2) * pow(data[5], 2) - 3 * data[1] * pow(data[3], 2) * data[9] + 2 * data[1] * data[3] * data[4] * data[6] + 2 * data[1] * data[3] * data[5] * data[7] + 2 * data[1] * data[3] * data[8];
		double c3 = -pow(data[2], 3) * data[4] * data[6] - pow(data[2], 3) * data[5] * data[7] + pow(data[2], 3) * data[8] * data[9] + pow(data[2], 2) * data[3] * pow(data[4], 2) + pow(data[2], 2) * data[3] * pow(data[5], 2) - pow(data[2], 2) * data[3] * data[9] - pow(data[2], 2) * data[4] * data[6] - pow(data[2], 2) * data[5] * data[7] - pow(data[2], 2) * data[8] - data[2] * pow(data[3], 2) * data[4] * data[6] - data[2] * pow(data[3], 2) * data[5] * data[7] + data[2] * pow(data[3], 2) * data[8] * data[9] + data[2] * data[3] * pow(data[4], 2) + data[2] * data[3] * pow(data[5], 2) - data[2] * data[3] * pow(data[6], 2) - data[2] * data[3] * pow(data[7], 2) - data[2] * data[3] * pow(data[8], 2) + data[2] * data[3] + pow(data[3], 3) * pow(data[4], 2) + pow(data[3], 3) * pow(data[5], 2) - pow(data[3], 3) * data[9] + pow(data[3], 2) * data[4] * data[6] + pow(data[3], 2) * data[5] * data[7] + pow(data[3], 2) * data[8];

		C << 0, 1, 0,
			0, 0, 1,
			-c0 / c3, -c1 / c3, -c2 / c3;
		return C;
	}
}