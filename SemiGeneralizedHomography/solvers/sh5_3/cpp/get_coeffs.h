#pragma once
#include <Eigen/Eigen>

namespace sh5_3
{
	inline Eigen::Matrix<double, 4, 1> get_coeffs(const double* data)
	{
		Eigen::Matrix<double, 4, 1> C;

		C(0) = data[0] * pow(data[1], 2) * data[8] * data[9] - pow(data[1], 3) * data[9];
		C(1) = 2 * data[0] * data[1] * data[3] * data[8] * data[9] - data[0] * data[1] * pow(data[6], 2) - data[0] * data[1] * pow(data[7], 2) - data[0] * data[1] * pow(data[8], 2) + data[0] * data[1] * pow(data[9], 2) + pow(data[1], 2) * data[2] * data[8] * data[9] - 3 * pow(data[1], 2) * data[3] * data[9] + pow(data[1], 2) * data[4] * data[6] + pow(data[1], 2) * data[5] * data[7] + pow(data[1], 2) * data[8];
		C(2) = data[0] * pow(data[3], 2) * data[8] * data[9] - data[0] * data[3] * pow(data[6], 2) - data[0] * data[3] * pow(data[7], 2) - data[0] * data[3] * pow(data[8], 2) + data[0] * data[3] * pow(data[9], 2) - data[0] * data[8] * data[9] + 2 * data[1] * data[2] * data[3] * data[8] * data[9] - data[1] * data[2] * pow(data[6], 2) - data[1] * data[2] * pow(data[7], 2) - data[1] * data[2] * pow(data[8], 2) + data[1] * data[2] * pow(data[9], 2) - 3 * data[1] * pow(data[3], 2) * data[9] + 2 * data[1] * data[3] * data[4] * data[6] + 2 * data[1] * data[3] * data[5] * data[7] + 2 * data[1] * data[3] * data[8] - data[1] * data[9];
		C(3) = data[2] * pow(data[3], 2) * data[8] * data[9] - data[2] * data[3] * pow(data[6], 2) - data[2] * data[3] * pow(data[7], 2) - data[2] * data[3] * pow(data[8], 2) + data[2] * data[3] * pow(data[9], 2) - data[2] * data[8] * data[9] - pow(data[3], 3) * data[9] + pow(data[3], 2) * data[4] * data[6] + pow(data[3], 2) * data[5] * data[7] + pow(data[3], 2) * data[8] - data[3] * data[9] + data[4] * data[6] + data[5] * data[7] + data[8];

		return C;
	}


	inline Eigen::Matrix<double, 3, 3> get_companion(const double* data)
	{
		auto coeffs = get_coeffs(data);

		Eigen::Matrix<double, 3, 3> C;

		C << 0, 1, 0,
			0, 0, 1,
			-coeffs(0) / coeffs(3), -coeffs(1) / coeffs(3), -coeffs(2) / coeffs(3);

		return C;
	}
}