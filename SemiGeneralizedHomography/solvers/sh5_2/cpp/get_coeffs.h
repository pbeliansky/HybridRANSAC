#pragma once
#include <Eigen/Eigen>

namespace sh5_2
{
	inline Eigen::Matrix<double, 6, 1> get_coeffs(const double* data)
	{
		Eigen::Matrix<double, 6, 1> C;

		C(0) = data[12] * data[13] * data[14] * pow(data[15], 2) - data[13] * pow(data[15], 3);
		C(1) = data[4] * data[13] * data[14] * pow(data[15], 2) + data[5] * data[12] * data[14] * pow(data[15], 2) - data[5] * pow(data[15], 3) + data[6] * data[12] * data[13] * pow(data[15], 2) + 2 * data[7] * data[12] * data[13] * data[14] * data[15] - 3 * data[7] * data[13] * pow(data[15], 2) + data[8] * data[10] * pow(data[15], 2) + data[9] * data[11] * pow(data[15], 2) - pow(data[10], 2) * data[14] * data[15] - pow(data[11], 2) * data[14] * data[15] - pow(data[12], 2) * data[14] * data[15] + data[12] * pow(data[15], 2) + pow(data[13], 2) * data[14] * data[15];
		C(2) = data[0] * data[10] * pow(data[15], 2) + data[1] * data[11] * pow(data[15], 2) + data[2] * data[8] * pow(data[15], 2) - 2 * data[2] * data[10] * data[14] * data[15] + data[3] * data[9] * pow(data[15], 2) - 2 * data[3] * data[11] * data[14] * data[15] + data[4] * data[5] * data[14] * pow(data[15], 2) + data[4] * data[6] * data[13] * pow(data[15], 2) + 2 * data[4] * data[7] * data[13] * data[14] * data[15] - 2 * data[4] * data[12] * data[14] * data[15] + data[4] * pow(data[15], 2) + data[5] * data[6] * data[12] * pow(data[15], 2) + 2 * data[5] * data[7] * data[12] * data[14] * data[15] - 3 * data[5] * data[7] * pow(data[15], 2) + 2 * data[5] * data[13] * data[14] * data[15] + 2 * data[6] * data[7] * data[12] * data[13] * data[15] - data[6] * pow(data[10], 2) * data[15] - data[6] * pow(data[11], 2) * data[15] - data[6] * pow(data[12], 2) * data[15] + data[6] * pow(data[13], 2) * data[15] + pow(data[7], 2) * data[12] * data[13] * data[14] - 3 * pow(data[7], 2) * data[13] * data[15] + 2 * data[7] * data[8] * data[10] * data[15] + 2 * data[7] * data[9] * data[11] * data[15] - data[7] * pow(data[10], 2) * data[14] - data[7] * pow(data[11], 2) * data[14] - data[7] * pow(data[12], 2) * data[14] + 2 * data[7] * data[12] * data[15] + data[7] * pow(data[13], 2) * data[14] - data[12] * data[13] * data[14] - data[13] * data[15];
		C(3) = data[0] * data[2] * pow(data[15], 2) + 2 * data[0] * data[7] * data[10] * data[15] + data[1] * data[3] * pow(data[15], 2) + 2 * data[1] * data[7] * data[11] * data[15] - pow(data[2], 2) * data[14] * data[15] - 2 * data[2] * data[6] * data[10] * data[15] + 2 * data[2] * data[7] * data[8] * data[15] - 2 * data[2] * data[7] * data[10] * data[14] - pow(data[3], 2) * data[14] * data[15] - 2 * data[3] * data[6] * data[11] * data[15] + 2 * data[3] * data[7] * data[9] * data[15] - 2 * data[3] * data[7] * data[11] * data[14] - pow(data[4], 2) * data[14] * data[15] + data[4] * data[5] * data[6] * pow(data[15], 2) + 2 * data[4] * data[5] * data[7] * data[14] * data[15] + 2 * data[4] * data[6] * data[7] * data[13] * data[15] - 2 * data[4] * data[6] * data[12] * data[15] + data[4] * pow(data[7], 2) * data[13] * data[14] - 2 * data[4] * data[7] * data[12] * data[14] + 2 * data[4] * data[7] * data[15] - data[4] * data[13] * data[14] + pow(data[5], 2) * data[14] * data[15] + 2 * data[5] * data[6] * data[7] * data[12] * data[15] + 2 * data[5] * data[6] * data[13] * data[15] + data[5] * pow(data[7], 2) * data[12] * data[14] - 3 * data[5] * pow(data[7], 2) * data[15] + 2 * data[5] * data[7] * data[13] * data[14] - data[5] * data[12] * data[14] - data[5] * data[15] + data[6] * pow(data[7], 2) * data[12] * data[13] - data[6] * data[7] * pow(data[10], 2) - data[6] * data[7] * pow(data[11], 2) - data[6] * data[7] * pow(data[12], 2) + data[6] * data[7] * pow(data[13], 2) - data[6] * data[12] * data[13] - pow(data[7], 3) * data[13] + pow(data[7], 2) * data[8] * data[10] + pow(data[7], 2) * data[9] * data[11] + pow(data[7], 2) * data[12] - data[7] * data[13] + data[8] * data[10] + data[9] * data[11] + data[12];
		C(4) = 2 * data[0] * data[2] * data[7] * data[15] + data[0] * pow(data[7], 2) * data[10] + data[0] * data[10] + 2 * data[1] * data[3] * data[7] * data[15] + data[1] * pow(data[7], 2) * data[11] + data[1] * data[11] - pow(data[2], 2) * data[6] * data[15] - pow(data[2], 2) * data[7] * data[14] - 2 * data[2] * data[6] * data[7] * data[10] + data[2] * pow(data[7], 2) * data[8] + data[2] * data[8] - pow(data[3], 2) * data[6] * data[15] - pow(data[3], 2) * data[7] * data[14] - 2 * data[3] * data[6] * data[7] * data[11] + data[3] * pow(data[7], 2) * data[9] + data[3] * data[9] - pow(data[4], 2) * data[6] * data[15] - pow(data[4], 2) * data[7] * data[14] + 2 * data[4] * data[5] * data[6] * data[7] * data[15] + data[4] * data[5] * pow(data[7], 2) * data[14] - data[4] * data[5] * data[14] + data[4] * data[6] * pow(data[7], 2) * data[13] - 2 * data[4] * data[6] * data[7] * data[12] - data[4] * data[6] * data[13] + data[4] * pow(data[7], 2) + data[4] + pow(data[5], 2) * data[6] * data[15] + pow(data[5], 2) * data[7] * data[14] + data[5] * data[6] * pow(data[7], 2) * data[12] + 2 * data[5] * data[6] * data[7] * data[13] - data[5] * data[6] * data[12] - data[5] * pow(data[7], 3) - data[5] * data[7];
		C(5) = data[0] * data[2] * pow(data[7], 2) + data[0] * data[2] + data[1] * data[3] * pow(data[7], 2) + data[1] * data[3] - pow(data[2], 2) * data[6] * data[7] - pow(data[3], 2) * data[6] * data[7] - pow(data[4], 2) * data[6] * data[7] + data[4] * data[5] * data[6] * pow(data[7], 2) - data[4] * data[5] * data[6] + pow(data[5], 2) * data[6] * data[7];

		return C;
	}

	inline Eigen::Matrix<double, 5, 5> get_companion(const double* data)
	{
		auto coeffs = get_coeffs(data);

		Eigen::Matrix<double, 5, 5> C;

		C << 0, 1, 0, 0, 0,
			0, 0, 1, 0, 0,
			0, 0, 0, 1, 0,
			0, 0, 0, 0, 1,
			-coeffs(0) / coeffs(5), -coeffs(1) / coeffs(5), -coeffs(2) / coeffs(5), -coeffs(3) / coeffs(5), -coeffs(4) / coeffs(5);

		return C;
	}
}