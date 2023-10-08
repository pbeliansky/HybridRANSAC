#ifndef SOLVERS_COMMON_COMMON_H_
#define SOLVERS_COMMON_COMMON_H_

#include <cmath>

namespace solver_common
{
	//return roots for cubic polynomial x^3 + c2*x^2 + c1*x + c0 = 0
	inline int solve_cubic_real(double c2, double c1, double c0, double* root) {
		const double pi = 3.141592653589793238462643383279;
		double a = c1 - c2 * c2 / 3.0;
		double b = (2.0 * c2 * c2 * c2 - 9.0 * c2 * c1) / 27.0 + c0;
		double c = b * b / 4.0 + a * a * a / 27.0;
		if (c > 0) {
			c = std::sqrt(c);
			b *= -0.5;
			root[0] = std::cbrt(b + c) + std::cbrt(b - c) - c2 / 3.0;
			return 1;
		}
		else {
			c = 3.0 * b / (2.0 * a) * std::sqrt(-3.0 / a);
			root[0] = 2.0 * std::sqrt(-a / 3.0) * std::cos(std::acos(c) / 3.0) - c2 / 3.0;
			root[1] = 2.0 * std::sqrt(-a / 3.0) * std::cos(std::acos(c) / 3.0 - 2 * pi / 3) - c2 / 3.0;
			root[2] = 2.0 * std::sqrt(-a / 3.0) * std::cos(std::acos(c) / 3.0 - 4 * pi / 3) - c2 / 3.0;
			return 3;
		}
	}


	void complex_sort(Eigen::Matrix<std::complex<double>, 2, 12>& A)
	{
		std::vector< Eigen::Matrix<std::complex<double>, 2, 1>> AA(12);

		for (size_t i = 0; i < 12; ++i)
			AA[i] = A.col(i);

		for (auto&& c : AA)
		{
			for (size_t i = 0; i < c.size(); ++i)
				c[i] = abs(c[i].imag()) > 1.0e-10 ? (0, 0) : c[i];
		}

		std::sort(AA.data(), AA.data() + AA.size(), [](Eigen::Matrix<std::complex<double>, 2, 1> a, Eigen::Matrix<std::complex<double>, 2, 1> b) {
			return abs(a(1).real()) > abs(b(1).real());
			});

		for (size_t i = 0; i < 12; ++i)
			A.col(i) = AA[i];
	}
}

#endif	// SOLVERS_COMMON_COMMON_H_