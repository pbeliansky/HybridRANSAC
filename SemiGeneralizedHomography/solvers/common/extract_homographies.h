#pragma once

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <array> 
#include <cmath>
#include <complex>
#include <vector>
#include <iostream>

namespace solver_common
{
	template <class T>
	inline constexpr T sign(T x)
	{
		return x == T(0) ? T(0) : x / abs(x);
	}

	template <int m, int max_n>
	using Md = Eigen::Matrix<double, m, Eigen::Dynamic, 0, m, max_n>;



    template <int m, int max_n>
    inline int extract_homographies_sh45(const Md<m, max_n>& vecs, const Eigen::Matrix3d& Rtransform,
        const Eigen::Matrix3d& Rtransform2, const Eigen::Vector3d& shift, std::vector<Eigen::Matrix3d>* Hs,
        std::vector<Eigen::Vector3d>* Nss, const Eigen::Matrix<double, 3, 5>& q, const Eigen::Matrix<double, 3, 5>& p,
        const Eigen::Matrix<double, 3, 5>& c)
    {
        Eigen::Vector3d Hq;
        Eigen::Vector3d residue;
        double minResidue = 10000.0;
        int index = -1;
        for (int i = 0; i < vecs.cols(); ++i)
        {
            Eigen::Vector3d N(vecs(6, i), vecs(7, i), vecs(8, i));

            Eigen::Matrix3d H;
            H << vecs(0, i), vecs(2, i), 0,
                vecs(1, i), vecs(3, i), 0,
                vecs(4, i), vecs(5, i), 1;

            // Using the elimination ideal(the simplest polynomial before we eliminated h31 from it) to extract h31 from it.
            double scale = sqrt((vecs(7, i) * vecs(7, i) + vecs(8, i) * vecs(8, i)) /
                (vecs(2, i) * vecs(2, i) * vecs(8, i) * vecs(8, i) + vecs(3, i) * vecs(3, i) * vecs(8, i) * vecs(8, i) + vecs(5, i) * vecs(5, i) * vecs(8, i) * vecs(8, i) -
                    2 * vecs(7, i) * vecs(8, i) * vecs(5, i) + vecs(7, i) * vecs(7, i)));


            // We need to ensure that the plane vector has a sign such that the
            // depth of the point is + ve.
            // If the 3D point is X = alpha * q then,
            // alpha * N ^ T * q + 1 = 0
            // which means that N ^ T * q must be - ve so that alpha is + ve
            // scale *= -sign(scale * vecs(8, i));
            if (scale * vecs(8, i) > 0) {
                scale = -scale;
            }

            N *= scale;
            H *= scale;

            // Testing the 5th point correspondence.
            Hq = H * q.col(4);
            residue = p.col(4).cross(Hq) + (N.dot(q.col(4))) * c.col(4);

            if (residue.norm() < minResidue) {
                minResidue = residue.norm();
                index = i;
            }

            //The reverse coordinate transform for
            // homographyand the plane vector
            Nss->push_back(Rtransform.transpose() * N);
            Hs->push_back(Rtransform2.transpose() * H);
            Hs->back() *= Rtransform;
            Hs->back() -= shift * Nss->back().transpose();
        }
        return index;
    }



	template <int m, int max_n>
	inline void extract_homographies_sh5(const Md<m, max_n>& vecs, const Eigen::Matrix3d& Rtransform, const Eigen::Matrix3d& Rtransform2, const Eigen::Vector3d& shift, 
		std::vector<Eigen::Matrix3d>* Hs, std::vector<Eigen::Vector3d>* Nss)
	{
		for (int i = 0; i < vecs.cols(); ++i)
		{
			Eigen::Vector3d N(vecs(6, i), vecs(7, i), vecs(8, i));

			Eigen::Matrix3d H;
			H << vecs(0, i), vecs(2, i), 0,
				vecs(1, i), vecs(3, i), 0,
				1, vecs(4, i), vecs(5, i);


			// Using the elimination ideal(the simplest polynomial before we eliminated h31 from it) to extract h31 from it.
			double scale = sqrt((vecs(7, i) * vecs(7, i) + vecs(8, i) * vecs(8, i)) / (vecs(2, i) * vecs(2, i) * vecs(8, i) * vecs(8, i) + vecs(3, i) * vecs(3, i) * vecs(8, i) * vecs(8, i) + vecs(4, i) * vecs(4, i) * vecs(8, i) * vecs(8, i) - (double)2 * vecs(4, i) * vecs(5, i) * vecs(7, i) * vecs(8, i) + vecs(5, i) * vecs(5, i) * vecs(7, i) * vecs(7, i)));


			// We need to ensure that the plane vector has a sign such that the
			// depth of the point is + ve.
			// If the 3D point is X = alpha * q then,
			// alpha * N ^ T * q + 1 = 0
			// which means that N ^ T * q must be - ve so that alpha is + ve
			scale *= -sign(scale * vecs(8, i));
			N *= scale;
			H *= scale;


			//The reverse coordinate transform for
			// homography and the plane vector
			Nss->push_back(Rtransform.transpose() * N);
			Hs->push_back(Rtransform2.transpose() * H);
			Hs->back() *= Rtransform;
			Hs->back() -= shift * Nss->back().transpose();
		}
	}



	template <int m, int max_n>
	inline void extract_homographies_sh5f(const Md<m, max_n>& vecs, const Eigen::Matrix3d& Rtransform, const Eigen::Matrix3d& Rtransform2, 
		const Eigen::Vector3d& shift, const Eigen::Matrix3d& Ktransform,
		std::vector<double>* fss, std::vector<Eigen::Matrix3d>* Hs, std::vector<Eigen::Vector3d>* Nss)
	{
		//Eigen::Matrix3d H;
		for (int i = 0; i < vecs.cols(); ++i)
		{
			Eigen::Vector3d N(vecs(6, i), vecs(7, i), vecs(8, i));

			Eigen::Matrix3d H;
			H << vecs(0, i), vecs(2, i), -vecs(0, i),
				vecs(1, i), vecs(3, i), -vecs(1, i),
				1, vecs(4, i), vecs(5, i);

			double scale = sqrt((pow(vecs(6, i), 2) + pow(vecs(7, i), 2)) / (pow(vecs(0, i), 2) * pow(vecs(7, i), 2) - 2 * vecs(0, i) * vecs(2, i) * vecs(6, i) * vecs(7, i) + pow(vecs(2, i), 2) * pow(vecs(6, i), 2) + pow(vecs(1, i), 2) * pow(vecs(7, i), 2) - 2 * vecs(1, i) * vecs(3, i) * vecs(6, i) * vecs(7, i) + pow(vecs(3, i), 2) * pow(vecs(6, i), 2) + pow(vecs(4, i), 2) * pow(vecs(6, i), 2) - 2 * vecs(4, i) * vecs(6, i) * vecs(7, i) + pow(vecs(7, i), 2)));

			// We need to ensure that the plane vector has a sign such that the
			// depth of the point is + ve.
			// If the 3D point is X = alpha * K ^ -1 * q then,
			// alpha * N ^ T * K ^ -1 * q + 1 = 0
			// which means that N ^ T * K ^ -1 * q must be - ve so that alpha is + ve
			scale *= -sign(scale * (vecs(6, i) + vecs(8, i)));

			N *= scale;
			H *= scale;

			// Extracting the focal length.
			double f = abs((std::complex<double>)vecs(7, i) / sqrt((std::complex<double>) (scale * scale * vecs(0, i) * vecs(0, i) * vecs(7, i) * vecs(7, i) + 2 * vecs(0, i) * vecs(2, i) * vecs(7, i) * vecs(8, i) * scale * scale + vecs(2, i) * vecs(2, i) * vecs(8, i) * vecs(8, i) * scale * scale + vecs(1, i) * vecs(1, i) * vecs(7, i) * vecs(7, i) * scale * scale + 2 * vecs(1, i) * vecs(3, i) * vecs(7, i) * vecs(8, i) * scale * scale
				+ vecs(3, i) * vecs(3, i) * vecs(8, i) * vecs(8, i) * scale * scale + vecs(4, i) * vecs(4, i) * vecs(8, i) * vecs(8, i) * scale * scale - 2 * vecs(4, i) * vecs(5, i) * vecs(7, i) * vecs(8, i) * scale * scale + vecs(5, i) * vecs(5, i) * vecs(7, i) * vecs(7, i) * scale * scale - vecs(8, i) * vecs(8, i))));

			Eigen::Matrix3d K; K << 1, 0, 0, 0, 1, 0, 0, 0, f;
			fss->push_back(1 / (f * Ktransform(0, 0)));

			N = K * N;
			H *= K;

			// Homography solution is reverse transformed
			Nss->push_back(Rtransform.transpose() * N);
			Hs->push_back(Rtransform2.transpose() * H);
			Hs->back() *= Rtransform;
			Hs->back() -= shift * Nss->back().transpose();

			//std::cout << fss->back() << std::endl << Nss->back() << std::endl << Hs->back() << std::endl;
		}
	}
}