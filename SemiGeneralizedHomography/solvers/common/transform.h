#pragma once

#include <Eigen/Eigen>

namespace solver_common
{
	inline void transform(Eigen::Matrix<double, 3, 5>& q_d, Eigen::Matrix<double, 3, 5>& p_d, Eigen::Matrix<double, 3, 5>& c_d, 
		Eigen::Matrix3d& Rtransform1, Eigen::Matrix3d& Rtransform2, Eigen::Vector3d& shift)
	{
		Eigen::Vector3d b(0, 0, 1);
		Eigen::Matrix3d id3;
		id3.setIdentity();

		// Coordinate transform Q, pin hole camera
		Eigen::Vector3d a = q_d.block<3, 1>(0, 0).normalized();
		Eigen::Vector3d v = a.cross(b);
		double c = a.dot(b);

		Eigen::Matrix3d vx;
		vx << 0, -v(2), v(1),
			v(2), 0, -v(0),
			-v(1), v(0), 0;

		Rtransform1 = id3 + vx + vx * vx / (1 + c);

		q_d = (Rtransform1 * q_d).colwise().hnormalized().colwise().homogeneous();	// q_d = Rtransform1 * q_d; q_d = q_d ./ q_d(3,:);

		q_d.colwise().normalize();	// q_d = q_d./sqrt(sum(q_d.^2));

		// Coordinate transform P, gen. camera
		shift = c_d.block<3, 1>(0, 0);
		a = p_d.block<3, 1>(0, 0).normalized();
		v = a.cross(b);
		c = a.dot(b);

		vx << 0, -v(2), v(1),
			v(2), 0, -v(0),
			-v(1), v(0), 0;

		Rtransform2 = id3 + vx + vx * vx / (1 + c);

		c_d = Rtransform2 * (c_d.colwise() - shift);
		p_d = Rtransform2 * p_d;

		p_d.colwise().normalize();	// p_d = p_d./sqrt(sum(p_d.^2));



		for (int i = 0; i < 5; ++i)
			c_d.col(i) = p_d.col(i).cross(c_d.col(i));	// might be faster to do it in place and *(-1)
	}
	
	
	inline void transform_focal(Eigen::Matrix<double, 3, 5>& q, Eigen::Matrix<double, 3, 5>& p, Eigen::Matrix<double, 3, 5>& c, 
		Eigen::Matrix3d& Rtransform, Eigen::Matrix3d& Rtransform2, Eigen::Vector3d& shift, Eigen::Matrix3d& Ktransform)
	{
		//Coordinate transforms Pin hole camera
		Eigen::Matrix2d aat;
		aat << q(0, 0), -q(1, 0),
			q(1, 0), q(0, 0);

		double scale = aat.row(0).norm();

		Eigen::Matrix2d bbt;
		bbt << scale, 0,
			0, scale;

		bbt *= aat.inverse();

		Rtransform << bbt(0, 0), -bbt(1, 0), 0,
			bbt(1, 0), bbt(1, 1), 0,
			0, 0, 1;

		Ktransform << 1 / scale, 0, 0,
			0, 1 / scale, 0,
			0, 0, 1;

		q = (Ktransform * Rtransform * q).colwise().hnormalized().colwise().homogeneous();


		// Coordinate transform gen. camera
		Eigen::Vector3d b(0, 0, 1);
		Eigen::Matrix3d id3;
		id3.setIdentity();

		shift = c.block<3, 1>(0, 0);

		Eigen::Vector3d a = p.block<3, 1>(0, 0).normalized();
		Eigen::Vector3d v = a.cross(b);
		double cc = a.dot(b);

		Eigen::Matrix3d vx;
		vx << 0, -v(2), v(1),
			v(2), 0, -v(0),
			-v(1), v(0), 0;

		Rtransform2 = id3 + vx + vx * vx / (1 + cc);
		c = Rtransform2 * (c.colwise() - shift);

		// All cameras
		p = (Rtransform2 * p).colwise().normalized();

		for (int i = 0; i < 5; ++i)
			c.col(i) = p.col(i).cross(c.col(i));	// might be faster to do it in place and *(-1)

	}
}