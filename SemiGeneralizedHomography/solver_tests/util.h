#pragma once
#include <vector>
#include <iostream>

#include <Eigen/Eigen>

struct qpc
{
    Eigen::Matrix<double, 3, 5> q;
    Eigen::Matrix<double, 3, 5> p;
    Eigen::Matrix<double, 3, 5> c;
};

void fill_with_random(std::vector<qpc>& v, size_t n, double scale)
{
    for (size_t i = 0; i < n; ++i)
    {
        v[i].q.block<2, 5>(0, 0).setRandom();
        v[i].q.block<2, 5>(0, 0) *= scale;
        v[i].q.block<1, 5>(2, 0).setOnes();

        v[i].p.block<2, 5>(0, 0).setRandom();
        v[i].p.block<2, 5>(0, 0) *= scale;
        v[i].p.block<1, 5>(2, 0).setOnes();

        v[i].c.setRandom();
        v[i].c *= scale;
    }
}