#pragma once
#include <Eigen/Eigen>
#include "transform.h"
#include "parameterize_HN.h"

#include "extract_homographies.h"

#include <chrono>

using namespace Eigen;

namespace sh45_2
{
    
    inline MatrixXd solver_problem_sh45_2_o(const VectorXd& data)
    {
        // Setup elimination template
        static const int coeffs0_ind[] = { 5,3,4,10,5,6,7,8,9,15,10,11,12,13,14,20,15,16,17,18,19,25,20,21,22,23,24,30,3,4,35,30,31,32,33,8,6,7,5,9,34,40,35,36,37,38,13,11,12,10,14,39,45,40,41,42,43,18,16,17,15,19,44,50,45,46,47,48,23,21,22,20,24,49,25,26,27,28,29 };
        static const int coeffs1_ind[] = { 102,104,93,91,92,94,78,76,77,75,79,75,58,56,57,55,59,55,33,31,32,30,34,60,55,56,57,58,38,36,37,35,39,59,80,75,76,77,78,63,61,62,60,64,79,65,60,61,62,63,43,41,42,40,44,64,91,92,93,83,81,82,80,84,94,85,80,81,82,83,68,66,67,65,69,84,70,65,66,67,68,48,46,47,45,49,69,102,98,96,97,99,104,96,97,98,88,86,87,85,89,99,85,86,87,88,73,71,72,70,74,89,70,71,72,73,53,51,52,50,54,74,50,51,52,53,28,26,27,25,29,54 };
        static const int C0_ind[] = { 0,4,10,11,12,13,14,15,21,22,23,24,25,26,32,33,34,35,36,37,43,44,45,46,47,48,54,55,60,64,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,111,112,113,114,120 } ;
        static const int C1_ind[] = { 7,9,16,17,18,20,27,28,29,30,31,33,38,39,40,41,42,44,49,50,51,52,53,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,124,126,127,128,130,131,134,135,136,137,138,139,140,141,142,144,145,146,147,148,149,150,151,152,153,155,156,157,158,159,160,161,162,163,164,166,167,168,169,170,171,172,173,174,175 };
        
        Matrix<double,11,11> C0; C0.setZero();
        Matrix<double,11,16> C1; C1.setZero();
        for (int i = 0; i < 79; i++) { C0(C0_ind[i]) = data(coeffs0_ind[i]); }
        for (int i = 0; i < 132; i++) { C1(C1_ind[i]) = data(coeffs1_ind[i]); }
        
        Matrix<double,11,16> C12 = C0.partialPivLu().solve(C1);
        
        // Setup action matrix
        Matrix<double,21, 16> RR;
        RR.block<5, 16>(0, 0) = -C12.bottomRows(5);
        RR.block<16, 16>(5, 0).setIdentity();
        //RR << -C12.bottomRows(5), Matrix<double,16,16>::Identity(16, 16);
        
        static const int AM_ind[] = { 16,13,11,10,0,1,12,2,14,15,3,17,18,19,20,4 };
        Matrix<double, 16, 16> AM;
        for (int i = 0; i < 16; i++) {
            AM.row(i) = RR.row(AM_ind[i]);
        }        
        
        // Solve eigenvalue problem
        EigenSolver<Matrix<double, 16, 16> > es(AM);
        ArrayXcd D = es.eigenvalues();
        ArrayXXcd V = es.eigenvectors();
        V = (V / V.row(0).array().replicate(16, 1)).eval();
                
        Matrix<double, 3, 16> solsReal;
        Matrix<std::complex<double>, 2, 16> solsComplex;
        solsComplex.setZero();
        solsReal.setZero();
                
        solsComplex.row(0) = V.row(1).array();
        solsComplex.row(1) = D.transpose().array();

        solsReal.block<2, 16>(0, 0) = solsComplex.real();
        solsReal.row(2).setOnes();
        
        return solsReal ;
    }
    
    inline int solver_eigen_optimized(Eigen::Matrix<double, 3, 5> q, Eigen::Matrix<double, 3, 5> p,
            Eigen::Matrix<double, 3, 5> c, std::vector<Eigen::Matrix3d>* homographies,
            std::vector<Eigen::Vector3d>* N_h, int test5thPt)
    {
        Eigen::Matrix3d Rtransform1; Eigen::Matrix3d Rtransform2; Eigen::Vector3d shift;
        
        solver_common::transform(q, p, c, Rtransform1, Rtransform2, shift);
        int n_roots = 16;
        // Nullspace
        Eigen::Matrix<double, 7,3> nullSpace;
        Eigen::Matrix<double, 5, 21> coeffsTemp = get_coeffs_opt(q, p, c, nullSpace);
        
        
        // Normalizing each equation's coefficients                                           ////////why twice
        coeffsTemp /= coeffsTemp.norm();

        Map<VectorXd> data(coeffsTemp.data(), 105);
        
        // We execute the GB solver to extract the roots
        MatrixXd roots = solver_problem_sh45_2_o(data);
        
        Eigen::Matrix<double, 10, Eigen::Dynamic, 0, 10, 16> vecs(10, n_roots);
        
        for (size_t i = 0; i < n_roots; ++i){
            vecs(7, i) = roots(0,i);
            vecs(8, i) = roots(1,i);
            vecs(9, i) = roots(2,i);
        }
        
        for (int j = 0; j < n_roots; j++) {
            vecs.block<7,1>(0,j) = nullSpace * roots.col(j);
        }
        
        
        homographies->clear();
        N_h->clear();
        int index = solver_common::extract_homographies_sh45<10, 16>(vecs, Rtransform1, Rtransform2, shift,
                homographies, N_h, q, p, c);
        if(test5thPt == 1){
            (*homographies)[0] = (*homographies)[index];
            homographies->resize(1);
            (*N_h)[0] = (*N_h)[index];
            N_h->resize(1);
            n_roots = 1;
        }
        return n_roots;
    }
}