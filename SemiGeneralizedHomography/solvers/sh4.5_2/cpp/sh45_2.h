#pragma once
#include <Eigen/Eigen>
#include "transform.h"
#include "parameterize_HN.h"

#include "extract_homographies.h"

#include <chrono>

using namespace Eigen;

namespace sh45_2
{
    
    inline MatrixXd solver_problem_sh45_2(const VectorXd& data)
    {
        
        // Compute coefficients
        const double* d = data.data();
        VectorXd coeffs(97);
        coeffs[0] = d[5];
        coeffs[1] = d[10];
        coeffs[2] = d[15];
        coeffs[3] = d[20];
        coeffs[4] = d[25];
        coeffs[5] = d[30];
        coeffs[6] = d[35];
        coeffs[7] = d[40];
        coeffs[8] = d[45];
        coeffs[9] = d[50];
        coeffs[10] = d[55];
        coeffs[11] = d[60];
        coeffs[12] = d[65];
        coeffs[13] = d[70];
        coeffs[14] = d[75];
        coeffs[15] = d[80];
        coeffs[16] = d[85];
        coeffs[17] = d[6];
        coeffs[18] = d[11];
        coeffs[19] = d[16];
        coeffs[20] = d[21];
        coeffs[21] = d[26];
        coeffs[22] = d[31];
        coeffs[23] = d[36];
        coeffs[24] = d[41];
        coeffs[25] = d[46];
        coeffs[26] = d[51];
        coeffs[27] = d[56];
        coeffs[28] = d[61];
        coeffs[29] = d[66];
        coeffs[30] = d[71];
        coeffs[31] = d[76];
        coeffs[32] = d[81];
        coeffs[33] = d[86];
        coeffs[34] = d[91];
        coeffs[35] = d[96];
        coeffs[36] = d[7];
        coeffs[37] = d[12];
        coeffs[38] = d[17];
        coeffs[39] = d[22];
        coeffs[40] = d[27];
        coeffs[41] = d[32];
        coeffs[42] = d[37];
        coeffs[43] = d[42];
        coeffs[44] = d[47];
        coeffs[45] = d[52];
        coeffs[46] = d[57];
        coeffs[47] = d[62];
        coeffs[48] = d[67];
        coeffs[49] = d[72];
        coeffs[50] = d[77];
        coeffs[51] = d[82];
        coeffs[52] = d[87];
        coeffs[53] = d[92];
        coeffs[54] = d[97];
        coeffs[55] = d[102];
        coeffs[56] = d[3];
        coeffs[57] = d[8];
        coeffs[58] = d[13];
        coeffs[59] = d[18];
        coeffs[60] = d[23];
        coeffs[61] = d[28];
        coeffs[62] = d[33];
        coeffs[63] = d[38];
        coeffs[64] = d[43];
        coeffs[65] = d[48];
        coeffs[66] = d[53];
        coeffs[67] = d[58];
        coeffs[68] = d[63];
        coeffs[69] = d[68];
        coeffs[70] = d[73];
        coeffs[71] = d[78];
        coeffs[72] = d[83];
        coeffs[73] = d[88];
        coeffs[74] = d[93];
        coeffs[75] = d[98];
        coeffs[76] = d[4];
        coeffs[77] = d[9];
        coeffs[78] = d[14];
        coeffs[79] = d[19];
        coeffs[80] = d[24];
        coeffs[81] = d[29];
        coeffs[82] = d[34];
        coeffs[83] = d[39];
        coeffs[84] = d[44];
        coeffs[85] = d[49];
        coeffs[86] = d[54];
        coeffs[87] = d[59];
        coeffs[88] = d[64];
        coeffs[89] = d[69];
        coeffs[90] = d[74];
        coeffs[91] = d[79];
        coeffs[92] = d[84];
        coeffs[93] = d[89];
        coeffs[94] = d[94];
        coeffs[95] = d[99];
        coeffs[96] = d[104];
        
        
        
        // Setup elimination template
        static const int coeffs0_ind[] = { 0,56,76,1,0,17,36,57,77,2,1,18,37,58,78,3,2,19,38,59,79,4,3,20,39,60,80,5,56,76,6,5,22,41,62,57,17,36,0,77,82,7,6,23,42,63,58,18,37,1,78,83,8,7,24,43,64,59,19,38,2,79,84,9,8,25,44,65,60,20,39,3,80,85,4,21,40,61,81 };
        static const int coeffs1_ind[] = { 55,96,74,34,53,94,71,31,50,14,91,14,67,27,46,10,87,10,62,22,41,5,82,11,10,27,46,67,63,23,42,6,83,87,15,14,31,50,71,68,28,47,11,88,91,12,11,28,47,68,64,24,43,7,84,88,34,53,74,72,32,51,15,92,94,16,15,32,51,72,69,29,48,12,89,92,13,12,29,48,69,65,25,44,8,85,89,55,75,35,54,95,96,35,54,75,73,33,52,16,93,95,16,33,52,73,70,30,49,13,90,93,13,30,49,70,66,26,45,9,86,90,9,26,45,66,61,21,40,4,81,86 };
        static const int C0_ind[] = { 0,4,10,11,12,13,14,15,21,22,23,24,25,26,32,33,34,35,36,37,43,44,45,46,47,48,54,55,60,64,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,111,112,113,114,120 } ;
        static const int C1_ind[] = { 7,9,16,17,18,20,27,28,29,30,31,33,38,39,40,41,42,44,49,50,51,52,53,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,124,126,127,128,130,131,134,135,136,137,138,139,140,141,142,144,145,146,147,148,149,150,151,152,153,155,156,157,158,159,160,161,162,163,164,166,167,168,169,170,171,172,173,174,175 };
        
        Matrix<double,11,11> C0; C0.setZero();
        Matrix<double,11,16> C1; C1.setZero();
        for (int i = 0; i < 79; i++) { C0(C0_ind[i]) = coeffs(coeffs0_ind[i]); }
        for (int i = 0; i < 132; i++) { C1(C1_ind[i]) = coeffs(coeffs1_ind[i]); }
        
        Matrix<double,11,16> C12 = C0.partialPivLu().solve(C1);
        
        
        
        
        // Setup action matrix
        Matrix<double,21, 16> RR;
        RR << -C12.bottomRows(5), Matrix<double,16,16>::Identity(16, 16);
        
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
        
        for (int k = 0; k < 2; k++) {
            for (int j = 0; j < 16; j++) {
                solsReal(k,j) = solsComplex(k,j).real();
            }
        }
        for (int j = 0; j < 16; j++) {
            solsReal(2,j) = 1.0;
        }
        
        return solsReal ;
    }
    
    inline int solver_eigen(Eigen::Matrix<double, 3, 5> q, Eigen::Matrix<double, 3, 5> p,
            Eigen::Matrix<double, 3, 5> c, std::vector<Eigen::Matrix3d>* homographies,
            std::vector<Eigen::Vector3d>* N_h, int test5thPt)
    {
        Eigen::Matrix3d Rtransform1; Eigen::Matrix3d Rtransform2; Eigen::Vector3d shift;
        
        solver_common::transform(q, p, c, Rtransform1, Rtransform2, shift);
        int n_roots = 16;

        // Nullspace
        Eigen::Matrix<double, 7,3> nullSpace;
        Eigen::Matrix<double, 5, 21> coeffsTemp = get_coeffs(q, p, c, nullSpace);
        
        
        // Normalizing each equation's coefficients
        for(int j=0; j<5; j++) {
            coeffsTemp.row(j) = coeffsTemp.row(j)/coeffsTemp.row(j).norm();
        }
        coeffsTemp = coeffsTemp/coeffsTemp.norm();
        VectorXd data(105);
        // Quick hack to map matrix to vector
        int tempInd = 0;
        for(int i=0;i<21;i++) {
            for(int j=0; j<5; j++) {
                data(tempInd) = coeffsTemp(j,i);
                tempInd = tempInd+1;
            }
        }
        
        // We execute the GB solver to extract the roots
        MatrixXd roots = solver_problem_sh45_2(data);
        
        
        
        Eigen::Matrix<double, 10, Eigen::Dynamic, 0, 10, 16> vecs(10, n_roots);
        
        for (size_t i = 0; i < n_roots; ++i){
            vecs(7, i) = roots(0,i);
            vecs(8, i) = roots(1,i);
            vecs(9, i) = roots(2,i);
        }
        
        for (int j = 0; j < n_roots; j++) {
            vecs.block(0,j,7,1) = nullSpace * roots.col(j);
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