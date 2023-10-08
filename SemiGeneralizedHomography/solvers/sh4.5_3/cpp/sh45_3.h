#pragma once
#include <Eigen/Eigen>
#include "transform.h"
#include "parameterize_HN.h"

#include "extract_homographies.h"

#include <chrono>

using namespace Eigen;

namespace sh45_3
{
    
    inline MatrixXd solver_problem_sh45_3(const VectorXd& data)
    {
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
        static const int coeffs0_ind[] = { 0,56,76,1,0,17,57,36,77,2,1,18,58,37,78,3,2,19,59,38,79,5,0,56,76,6,5,22,62,1,17,0,41,57,36,77,82,7,6,23,63,2,18,1,42,58,37,78,83,8,7,24,64,3,19,2,43,59,38,79,84,9,8,25,65,4,20,3,44,60,39,80,85,5,0,56,76,10,6,22,5,62,1,36,41,57,17,0,77,82,11,10,27,67,7,23,6,46,63,2,37,42,58,18,1,78,83,87,12,11,28,68,8,24,7,47,64,3,38,43,59,19,2,79,84,88,13,12,29,69,9,25,8,48,65,4,39,44,60,20,3,80,85,89,5,56,76,10,6,41,62,22,57,17,0,5,36,77,82,14,11,27,10,67,7,42,46,63,23,58,18,1,6,37,78,83,87,15,14,31,71,12,28,11,50,68,8,43,47,64,24,59,19,2,7,38,79,84,88,91,10,62,22,5,41,82,14,11,46,67,27,63,23,6,10,42,83,87,15,31,14,71,12,47,50,68,28,64,24,7,11,43,84,88,91,16,15,32,72,13,29,12,51,69,9,44,48,65,25,60,20,3,8,39,80,85,89,92,16,33,73,30,13,52,70,45,49,66,26,61,21,4,9,40,81,86,90,93 };
        static const int coeffs1_ind[] = { 55,96,74,34,53,94,71,31,14,50,91,14,67,27,10,46,87,15,50,71,31,68,28,11,14,47,88,91,53,74,34,72,32,15,51,92,94,34,74,16,51,53,72,32,69,29,12,15,48,89,92,94,34,74,16,32,15,53,72,13,48,51,69,29,65,25,8,12,44,85,89,92,94,55,75,35,54,95,96,54,55,75,35,73,33,16,52,93,95,96,35,55,75,52,54,73,33,70,30,13,16,49,90,93,95,96,35,75,33,16,54,73,49,52,70,30,66,26,9,13,45,86,90,93,95 };
        static const int C0_ind[] = { 0,3,22,23,24,25,26,30,45,46,47,48,49,53,68,69,70,71,72,76,91,92,96,100,113,115,116,117,118,119,120,121,122,123,126,136,137,138,139,140,141,142,143,144,145,146,149,159,160,161,162,163,164,165,166,167,168,169,172,182,183,184,185,186,187,188,189,190,191,192,195,205,206,211,216,219,227,230,234,235,236,238,239,240,241,242,243,247,250,251,253,254,255,256,257,258,259,260,261,262,263,264,265,266,270,273,274,275,276,277,278,279,280,281,282,283,284,285,286,287,288,289,293,296,297,298,299,300,301,302,303,304,305,306,307,308,309,310,311,312,316,319,320,321,331,336,341,349,354,355,357,358,359,360,361,362,363,364,365,368,372,373,374,376,377,378,379,380,381,382,383,384,385,386,387,388,389,391,392,393,394,395,396,397,398,399,400,401,402,403,404,405,406,407,408,409,410,411,412,413,423,428,429,430,432,433,441,446,447,449,450,451,452,453,454,455,456,457,464,465,466,468,469,470,471,472,473,474,475,476,477,478,479,480,481,483,484,485,486,487,488,489,490,491,492,493,494,495,496,497,498,499,500,501,502,503,504,505,507,508,509,511,512,513,514,516,517,518,519,520,521,522,523,524,525,526,527,528 } ;
        static const int C1_ind[] = { 18,19,37,38,41,42,60,61,62,64,65,78,83,84,85,87,88,101,102,104,105,106,107,108,109,110,111,112,125,127,128,129,130,131,133,134,135,143,146,147,148,149,150,151,152,153,154,155,156,157,158,159,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,194,198,199,202,203,204,217,218,219,220,221,222,223,225,226,227,228,235,237,238,240,241,242,243,244,245,246,247,248,249,250,251,252,255,256,258,259,260,261,263,264,265,266,267,268,269,270,271,272,273,274,275 };
        
        Matrix<double,23,23> C0; C0.setZero();
        Matrix<double,23,12> C1; C1.setZero();
        for (int i = 0; i < 278; i++) { C0(C0_ind[i]) = coeffs(coeffs0_ind[i]); }
        for (int i = 0; i < 125; i++) { C1(C1_ind[i]) = coeffs(coeffs1_ind[i]); }
        
        Matrix<double,23,12> C12 = C0.partialPivLu().solve(C1);
        
        
        
        
        // Setup action matrix
        Matrix<double,16, 12> RR;
        RR << -C12.bottomRows(4), Matrix<double,12,12>::Identity(12, 12);
        
        static const int AM_ind[] = { 12,9,8,0,1,10,11,2,13,14,15,3 };
        Matrix<double, 12, 12> AM;
        for (int i = 0; i < 12; i++) {
            AM.row(i) = RR.row(AM_ind[i]);
        }
        
        
        Matrix<double, 3, 12> solsReal;
        Matrix<std::complex<double>, 2, 12> solsComplex;
        solsComplex.setZero();
        solsReal.setZero();
        
        
        EigenSolver<Matrix<double, 12, 12> > es(AM);
        ArrayXcd D = es.eigenvalues();
        ArrayXXcd V = es.eigenvectors();
        V = (V / V.row(0).array().replicate(12, 1)).eval();
        
        
        solsComplex.row(0) = V.row(1).array();
        solsComplex.row(1) = D.transpose().array();
        
        for (int j = 0; j < 12; j++) {
            for (int k = 0; k < 2; k++) {
                solsReal(k,j) = solsComplex(k,j).real();
            }
            
        }
        
        
        for (int j = 0; j < 12; j++) {
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
        int n_roots = 12;
        // Nullspace
        Eigen::Matrix<double, 7,3> nullSpace;
        Eigen::Matrix<double, 5, 21> coeffsTemp = getCoeffs(q, p, c, nullSpace);
        
        // Normalizing each equation's coefficients
        for(int j=0; j<5; j++) {
            coeffsTemp.row(j) = coeffsTemp.row(j)/coeffsTemp.row(j).norm();
        }
        
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
        MatrixXd roots = solver_problem_sh45_3(data);
        
        Eigen::Matrix<double, 10, Eigen::Dynamic, 0, 10, 12> vecs(10, n_roots);
        
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
        // Once we know the roots, we extract the scale of Homography matrix, the homography matrix
        // and the plane normal vector
        int index = solver_common::extract_homographies_sh45<10, 12>(vecs, Rtransform1, Rtransform2, shift, homographies, N_h, q, p, c);
        if(test5thPt == 1){
            homographies->erase(homographies->begin(), homographies->begin() + index);
            homographies->erase(homographies->begin()+(index+1), homographies->begin()+n_roots);
            N_h->erase(N_h->begin(), N_h->begin() + index);
            N_h->erase(N_h->begin()+(index+1), N_h->begin() + n_roots);
            n_roots = 1;
        }
        return n_roots;
    }
}