#pragma once
#include <Eigen/Eigen>
#include "transform.h"
#include "parameterize_HN.h"
#include "common.h"
#include "extract_homographies.h"

namespace sh45_3
{
    inline MatrixXd solver_problem_sh45_3_o(const VectorXd& data)
    {
        //std::cout << "o" << std::endl;
        
        // Setup elimination template
        static const int coeffs0_ind[] = { 5,3,4,10,5,6,8,7,9,15,10,11,13,12,14,20,15,16,18,17,19,30,5,3,4,35,30,31,33,10,6,5,32,8,7,9,34,40,35,36,38,15,11,10,37,13,12,14,39,45,40,41,43,20,16,15,42,18,17,19,44,50,45,46,48,25,21,20,47,23,22,24,49,30,5,3,4,55,35,31,30,33,10,7,32,8,6,5,9,34,60,55,56,58,40,36,35,57,38,15,12,37,13,11,10,14,39,59,65,60,61,63,45,41,40,62,43,20,17,42,18,16,15,19,44,64,70,65,66,68,50,46,45,67,48,25,22,47,23,21,20,24,49,69,30,3,4,55,35,32,33,31,8,6,5,30,7,9,34,75,60,56,55,58,40,37,57,38,36,13,11,10,35,12,14,39,59,80,75,76,78,65,61,60,77,63,45,42,62,43,41,18,16,15,40,17,19,44,64,79,55,33,31,30,32,34,75,60,57,58,56,38,36,35,55,37,39,59,80,76,75,78,65,62,77,63,61,43,41,40,60,42,44,64,79,85,80,81,83,70,66,65,82,68,50,47,67,48,46,23,21,20,45,22,24,49,69,84,85,86,88,71,70,87,73,52,72,53,51,28,26,25,50,27,29,54,74,89 };
        static const int coeffs1_ind[] = { 102,104,93,91,92,94,78,76,75,77,79,75,58,56,55,57,59,80,77,78,76,63,61,60,75,62,64,79,92,93,91,83,81,80,82,84,94,91,93,85,82,92,83,81,68,66,65,80,67,69,84,94,91,93,85,81,80,92,83,70,67,82,68,66,48,46,45,65,47,49,69,84,94,102,98,96,97,99,104,97,102,98,96,88,86,85,87,89,99,104,96,102,98,87,97,88,86,73,71,70,85,72,74,89,99,104,96,98,86,85,97,88,72,87,73,71,53,51,50,70,52,54,74,89,99 };
        static const int C0_ind[] = { 0,3,22,23,24,25,26,30,45,46,47,48,49,53,68,69,70,71,72,76,91,92,96,100,113,115,116,117,118,119,120,121,122,123,126,136,137,138,139,140,141,142,143,144,145,146,149,159,160,161,162,163,164,165,166,167,168,169,172,182,183,184,185,186,187,188,189,190,191,192,195,205,206,211,216,219,227,230,234,235,236,238,239,240,241,242,243,247,250,251,253,254,255,256,257,258,259,260,261,262,263,264,265,266,270,273,274,275,276,277,278,279,280,281,282,283,284,285,286,287,288,289,293,296,297,298,299,300,301,302,303,304,305,306,307,308,309,310,311,312,316,319,320,321,331,336,341,349,354,355,357,358,359,360,361,362,363,364,365,368,372,373,374,376,377,378,379,380,381,382,383,384,385,386,387,388,389,391,392,393,394,395,396,397,398,399,400,401,402,403,404,405,406,407,408,409,410,411,412,413,423,428,429,430,432,433,441,446,447,449,450,451,452,453,454,455,456,457,464,465,466,468,469,470,471,472,473,474,475,476,477,478,479,480,481,483,484,485,486,487,488,489,490,491,492,493,494,495,496,497,498,499,500,501,502,503,504,505,507,508,509,511,512,513,514,516,517,518,519,520,521,522,523,524,525,526,527,528 } ;
        static const int C1_ind[] = { 18,19,37,38,41,42,60,61,62,64,65,78,83,84,85,87,88,101,102,104,105,106,107,108,109,110,111,112,125,127,128,129,130,131,133,134,135,143,146,147,148,149,150,151,152,153,154,155,156,157,158,159,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,194,198,199,202,203,204,217,218,219,220,221,222,223,225,226,227,228,235,237,238,240,241,242,243,244,245,246,247,248,249,250,251,252,255,256,258,259,260,261,263,264,265,266,267,268,269,270,271,272,273,274,275 };
        
        Eigen::Matrix<double,23,23> C0; C0.setZero();
        Eigen::Matrix<double,23,12> C1; C1.setZero();
        for (int i = 0; i < 278; i++) { C0(C0_ind[i]) = data(coeffs0_ind[i]); }
        for (int i = 0; i < 125; i++) { C1(C1_ind[i]) = data(coeffs1_ind[i]); }
        
        Eigen::Matrix<double,23,12> C12 = C0.partialPivLu().solve(C1);
        
        
        
        
// Setup action matrix
        Eigen::Matrix<double,16, 12> RR;
        RR.block<4, 12>(0, 0) = -C12.bottomRows(4);
        RR.block<12, 12>(4, 0).setIdentity();
        //RR << -C12.bottomRows(4), Matrix<double,12,12>::Identity(12, 12);
        
        static const int AM_ind[] = { 12,9,8,0,1,10,11,2,13,14,15,3 };
        Eigen::Matrix<double, 12, 12> AM;
        for (int i = 0; i < 12; i++) {
            AM.row(i) = RR.row(AM_ind[i]);
        }        
        
        //std::cout << "AM:" << std::endl << AM << std::endl << std::endl;

        // Solve eigenvalue problem
        Eigen::EigenSolver<Eigen::Matrix<double, 12, 12> > es(AM);
        Eigen::ArrayXcd D = es.eigenvalues();
        Eigen::ArrayXXcd V = es.eigenvectors();
        V = (V / V.row(0).array().replicate(12, 1)).eval();
        
        
        Eigen::Matrix<double, 3, 12> solsReal;
        Eigen::Matrix<std::complex<double>, 2, 12> solsComplex;
        solsComplex.setZero();
        solsReal.setZero();
        

        solsComplex.row(0) = V.row(1).array();
        solsComplex.row(1) = D.transpose().array();

        solver_common::complex_sort(solsComplex);
        //std::cout << "solutions" << std::endl << solsComplex << std::endl << std::endl;

        solsReal.block<2, 12>(0, 0) = solsComplex.real();
        solsReal.row(2).setOnes();
        
        /*for (int j = 0; j < 12; j++) {
            for (int k = 0; k < 2; k++) {
                solsReal(k,j) = solsComplex(k,j).real();
            }
            
        }
        
        
        for (int j = 0; j < 12; j++) {
            solsReal(2,j) = 1.0;
        }*/
        
        
        
        return solsReal ;
    }
    
    inline int solver_eigen_optimized(Eigen::Matrix<double, 3, 5> q, Eigen::Matrix<double, 3, 5> p,
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
        coeffsTemp /= coeffsTemp.norm();

        Eigen::Map<Eigen::Matrix<double, 105, 1>> data(coeffsTemp.data(), 105);
        
        //VectorXd data(105);
        //// Quick hack to map matrix to vector
        //int tempInd = 0;
        //for(int i=0;i<21;i++) {
        //    for(int j=0; j<5; j++) {
        //        data(tempInd) = coeffsTemp(j,i);
        //        tempInd = tempInd+1;
        //    }
        //}
        
        // We execute the GB solver to extract the roots
        Eigen::MatrixXd roots = solver_problem_sh45_3_o(data);
        
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