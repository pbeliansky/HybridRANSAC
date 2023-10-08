#include "../../solvers/sh5f_3/cpp/sh5f_3_cf.h"
#include <chrono>

#include "mex.h"
// #include "mexAdapter.hpp"
#include <Eigen/Eigen>
using namespace Eigen;




void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {

    if (nrhs != 1) {
        mexErrMsgTxt("One input argument required.");
    }
    if (nlhs != 3) {
        mexErrMsgTxt("Four output arguments needed.");
    }
    double const* const matrixAux = static_cast<double const*>(mxGetData(prhs[0]));
    const mwSize* sizeInputMatrix = mxGetDimensions(prhs[0]);
    double* xX = (double*)malloc(sizeInputMatrix[0] * sizeInputMatrix[1] * sizeof(double));
    double* pindH, * pindf, * pindN, * pindb;
    const int size0 = sizeInputMatrix[0]; 
    const int size1 = sizeInputMatrix[1];

    for (int k = 0; k < size0; k++)
    {
        int kOffset = k * size1; 
        for (int i = 0; i < size1; i++)
        {
            int iOffset = i * size0;
            xX[i + kOffset] = matrixAux[i + kOffset];
        }
    }

    Matrix<double, 3, 5> q, p, c;


    q << xX[0], xX[1], xX[2], xX[3], xX[4], xX[5],
        xX[6], xX[7], xX[8], xX[9], xX[10], xX[11],
        xX[12], xX[13], xX[14];
    p << xX[15], xX[16], xX[17], xX[18], xX[19], xX[20],
        xX[21], xX[22], xX[23], xX[24], xX[25], xX[26],
        xX[27], xX[28], xX[29];
    c << xX[30], xX[31], xX[32], xX[33], xX[34], xX[35],
        xX[36], xX[37], xX[38], xX[39], xX[40], xX[41],
        xX[42], xX[43], xX[44];

    //     for (int k = 0; k < 3; k++) {
    //         for (int j = 0; j < 5; j++) {
    //             mexPrintf(" %f ", p(k,j));
    //         }
    //         mexPrintf(" \n");
    //     }

    std::vector<double> fss;
    std::vector<Eigen::Matrix3d> Hs;
    std::vector<Eigen::Vector3d> Nss;
    int noOfSol = sh5f_3::solver_closed_form(q, p, c, &fss, &Hs, &Nss);
    plhs[0] = mxCreateDoubleMatrix(3, noOfSol * 3, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(3, noOfSol, mxREAL);
    plhs[2] = mxCreateDoubleMatrix(1, noOfSol, mxREAL);

    pindH = mxGetPr(plhs[0]);
    pindN = mxGetPr(plhs[1]);
    pindf = mxGetPr(plhs[2]);

    //
    for (int i = 0; i < noOfSol; i++) {
        for (int k = 0; k < 3; k++) {
            for (int j = 0; j < 3; j++) {
                pindH[(k)+3 * j + 9 * i] = Hs[i](k, j);
            }
        }
    }
    //
    for (int i = 0; i < noOfSol; i++) {
        //         mexPrintf(" %f ", fss[i]);
        pindf[i] = fss[i];
        for (int j = 0; j < 3; j++) {


            pindN[j + 3 * i] = Nss[i](j);
        }

    }
}