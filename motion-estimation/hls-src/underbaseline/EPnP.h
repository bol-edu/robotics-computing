#ifndef __EPNP_H__
#define __EPNP_H__

#include "Matrix.h"

#define MODEL_POINTS 5

void gauss_newton(const Matrix<FLOAT, 6, 10> &L_6x10,
                  const Matrix<FLOAT, 6, 1> &Rho, FLOAT betas[4]);

class EPnP
{
private:
    FLOAT cx;
    FLOAT cy;
    FLOAT fx;
    FLOAT fy;
    Matrix<FLOAT, MODEL_POINTS, 3> pws;
    Matrix<FLOAT, MODEL_POINTS, 3> pcs;
    Matrix<FLOAT, MODEL_POINTS, 2> us;
    Matrix<FLOAT, MODEL_POINTS, 4> alphas;

    Matrix<FLOAT, 4, 3> cws;
    Matrix<FLOAT, MODEL_POINTS, 3> PW0;
    Matrix<FLOAT, 3, 3> PW0tPW0;
    Matrix<FLOAT, 3, 1> DC;
    Matrix<FLOAT, 3, 3> UC;
    Matrix<FLOAT, 3, 3> VC;
    Matrix<FLOAT, 3, 3> CC;
    Matrix<FLOAT, 3, 3> CC_inv;
    Matrix<FLOAT, 2 * MODEL_POINTS, 12> M;
    Matrix<FLOAT, 12, 12> MtM;
    Matrix<FLOAT, 12, 1> DM;
    Matrix<FLOAT, 12, 12> UM;
    Matrix<FLOAT, 12, 12> VM;
    // Matrix<FLOAT, 6, 10> L_6x10;
    //  Matrix<FLOAT, 6, 1> Rho;
    Matrix<FLOAT, 6, 3> dv[4];
    // Matrix<FLOAT, 4, 4> Betas;
    Matrix<FLOAT, 3, 3> Rs[4];
    Matrix<FLOAT, 4, 3> ts;
    Matrix<FLOAT, 6, 4> L_6x4;
    Matrix<FLOAT, 4, 1> B4;
    Matrix<FLOAT, 6, 3> L_6x3;
    Matrix<FLOAT, 3, 1> B3;
    Matrix<FLOAT, 6, 5> L_6x5;
    Matrix<FLOAT, 5, 1> B5;

    FLOAT compute_R_and_t(const Matrix<FLOAT, 12, 12> &u, const FLOAT *betas,
                          Matrix<FLOAT, 3, 3> &R, FLOAT t[3]);

public:
    Matrix<FLOAT, 3, 3> rmat;
    Matrix<FLOAT, 3, 1> tvec;

    EPnP(FLOAT opoint[MODEL_POINTS][3],
         FLOAT ipoint[MODEL_POINTS][2],
         FLOAT _fx, FLOAT _fy, FLOAT _cx, FLOAT _cy);

    void compute();
};

#endif
