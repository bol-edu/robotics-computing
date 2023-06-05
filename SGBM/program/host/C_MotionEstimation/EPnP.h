#include "Matrix.h"
#include <stdio.h>

class EPnP
{
private:
    FLOAT_t cx;
    FLOAT_t cy;
    FLOAT_t fx;
    FLOAT_t fy;
    int32_t number_of_correspondences;
    Matrix pws;
    Matrix pcs;
    Matrix us;
    Matrix alphas;
    uint32_t max_nr;
    FLOAT_t *A1, *A2;

    void gauss_newton(const Matrix &L_6x10, const Matrix &Rho, FLOAT_t betas[4]);

    FLOAT_t compute_R_and_t(const Matrix &ut, const FLOAT_t *betas,
                          Matrix &R, FLOAT_t t[3]);

public:
    EPnP(Matrix opoint, Matrix ipoint, FLOAT_t _fx, FLOAT_t _fy, FLOAT_t _cx, FLOAT_t _cy);
    void compute(Matrix &rmat, Matrix &tvec);
    ~EPnP();
};
