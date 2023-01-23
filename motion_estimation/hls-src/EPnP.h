#include "Matrix.h"
#include <stdio.h>

class EPnP
{
private:
    int32_t number_of_correspondences;
    Matrix pws;
    Matrix pcs;
    Matrix us;
    Matrix alphas;
    FLOAT cx;
    FLOAT cy;
    FLOAT fx;
    FLOAT fy;
    uint32_t max_nr;
    FLOAT *A1, *A2;

    void gauss_newton(const Matrix &L_6x10, const Matrix &Rho, FLOAT betas[4]);

    FLOAT compute_R_and_t(const Matrix &ut, const FLOAT *betas,
                          Matrix &R, FLOAT t[3]);

public:
    EPnP(Matrix &opoint, Matrix &ipoint, Matrix &k);
    void compute(Matrix &rmat, Matrix &tvec);
    ~EPnP();
};
