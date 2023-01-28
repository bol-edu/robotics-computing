#include "Matrix.h"

class EPnP
{
private:
    FLOAT cx;
    FLOAT cy;
    FLOAT fx;
    FLOAT fy;
    int32_t number_of_correspondences;
    Matrix pws;
    Matrix pcs;
    Matrix us;
    Matrix alphas;
    int32_t max_nr;

    void gauss_newton(const Matrix &L_6x10, const Matrix &Rho, FLOAT betas[4]);

    FLOAT compute_R_and_t(const Matrix &ut, const FLOAT *betas,
                          Matrix &R, FLOAT t[3]);

public:
    EPnP(Matrix opoint, Matrix ipoint, FLOAT _fx, FLOAT _fy, FLOAT _cx, FLOAT _cy);
    void compute(Matrix &rmat, Matrix &tvec);
    ~EPnP();
};
