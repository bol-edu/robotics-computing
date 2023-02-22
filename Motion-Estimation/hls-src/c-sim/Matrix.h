#pragma once
#ifndef __MAT_H__
#define __MAT_H__

typedef signed int int32_t;
typedef double FLOAT; // double precision

class Matrix
{
public:
    // init empty 0x0 matrix
    Matrix();
    // init empty rxc matrix
    Matrix(const int32_t r, const int32_t c);
    // init rxc matrix with values from array 'val'
    Matrix(const int32_t r, const int32_t c, const FLOAT *val_);

    ~Matrix();

    // assignment operator, copies contents of M
    Matrix &operator=(const Matrix &M);

    // set or get submatrices of current matrix
    Matrix getMat(int32_t i1, int32_t j1, int32_t i2 = -1, int32_t j2 = -1);
    void setMat(const Matrix &M, const int32_t i, const int32_t j);

    // create identity matrix
    static Matrix eye(const int32_t m);

    // simple arithmetic operations

    // dot product of 2 vectors
    static FLOAT dot(const FLOAT *v1, const FLOAT *v2);
    Matrix operator+(const Matrix &M); // add matrix
    Matrix operator-(const Matrix &M); // subtract matrix
    Matrix operator*(const Matrix &M); // multiply with matrix
    Matrix operator*(const FLOAT &s);  // multiply with scalar

    // muliply transpose MtM
    Matrix multrans();

    // transpose
    Matrix trans();

    // complex arithmetic operations
    Matrix inv(); // invert matrix M

    // solve linear system M*x=B, replaces *this and M
    bool solve(const Matrix &M, FLOAT eps = 1e-20);

    // solve nonlinear equation 'Ax = b' by SVD
    static void solve(Matrix &A, Matrix &x, Matrix &b);
    void svd(Matrix &U, Matrix &W, Matrix &V); // singular value decomposition *this = U*diag(W)*V^T

    void releaseMemory();

    // direct data access
    FLOAT **val;
    int32_t m, n;

private:
    void allocateMemory(const int32_t r_, const int32_t c_);
    inline FLOAT pythag(FLOAT a, FLOAT b);
};

#endif
