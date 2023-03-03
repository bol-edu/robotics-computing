#pragma once
#include <iostream>
#include <string.h>
#include <math.h>

using namespace std;

// typedef __int32 int32_t;
typedef double FLOAT_t; // double precision

class Matrix
{
public:
    // init empty 0x0 matrix
    Matrix();
    // init empty rxc matrix
    Matrix(const int32_t r, const int32_t c);
    // init rxc matrix with values from array 'val'
    Matrix(const int32_t r, const int32_t c, const FLOAT_t *val_);

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
    static FLOAT_t dot(const FLOAT_t *v1, const FLOAT_t *v2);
    Matrix operator+(const Matrix &M); // add matrix
    Matrix operator-(const Matrix &M); // subtract matrix
    Matrix operator*(const Matrix &M); // multiply with matrix
    Matrix operator*(const FLOAT_t &s);  // multiply with scalar
    // Matrix operator/(const Matrix &M); // divide elementwise by matrix (or vector)
    // Matrix operator/(const FLOAT_t &s);  // divide by scalar
    // Matrix operator-();                // negative matrix

    // muliply transpose MtM
    Matrix multrans();

    // transpose
    Matrix trans();
    // FLOAT_t l2norm();                    // euclidean norm (vectors) / frobenius norm (matrices)
    // FLOAT_t mean();                      // mean of all elements in matrix

    // complex arithmetic operations
    // static Matrix cross(const Matrix &a, const Matrix &b); // cross product of two vectors
    Matrix inv(); // invert matrix M
    // bool inv();                                            // invert this matrix

    // returns determinant of matrix
    // FLOAT_t det();

    // solve linear system M*x=B, replaces *this and M
    bool solve(const Matrix &M, FLOAT_t eps = 1e-20);

    // solve nonlinear equation 'Ax = b' by SVD
    static void solve(Matrix &A, Matrix &x, Matrix &b);
    // bool lu(int32_t *idx, FLOAT_t &d, FLOAT_t eps = 1e-20);    // replace *this by lower upper decomposition
    void svd(Matrix &U, Matrix &W, Matrix &V); // singular value decomposition *this = U*diag(W)*V^T

    void releaseMemory();

    // print matrix to stream
    friend std::ostream &operator<<(std::ostream &out, const Matrix &M);

    // direct data access
    FLOAT_t **val;
    int32_t m, n;

private:
    void allocateMemory(const int32_t r_, const int32_t c_);
    inline FLOAT_t pythag(FLOAT_t a, FLOAT_t b);
};