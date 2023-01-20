#pragma once
#include <iostream>
#include <string.h>
#include <math.h>

using namespace std;

// typedef __int32 int32_t;
typedef double FLOAT; // double precision

class Matrix
{
public:
    Matrix();                                                    // init empty 0x0 matrix
    Matrix(const int32_t r, const int32_t c);                    // init empty rxc matrix
    Matrix(const int32_t r, const int32_t c, const FLOAT *val_); // init rxc matrix with values from array 'val'
    Matrix(const Matrix &M);                                     // creates deepcopy of M
    ~Matrix();

    // set or get submatrices of current matrix
    Matrix getMat(int32_t i1, int32_t j1, int32_t i2 = -1, int32_t j2 = -1);
    void setMat(const Matrix &M, const int32_t i, const int32_t j);

    // create identity matrix
    // static Matrix eye(const int32_t m);

    // simple arithmetic operations
    // Matrix operator+(const Matrix &M); // add matrix
    // Matrix operator-(const Matrix &M); // subtract matrix
    Matrix operator*(const Matrix &M); // multiply with matrix
    // Matrix operator*(const FLOAT &s);  // multiply with scalar
    // Matrix operator/(const Matrix &M); // divide elementwise by matrix (or vector)
    // Matrix operator/(const FLOAT &s);  // divide by scalar
    // Matrix operator-();                // negative matrix
    Matrix multrans(); // muliply transpose MtM
    Matrix trans();    // transpose
    // FLOAT l2norm();                    // euclidean norm (vectors) / frobenius norm (matrices)
    // FLOAT mean();                      // mean of all elements in matrix

    // complex arithmetic operations
    // static Matrix cross(const Matrix &a, const Matrix &b); // cross product of two vectors
    static Matrix inv(const Matrix &M); // invert matrix M
    // bool inv();                                            // invert this matrix
    // FLOAT det();                                           // returns determinant of matrix
    bool solve(const Matrix &M, FLOAT eps = 1e-20); // solve linear system M*x=B, replaces *this and M
    // bool lu(int32_t *idx, FLOAT &d, FLOAT eps = 1e-20);    // replace *this by lower upper decomposition
    void svd(Matrix &U, Matrix &W, Matrix &V); // singular value decomposition *this = U*diag(W)*V^T

    void releaseMemory();

    // print matrix to stream
    friend std::ostream &operator<<(std::ostream &out, const Matrix &M);

    // direct data access
    FLOAT **val;
    int32_t m, n;

private:
    void allocateMemory(const int32_t r_, const int32_t c_);
    inline FLOAT pythag(FLOAT a, FLOAT b);
};