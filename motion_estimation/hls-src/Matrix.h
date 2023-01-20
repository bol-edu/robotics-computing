#pragma once
#include <iostream>
#include <string.h>

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

    void releaseMemory();

    // print matrix to stream
    friend std::ostream &operator<<(std::ostream &out, const Matrix &M);

    // direct data access
    FLOAT **val;
    int32_t r, c;

private:
    void allocateMemory(const int32_t r_, const int32_t c_);
};