#include "Matrix.h"

Matrix::Matrix()
{
    r = 0;
    c = 0;
    val = 0;
}

Matrix::Matrix(const int32_t r_, const int32_t c_)
{
    allocateMemory(r_, c_);
}

Matrix::Matrix(const int32_t r_, const int32_t c_, const FLOAT *val_)
{
    allocateMemory(r_, c_);
    int32_t k = 0;
    for (int32_t i = 0; i < r_; i++)
        for (int32_t j = 0; j < c_; j++)
            val[i][j] = val_[k++];
}

Matrix::Matrix(const Matrix &M)
{
    allocateMemory(M.r, M.c);
    for (int32_t i = 0; i < M.r; i++)
        memcpy(val[i], M.val[i], M.c * sizeof(FLOAT));
}

Matrix::~Matrix()
{
    // releaseMemory();
}

void Matrix::allocateMemory(const int32_t r_, const int32_t c_)
{
    r = abs(r_);
    c = abs(c_);
    if (r == 0 || c == 0)
    {
        val = 0;
        return;
    }
    val = (FLOAT **)malloc(r * sizeof(FLOAT *));
    val[0] = (FLOAT *)calloc(r * c, sizeof(FLOAT));
    for (int32_t i = 1; i < r; i++)
        val[i] = val[i - 1] + c;
}

void Matrix::releaseMemory()
{
    if (val != 0)
    {
        free(val[0]);
        free(val);
    }
}

ostream &operator<<(ostream &out, const Matrix &M)
{
    if (M.r == 0 || M.c == 0)
    {
        out << "[empty matrix]";
    }
    else
    {
        char buffer[1024];
        for (int32_t i = 0; i < M.r; i++)
        {
            sprintf(buffer, "%d\t", i);
            out << buffer;
            for (int32_t j = 0; j < M.c; j++)
            {
                sprintf(buffer, "%12.7f ", M.val[i][j]);
                out << buffer;
            }
            if (i < M.r - 1)
                out << endl;
        }
    }
    return out;
}