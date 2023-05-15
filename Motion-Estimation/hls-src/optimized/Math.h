#pragma once

#include "helper.h"
#include "gesvd.hpp"
#include <float.h>
#include <hls_math.h>

#define MERGE_SIZE 6
#define STAGES 3

FLOAT dot(FLOAT v1[3], FLOAT v2[3]);

void merge_arrays(double in[MERGE_SIZE], int width, double out[MERGE_SIZE], int idxIn[MERGE_SIZE], int idxOut[MERGE_SIZE]);

void merge_sort_parallel(double A[MERGE_SIZE], int outIdx[MERGE_SIZE]);

void multrans(FLOAT *input, FLOAT *output, int row, int col);

void trans(FLOAT *input, double *output, int row, int col);

void solve(FLOAT *A, FLOAT *x, FLOAT *b, int m, int n);

FLOAT norm(FLOAT *input, int len);
