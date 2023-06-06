#include "Matrix.h"

void merge_arrays(double in[MERGE_SIZE], int width, double out[MERGE_SIZE], int idxIn[MERGE_SIZE], int idxOut[MERGE_SIZE])
{
    int f1 = 0;
    int f2 = width;
    int i2 = width;
    int i3 = 2 * width;
    if (i2 >= MERGE_SIZE)
        i2 = MERGE_SIZE;
    if (i3 >= MERGE_SIZE)
        i3 = MERGE_SIZE;
merge_arrays:
    for (int i = 0; i < MERGE_SIZE; i++)
    {
#pragma HLS PIPELINE II=1
        double t1 = in[f1];
        int id1 = idxIn[f1];
        double t2 = (f2 == i3) ? 0 : in[f2];
        int id2 = (f2 == i3) ? 0 : idxIn[f2];
        if (f2 == i3 || (f1 < i2 && t1 >= t2))
        {
            out[i] = t1;
            idxOut[i] = id1;
            f1++;
        }
        else
        {
            out[i] = t2;
            idxOut[i] = id2;
            f2++;
        }
        if (f1 == i2 && f2 == i3)
        {
            f1 = i3;
            i2 += 2 * width;
            i3 += 2 * width;
            if (i2 >= MERGE_SIZE)
                i2 = MERGE_SIZE;
            if (i3 >= MERGE_SIZE)
                i3 = MERGE_SIZE;
            f2 = i2;
        }
    }
}

void merge_sort_parallel(double A[MERGE_SIZE], int outIdx[MERGE_SIZE])
{
    double temp[STAGES][MERGE_SIZE];
#pragma HLS ARRAY_PARTITION variable=temp type=complete dim=1
    int idxTemp[STAGES - 1][MERGE_SIZE];
    int inIdx[MERGE_SIZE];
    for (int i = 0; i < MERGE_SIZE; i++)
        inIdx[i] = i;
    int width = 1;
    merge_arrays(A, width, temp[0], inIdx, idxTemp[0]);
    width *= 2;
    for (int stage = 1; stage < STAGES - 1; stage++)
    {
#pragma HLS UNROLL
        merge_arrays(temp[stage - 1], width, temp[stage], idxTemp[stage - 1], idxTemp[stage]);
        width *= 2;
    }
    merge_arrays(temp[STAGES - 2], width, temp[STAGES - 1], idxTemp[STAGES - 2], outIdx);
}


void multrans(FLOAT* input, FLOAT* output, int row, int col)
{
    for (int i = 0; i < col; i++)
    {
        for (int j = i; j < col; j++)
        {
            for (int k = 0; k < row; k++)
            {
            	FLOAT a = input[k * col + i] * input[k * col + j];
                output[i * col + j] += a;
                if (i != j)
                    output[j * col + i] += a;
            }
        }
    }
}

void trans(FLOAT* input, double* output, int row, int col)
{
    for (int i = 0; i < row; i++)
        for (int j = 0; j < col; j++)
        	output[j*row+i] = (double)(input[i*col+j]);
}

void solve(FLOAT* A, FLOAT* x, FLOAT* b, int m, int n)
{
	double U[25];
	double W[6];
	double V[36];
	double At[30];

    for(int i=0; i<n; i++)
    {
    	x[i] = 0;
    }

	trans(A, At, m, n);
	xf::solver::gesvj<double, 5, 6, 1, 1>(n, m, At, U, W, V);

	int index[6];
	int U_index[5];
#pragma HLS ARRAY_PARTITION variable=U_index type=complete dim=1
#pragma HLS ARRAY_PARTITION variable=index type=complete dim=1
    merge_sort_parallel(W, index);


	int arr[5];
#pragma HLS ARRAY_PARTITION variable=arr type=complete dim=1

	for(int i=0; i<n; i++)
		arr[i] = index[i];

	for(int i=0; i<n; i++)
	{
    	for(int j=0; j<i; j++)
    	{
    		if(arr[j]>arr[i])
    		{
    			int temp = arr[j];
    			arr[j] = arr[i];
    			arr[i] = temp;
    		}
    	}
    }

	for(int i=0; i<n; i++)
	{
	    for(int j=0; j<n; j++)
	    {
	    	if(index[j] == arr[i])
	    	{
	    		U_index[j] = i;
	    	}
	    }
	}


    for (int i = 0; i < n; i++)
    {
        double wi = 1 / W[index[i]];
        double s = 0;

        for (int j = 0; j < m; j++)
        {
#pragma HLS PIPELINE off
        	double sum = s + V[j*m+index[i]] * (double)(b[j]);
            s = sum;
        }
        s *= wi;

        for (int j = 0; j < n; j++)
        {
        	double sum = (double)(x[j]) + s * U[j*n+U_index[i]];
            x[j] = (FLOAT)sum;
        }
    }
}
