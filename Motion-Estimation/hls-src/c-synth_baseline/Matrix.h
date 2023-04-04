#pragma once
#ifndef __MAT_H__
#define __MAT_H__

#include <string.h>
#include <math.h>
#include <stdio.h>
#include "gesvd.hpp"

#define MAX_KEYPOINT_NUM 500
#define IMAGE_HEIGTH 376
#define IMAGE_WIDTH 1241
#define MERGE_SIZE 6
#define STAGES 3

typedef signed int int32_t;
typedef float FLOAT; // FLOAT precision

#define SIGN(a, b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
static FLOAT sqrarg;
#define SQR(a) ((sqrarg = (a)) == 0.0 ? 0.0 : sqrarg * sqrarg)
#define MAX(a, b) (a > b ? a : b)
#define MIN(a, b) (a < b ? a : b)

struct OPOINT
{
	FLOAT x;
	FLOAT y;
	FLOAT z;
};

struct IPOINT
{
	FLOAT x;
	FLOAT y;
};

static FLOAT pythag(FLOAT a, FLOAT b)
{
    FLOAT absa, absb;
    absa = fabs(a);
    absb = fabs(b);
    if (absa > absb)
        return absa * sqrt(1.0 + SQR(absb / absa));
    else
        return (absb == 0.0 ? 0.0 : absb * sqrt(1.0 + SQR(absa / absb)));
}


static FLOAT dot(FLOAT v1[3], FLOAT v2[3])
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

static void SWAP(FLOAT &a, FLOAT &b)
{
    FLOAT temp = a;
    a = b;
    b = temp;
}

template <int ROW>
void solve(FLOAT B[ROW][ROW], FLOAT A[ROW][ROW])
{
	FLOAT eps = 1e-20;
    // index vectors for bookkeeping on the pivoting
    int indxc[ROW]; //*indxc = new int[m];
    int indxr[ROW]; //*indxr = new int[m];
    int ipiv[ROW];  //*ipiv = new int[m];

    // loop variables
    int i, icol, irow, j, k, l, ll;
    FLOAT big, dum, pivinv, temp;

    // initialize pivots to zero
    for (j = 0; j < ROW; j++)
        ipiv[j] = 0;

    // main loop over the columns to be reduced
    for (i = 0; i < ROW; i++)
    {

        big = 0.0;

        // search for a pivot element
        for (j = 0; j < ROW; j++)
            if (ipiv[j] != 1)
                for (k = 0; k < ROW; k++)
                    if (ipiv[k] == 0)
                        if (fabs(A[j][k]) >= big)
                        {
                            big = fabs(A[j][k]);
                            irow = j;
                            icol = k;
                        }
        ++(ipiv[icol]);

        // We now have the pivot element, so we interchange rows, if needed, to put the pivot
        // element on the diagonal. The columns are not physically interchanged, only relabeled.
        if (irow != icol)
        {
            for (l = 0; l < ROW; l++)
                SWAP(A[irow][l], A[icol][l]);
            for (l = 0; l < ROW; l++)
                SWAP(B[irow][l], B[icol][l]);
        }

        indxr[i] = irow; // We are now ready to divide the pivot row by the
        indxc[i] = icol; // pivot element, located at irow and icol.

        // check for singularity
        if (fabs(A[icol][icol]) < eps)
        {
            memset(indxc, 0, ROW);
            memset(indxr, 0, ROW);
            memset(ipiv, 0, ROW);
            return;
        }

        pivinv = 1.0 / A[icol][icol];
        A[icol][icol] = 1.0;
        for (l = 0; l < ROW; l++)
            A[icol][l] *= pivinv;
        for (l = 0; l < ROW; l++)
            B[icol][l] *= pivinv;

        // Next, we reduce the rows except for the pivot one
        for (ll = 0; ll < ROW; ll++)
            if (ll != icol)
            {
                dum = A[ll][icol];
                A[ll][icol] = 0.0;
                for (l = 0; l < ROW; l++)
                    A[ll][l] -= A[icol][l] * dum;
                for (l = 0; l < ROW; l++)
                    B[ll][l] -= B[icol][l] * dum;
            }
    }

    // This is the end of the main loop over columns of the reduction. It only remains to unscramble
    // the solution in view of the column interchanges. We do this by interchanging pairs of
    // columns in the reverse order that the permutation was built up.
    for (l = ROW - 1; l >= 0; l--)
    {
        if (indxr[l] != indxc[l])
            for (k = 0; k < ROW; k++)
                SWAP(A[k][indxr[l]], A[k][indxc[l]]);
    }
}

template <int ROW>
void inv(FLOAT input[ROW][ROW], FLOAT output[ROW][ROW])
{
	for(int i=0; i<ROW; i++)
	{
		for(int j=0; j<ROW; j++)
		{
			if(i==j)
				output[i][j] = 1;
			else
				output[i][j] = 0;
		}
	}

    solve<ROW>(output, input);
}

void multrans(FLOAT* input, FLOAT* output, int row, int col);

void trans(FLOAT* input, FLOAT* output, int row, int col);

void solve(FLOAT* A, FLOAT* x, FLOAT* b, int m, int n);

void merge_arrays(double in[MERGE_SIZE], int width, double out[MERGE_SIZE], int idxIn[MERGE_SIZE], int idxOut[MERGE_SIZE]);
void merge_sort_parallel(double A[MERGE_SIZE], int outIdx[MERGE_SIZE]);


template <int ROW, int COL>
void trans(FLOAT input[ROW][COL], FLOAT output[COL][ROW])
{
    for (int i = 0; i < ROW; i++)
        for (int j = 0; j < COL; j++)
            output[j][i] = input[i][j];
}

template <int ROW, int COL>
void svd(FLOAT U[ROW][COL], FLOAT U2[ROW][ROW], FLOAT W[MIN(ROW, COL)], FLOAT V[COL][COL])
{
    FLOAT w[COL];
    FLOAT rv1[COL];

    int flag, i, its, j, jj, k, l, nm;
    FLOAT anorm, c, f, g, h, s, scale, x, y, z;

    g = scale = anorm = 0.0; // Householder reduction to bidiagonal form.
    for (i = 0; i < COL; i++)
    {
        l = i + 1;
        rv1[i] = scale * g;

        g = s = scale = 0.0;
        if (i < ROW)
        {
            for (k = i; k < ROW; k++)
            {
                scale += fabs(U[k][i]);
            }
            if (scale)
            {
                for (k = i; k < ROW; k++)
                {
                    U[k][i] /= scale;
                    s += U[k][i] * U[k][i];
                }
                f = U[i][i];
                g = -SIGN(sqrt(s), f);
                h = f * g - s;
                U[i][i] = f - g;
                for (j = l; j < COL; j++)
                {
                    for (s = 0.0, k = i; k < ROW; k++)
                        s += U[k][i] * U[k][j];
                    f = s / h;
                    for (k = i; k < ROW; k++)
                        U[k][j] += f * U[k][i];
                }
                for (k = i; k < ROW; k++)
                    U[k][i] *= scale;
            }
        }
        w[i] = scale * g;
        g = s = scale = 0.0;
        if (i < ROW && i != COL - 1)
        {
            for (k = l; k < COL; k++)
                scale += fabs(U[i][k]);
            if (scale)
            {
                for (k = l; k < COL; k++)
                {
                    U[i][k] /= scale;
                    s += U[i][k] * U[i][k];
                }
                f = U[i][l];
                g = -SIGN(sqrt(s), f);
                h = f * g - s;
                U[i][l] = f - g;
                for (k = l; k < COL; k++)
                    rv1[k] = U[i][k] / h;
                for (j = l; j < ROW; j++)
                {
                    for (s = 0.0, k = l; k < COL; k++)
                        s += U[j][k] * U[i][k];
                    for (k = l; k < COL; k++)
                        U[j][k] += s * rv1[k];
                }
                for (k = l; k < COL; k++)
                    U[i][k] *= scale;
            }
        }
        anorm = MAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
    }

    for (i = COL - 1; i >= 0; i--)
    { // Accumulation of right-hand transformations.
        if (i < COL - 1)
        {
            if (g)
            {
                for (j = l; j < COL; j++) // FLOAT division to avoid possible underflow.
                    V[j][i] = (U[i][j] / U[i][l]) / g;
                for (j = l; j < COL; j++)
                {
                    for (s = 0.0, k = l; k < COL; k++)
                        s += U[i][k] * V[k][j];
                    for (k = l; k < COL; k++)
                        V[k][j] += s * V[k][i];
                }
            }
            for (j = l; j < COL; j++)
                V[i][j] = V[j][i] = 0.0;
        }
        V[i][i] = 1.0;
        g = rv1[i];
        l = i;
    }
    for (i = MIN(ROW, COL) - 1; i >= 0; i--)
    { // Accumulation of left-hand transformations.
        l = i + 1;
        g = w[i];
        for (j = l; j < COL; j++)
            U[i][j] = 0.0;
        if (g)
        {
            g = 1.0 / g;
            for (j = l; j < COL; j++)
            {
                for (s = 0.0, k = l; k < ROW; k++)
                    s += U[k][i] * U[k][j];
                f = (s / U[i][i]) * g;
                for (k = i; k < ROW; k++)
                    U[k][j] += f * U[k][i];
            }
            for (j = i; j < ROW; j++)
                U[j][i] *= g;
        }
        else
            for (j = i; j < ROW; j++)
                U[j][i] = 0.0;
        ++U[i][i];
    }

    for (k = COL - 1; k >= 0; k--)
    { // Diagonalization of the bidiagonal form: Loop over singular values,
        for (its = 0; its < 30; its++)
        { // and over allowed iterations.
            flag = 1;
            for (l = k; l >= 0; l--)
            { // Test for splitting.
                nm = l - 1;
                if ((FLOAT)(fabs(rv1[l]) + anorm) == anorm)
                {
                    flag = 0;
                    break;
                }
                if ((FLOAT)(fabs(w[nm]) + anorm) == anorm)
                {
                    break;
                }
            }
            if (flag)
            {
                c = 0.0; // Cancellation of rv1[l], if l > 1.
                s = 1.0;
                for (i = l; i <= k; i++)
                {
                    f = s * rv1[i];
                    rv1[i] = c * rv1[i];
                    if ((FLOAT)(fabs(f) + anorm) == anorm)
                        break;
                    g = w[i];
                    h = pythag(f, g);
                    w[i] = h;
                    h = 1.0 / h;
                    c = g * h;
                    s = -f * h;
                    for (j = 0; j < ROW; j++)
                    {
                        y = U[j][nm];
                        z = U[j][i];
                        U[j][nm] = y * c + z * s;
                        U[j][i] = z * c - y * s;
                    }
                }
            }
            z = w[k];
            if (l == k)
            { // Convergence.
                if (z < 0.0)
                { // Singular value is made nonnegative.
                    w[k] = -z;
                    for (j = 0; j < COL; j++)
                        V[j][k] = -V[j][k];
                }
                break;
            }

            x = w[l]; // Shift from bottom 2-by-2 minor.
            nm = k - 1;
            y = w[nm];
            g = rv1[nm];
            h = rv1[k];
            f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
            g = pythag(f, 1.0);
            f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;
            c = s = 1.0; // Next QR transformation:
            for (j = l; j <= nm; j++)
            {
                i = j + 1;
                g = rv1[i];
                y = w[i];
                h = s * g;
                g = c * g;
                z = pythag(f, h);
                rv1[j] = z;
                c = f / z;
                s = h / z;
                f = x * c + g * s;
                g = g * c - x * s;
                h = y * s;
                y *= c;
                for (jj = 0; jj < COL; jj++)
                {
                    x = V[jj][j];
                    z = V[jj][i];
                    V[jj][j] = x * c + z * s;
                    V[jj][i] = z * c - x * s;
                }
                z = pythag(f, h);
                w[j] = z; // Rotation can be arbitrary if z = 0.
                if (z)
                {
                    z = 1.0 / z;
                    c = f * z;
                    s = h * z;
                }
                f = c * g + s * y;
                x = c * y - s * g;
                for (jj = 0; jj < ROW; jj++)
                {
                    y = U[jj][j];
                    z = U[jj][i];
                    U[jj][j] = y * c + z * s;
                    U[jj][i] = z * c - y * s;
                }
            }
            rv1[l] = 0.0;
            rv1[k] = f;
            w[k] = x;
        }
    }

    // sort singular values and corresponding columns of u and v
    // by decreasing magnitude. Also, signs of corresponding columns are
    // flipped so as to maximize the number of positive elements.
    int s2, inc = 1;
    FLOAT sw;
    FLOAT su[ROW];
    FLOAT sv[COL];
    do
    {
        inc *= 3;
        inc++;
    } while (inc <= COL);
    do
    {
        inc /= 3;
        for (i = inc; i < COL; i++)
        {
            sw = w[i];
            for (k = 0; k < ROW; k++)
                su[k] = U[k][i];
            for (k = 0; k < COL; k++)
                sv[k] = V[k][i];
            j = i;
            while (w[j - inc] < sw)
            {
                w[j] = w[j - inc];
                for (k = 0; k < ROW; k++)
                    U[k][j] = U[k][j - inc];
                for (k = 0; k < COL; k++)
                    V[k][j] = V[k][j - inc];
                j -= inc;
                if (j < inc)
                    break;
            }
            w[j] = sw;
            for (k = 0; k < ROW; k++)
                U[k][j] = su[k];
            for (k = 0; k < COL; k++)
                V[k][j] = sv[k];
        }
    } while (inc > 1);
    for (k = 0; k < COL; k++)
    { // flip signs
        s2 = 0;
        for (i = 0; i < ROW; i++)
            if (U[i][k] < 0.0)
                s2++;
        for (j = 0; j < COL; j++)
            if (V[j][k] < 0.0)
                s2++;
        if (s2 > (ROW+COL) / 2)
        {
            for (i = 0; i < ROW; i++)
                U[i][k] = -U[i][k];
            for (j = 0; j < COL; j++)
                V[j][k] = -V[j][k];
        }
    }

    // create vector and copy singular values
    for (int i = 0; i < MIN(ROW, COL); i++)
        W[i] = w[i];

    // extract mxm submatrix U
    for (int i = 0; i < ROW; i++)
        for (int j = 0; j < MIN(ROW,COL); j++)
            U2[i][j] = U[i][j];

    memset(w, 0, COL);
    memset(rv1, 0, COL);
    memset(su, 0, ROW);
    memset(sv, 0, COL);
}

template <int M, int N>
void solve(FLOAT A[M][N], FLOAT x[N], FLOAT b[M])
{
    FLOAT U[N][N];
    FLOAT W[MIN(M,N)];
    FLOAT V[M][M];
    FLOAT At[N][M];
    trans<M,N>(A, At);
    svd<N, M>(At, U, W, V);

    for (int i = 0; i < N; i++)
    {
        FLOAT wi = 1 / W[i];
        FLOAT s = 0;

        for (int j = 0; j < M; j++)
        {
            s += V[j][i] * b[j];
        }
        s *= wi;

        for (int j = 0; j < N; j++)
        {
            x[j] = (FLOAT)(x[j] + s * U[j][i]);
        }
    }
}
#endif
