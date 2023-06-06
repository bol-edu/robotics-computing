#pragma once
#ifndef __MAT_H__
#define __MAT_H__

#include <string.h>
#include <math.h>

#define MAX_KEYPOINT_NUM 500
#define IMAGE_HEIGTH 376
#define IMAGE_WIDTH 1241

typedef signed int int32_t;
typedef float FLOAT; // FLOAT precision

#define SIGN(a, b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
static FLOAT sqrarg;
#define SQR(a) ((sqrarg = (a)) == 0.0 ? 0.0 : sqrarg * sqrarg)
#define MAX(a, b) (a > b ? a : b)
#define MIN(a, b) (a < b ? a : b)

static int tmpArray[IMAGE_HEIGTH][IMAGE_WIDTH];

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

template <typename T, unsigned int ROW, unsigned int COL>
class Matrix
{
public:
    Matrix();

    void setRow(int32_t row);
    Matrix &operator=(const Matrix &M);
    Matrix operator-(const Matrix &M); // subtract matrix
    Matrix operator+(const Matrix &M); // add matrix

    template <unsigned int COL2>
    Matrix<T, ROW, COL2> operator*(const Matrix<T, COL, COL2> &M) const; // multiply with matrix
    Matrix operator*(const FLOAT &s);

    FLOAT norm();
    Matrix inv();
    Matrix<T, COL, ROW> trans();
    Matrix<T, COL, COL> multrans();
    bool solve(const Matrix &M, FLOAT eps = 1e-20);
    void svd(Matrix<T, ROW, ROW> &U2, Matrix<T, MIN(ROW, COL), 1> &W, Matrix<T, COL, COL> &V);

    T val[ROW][COL];
    unsigned int m, n;
};

// function defination

template <typename T, unsigned int ROW, unsigned int COL>
template <unsigned int COL2>
Matrix<T, ROW, COL2> Matrix<T, ROW, COL>::operator*(const Matrix<T, COL, COL2> &M) const
{
    Matrix<T, ROW, COL2> C = Matrix<T, ROW, COL2>();
    for (int32_t i = 0; i < m; i++)
        for (int32_t j = 0; j < M.n; j++)
            for (int32_t k = 0; k < n; k++)
                C.val[i][j] += val[i][k] * M.val[k][j];
    return C;
}

template <typename T, unsigned int ROW, unsigned int COL>
Matrix<T, ROW, COL>::Matrix()
{
    m = ROW;
    n = COL;
    memcpy(val, tmpArray, m * n * sizeof(T));
}

template <typename T, unsigned int ROW, unsigned int COL>
void Matrix<T, ROW, COL>::setRow(int32_t row)
{
    if (row <= ROW)
        m = row;
}

template <typename T, unsigned int ROW, unsigned int COL>
FLOAT Matrix<T, ROW, COL>::norm()
{
    FLOAT b = 0.0;
    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++)
            b += val[i][j] * val[i][j];
    return sqrt(b);
}

template <typename T, unsigned int ROW, unsigned int COL>
Matrix<T, ROW, COL> &Matrix<T, ROW, COL>::operator=(const Matrix<T, ROW, COL> &M)
{
    for (int32_t i = 0; i < M.m; i++)
        memcpy(val[i], M.val[i], M.n * sizeof(T));
    return *this;
}

template <typename T, unsigned int ROW, unsigned int COL>
Matrix<T, ROW, COL> Matrix<T, ROW, COL>::operator+(const Matrix<T, ROW, COL> &M)
{
    Matrix<T, ROW, COL> C = Matrix<T, ROW, COL>();
    for (int32_t i = 0; i < m; i++)
        for (int32_t j = 0; j < n; j++)
            C.val[i][j] = val[i][j] + M.val[i][j];
    return C;
}

template <typename T, unsigned int ROW, unsigned int COL>
Matrix<T, ROW, COL> Matrix<T, ROW, COL>::operator-(const Matrix<T, ROW, COL> &M)
{
    Matrix<T, ROW, COL> C = Matrix<T, ROW, COL>();
    for (int32_t i = 0; i < m; i++)
        for (int32_t j = 0; j < n; j++)
            C.val[i][j] = val[i][j] - M.val[i][j];
    return C;
}

template <typename T, unsigned int ROW, unsigned int COL>
Matrix<T, ROW, COL> Matrix<T, ROW, COL>::operator*(const FLOAT &s)
{
    Matrix<T, ROW, COL> C = Matrix<T, ROW, COL>();
    for (int32_t i = 0; i < m; i++)
        for (int32_t j = 0; j < n; j++)
            C.val[i][j] = val[i][j] * s;
    return C;
}

template <typename T, unsigned int ROW>
Matrix<T, ROW, ROW> eye()
{
    Matrix<T, ROW, ROW> M = Matrix<T, ROW, ROW>();
    for (int32_t i = 0; i < ROW; i++)
        M.val[i][i] = 1;
    return M;
}

template <typename T, unsigned int ROW, unsigned int COL>
Matrix<T, COL, ROW> Matrix<T, ROW, COL>::trans()
{
    Matrix<T, COL, ROW> C = Matrix<T, COL, ROW>();
    for (int32_t i = 0; i < m; i++)
        for (int32_t j = 0; j < n; j++)
            C.val[j][i] = val[i][j];
    return C;
}

template <typename T, unsigned int ROW, unsigned int COL>
Matrix<T, ROW, COL> Matrix<T, ROW, COL>::inv()
{
    Matrix<T, ROW, ROW> B = eye<T, ROW>();
    B.solve(*this);
    return B;
}

template <typename T, unsigned int ROW, unsigned int COL>
Matrix<T, COL, COL> Matrix<T, ROW, COL>::multrans()
{
    Matrix<T, COL, COL> C = Matrix<T, COL, COL>();
    for (int32_t i = 0; i < n; i++)
    {
        for (int32_t j = i; j < n; j++)
        {
            for (int32_t k = 0; k < m; k++)
            {
                C.val[i][j] += val[k][i] * val[k][j];
                if (i != j)
                    C.val[j][i] += val[k][i] * val[k][j];
            }
        }
    }
    return C;
}

template <typename T, unsigned int ROW, unsigned int COL>
bool Matrix<T, ROW, COL>::solve(const Matrix<T, ROW, COL> &M, FLOAT eps)
{

    // substitutes
    // const Matrix<T, ROW, COL> &A = M;
    Matrix<T, ROW, COL> A = M;
    Matrix<T, ROW, COL> &B = *this;

    // index vectors for bookkeeping on the pivoting
    int32_t indxc[ROW]; //*indxc = new int32_t[m];
    int32_t indxr[ROW]; //*indxr = new int32_t[m];
    int32_t ipiv[ROW];  //*ipiv = new int32_t[m];

    // loop variables
    int32_t i, icol, irow, j, k, l, ll;
    FLOAT big, dum, pivinv, temp;

    // initialize pivots to zero
    for (j = 0; j < m; j++)
        ipiv[j] = 0;

    // main loop over the columns to be reduced
    for (i = 0; i < m; i++)
    {

        big = 0.0;

        // search for a pivot element
        for (j = 0; j < m; j++)
            if (ipiv[j] != 1)
                for (k = 0; k < m; k++)
                    if (ipiv[k] == 0)
                        if (fabs(A.val[j][k]) >= big)
                        {
                            big = fabs(A.val[j][k]);
                            irow = j;
                            icol = k;
                        }
        ++(ipiv[icol]);

        // We now have the pivot element, so we interchange rows, if needed, to put the pivot
        // element on the diagonal. The columns are not physically interchanged, only relabeled.
        if (irow != icol)
        {
            for (l = 0; l < m; l++)
                SWAP(A.val[irow][l], A.val[icol][l]);
            for (l = 0; l < n; l++)
                SWAP(B.val[irow][l], B.val[icol][l]);
        }

        indxr[i] = irow; // We are now ready to divide the pivot row by the
        indxc[i] = icol; // pivot element, located at irow and icol.

        // check for singularity
        if (fabs(A.val[icol][icol]) < eps)
        {
            memset(indxc, 0, ROW);
            memset(indxr, 0, ROW);
            memset(ipiv, 0, ROW);
            // delete[] indxc;
            // delete[] indxr;
            // delete[] ipiv;
            return false;
        }

        pivinv = 1.0 / A.val[icol][icol];
        A.val[icol][icol] = 1.0;
        for (l = 0; l < m; l++)
            A.val[icol][l] *= pivinv;
        for (l = 0; l < n; l++)
            B.val[icol][l] *= pivinv;

        // Next, we reduce the rows except for the pivot one
        for (ll = 0; ll < m; ll++)
            if (ll != icol)
            {
                dum = A.val[ll][icol];
                A.val[ll][icol] = 0.0;
                for (l = 0; l < m; l++)
                    A.val[ll][l] -= A.val[icol][l] * dum;
                for (l = 0; l < n; l++)
                    B.val[ll][l] -= B.val[icol][l] * dum;
            }
    }

    // This is the end of the main loop over columns of the reduction. It only remains to unscramble
    // the solution in view of the column interchanges. We do this by interchanging pairs of
    // columns in the reverse order that the permutation was built up.
    for (l = m - 1; l >= 0; l--)
    {
        if (indxr[l] != indxc[l])
            for (k = 0; k < m; k++)
                SWAP(A.val[k][indxr[l]], A.val[k][indxc[l]]);
    }

    // success
    /*delete[] indxc;
    delete[] indxr;
    delete[] ipiv;*/
    return true;
}

template <typename T, unsigned int M, unsigned int N>
void solve(Matrix<T, M, N> &A, Matrix<T, N, 1> &x, Matrix<T, M, 1> &b)
{
    Matrix<T, N, N> U;
    Matrix<T, MIN(N, M), 1> W;
    Matrix<T, M, M> V;
    // Matrix U, W, V;
    A.trans().svd(U, W, V);

    x = Matrix<T, N, 1>(); // Matrix(A.n, 1);

    for (int i = 0; i < A.n; i++)
    {
        FLOAT wi = 1 / W.val[i][0];
        FLOAT s = 0;

        for (int j = 0; j < A.m; j++)
        {
            s += V.val[j][i] * b.val[j][0];
        }
        s *= wi;

        for (int j = 0; j < A.n; j++)
        {
            x.val[j][0] = (FLOAT)(x.val[j][0] + s * U.val[j][i]);
        }
    }
}

template <typename T, unsigned int ROW, unsigned int COL>
void Matrix<T, ROW, COL>::svd(Matrix<T, ROW, ROW> &U2, Matrix<T, MIN(ROW, COL), 1> &W, Matrix<T, COL, COL> &V)
{
    Matrix<T, ROW, COL> U = *this; // Matrix(*this);
    U2 = Matrix<T, ROW, ROW>();
    V = Matrix<T, COL, COL>();

    FLOAT w[COL];
    FLOAT rv1[COL];

    int32_t flag, i, its, j, jj, k, l, nm;
    FLOAT anorm, c, f, g, h, s, scale, x, y, z;

    g = scale = anorm = 0.0; // Householder reduction to bidiagonal form.
    for (i = 0; i < n; i++)
    {
        l = i + 1;
        rv1[i] = scale * g;

        g = s = scale = 0.0;
        if (i < m)
        {
            for (k = i; k < m; k++)
            {
                scale += fabs(U.val[k][i]);
            }
            if (scale)
            {
                for (k = i; k < m; k++)
                {
                    U.val[k][i] /= scale;
                    s += U.val[k][i] * U.val[k][i];
                }
                f = U.val[i][i];
                g = -SIGN(sqrt(s), f);
                h = f * g - s;
                U.val[i][i] = f - g;
                for (j = l; j < n; j++)
                {
                    for (s = 0.0, k = i; k < m; k++)
                        s += U.val[k][i] * U.val[k][j];
                    f = s / h;
                    for (k = i; k < m; k++)
                        U.val[k][j] += f * U.val[k][i];
                }
                for (k = i; k < m; k++)
                    U.val[k][i] *= scale;
            }
        }
        w[i] = scale * g;
        g = s = scale = 0.0;
        if (i < m && i != n - 1)
        {
            for (k = l; k < n; k++)
                scale += fabs(U.val[i][k]);
            if (scale)
            {
                for (k = l; k < n; k++)
                {
                    U.val[i][k] /= scale;
                    s += U.val[i][k] * U.val[i][k];
                }
                f = U.val[i][l];
                g = -SIGN(sqrt(s), f);
                h = f * g - s;
                U.val[i][l] = f - g;
                for (k = l; k < n; k++)
                    rv1[k] = U.val[i][k] / h;
                for (j = l; j < m; j++)
                {
                    for (s = 0.0, k = l; k < n; k++)
                        s += U.val[j][k] * U.val[i][k];
                    for (k = l; k < n; k++)
                        U.val[j][k] += s * rv1[k];
                }
                for (k = l; k < n; k++)
                    U.val[i][k] *= scale;
            }
        }
        anorm = MAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
    }

    for (i = n - 1; i >= 0; i--)
    { // Accumulation of right-hand transformations.
        if (i < n - 1)
        {
            if (g)
            {
                for (j = l; j < n; j++) // FLOAT division to avoid possible underflow.
                    V.val[j][i] = (U.val[i][j] / U.val[i][l]) / g;
                for (j = l; j < n; j++)
                {
                    for (s = 0.0, k = l; k < n; k++)
                        s += U.val[i][k] * V.val[k][j];
                    for (k = l; k < n; k++)
                        V.val[k][j] += s * V.val[k][i];
                }
            }
            for (j = l; j < n; j++)
                V.val[i][j] = V.val[j][i] = 0.0;
        }
        V.val[i][i] = 1.0;
        g = rv1[i];
        l = i;
    }
    for (i = MIN(m, n) - 1; i >= 0; i--)
    { // Accumulation of left-hand transformations.
        l = i + 1;
        g = w[i];
        for (j = l; j < n; j++)
            U.val[i][j] = 0.0;
        if (g)
        {
            g = 1.0 / g;
            for (j = l; j < n; j++)
            {
                for (s = 0.0, k = l; k < m; k++)
                    s += U.val[k][i] * U.val[k][j];
                f = (s / U.val[i][i]) * g;
                for (k = i; k < m; k++)
                    U.val[k][j] += f * U.val[k][i];
            }
            for (j = i; j < m; j++)
                U.val[j][i] *= g;
        }
        else
            for (j = i; j < m; j++)
                U.val[j][i] = 0.0;
        ++U.val[i][i];
    }

    for (k = n - 1; k >= 0; k--)
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
                    for (j = 0; j < m; j++)
                    {
                        y = U.val[j][nm];
                        z = U.val[j][i];
                        U.val[j][nm] = y * c + z * s;
                        U.val[j][i] = z * c - y * s;
                    }
                }
            }
            z = w[k];
            if (l == k)
            { // Convergence.
                if (z < 0.0)
                { // Singular value is made nonnegative.
                    w[k] = -z;
                    for (j = 0; j < n; j++)
                        V.val[j][k] = -V.val[j][k];
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
                for (jj = 0; jj < n; jj++)
                {
                    x = V.val[jj][j];
                    z = V.val[jj][i];
                    V.val[jj][j] = x * c + z * s;
                    V.val[jj][i] = z * c - x * s;
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
                for (jj = 0; jj < m; jj++)
                {
                    y = U.val[jj][j];
                    z = U.val[jj][i];
                    U.val[jj][j] = y * c + z * s;
                    U.val[jj][i] = z * c - y * s;
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
    int32_t s2, inc = 1;
    FLOAT sw;
    FLOAT su[ROW];
    FLOAT sv[COL];
    do
    {
        inc *= 3;
        inc++;
    } while (inc <= n);
    do
    {
        inc /= 3;
        for (i = inc; i < n; i++)
        {
            sw = w[i];
            for (k = 0; k < m; k++)
                su[k] = U.val[k][i];
            for (k = 0; k < n; k++)
                sv[k] = V.val[k][i];
            j = i;
            while (w[j - inc] < sw)
            {
                w[j] = w[j - inc];
                for (k = 0; k < m; k++)
                    U.val[k][j] = U.val[k][j - inc];
                for (k = 0; k < n; k++)
                    V.val[k][j] = V.val[k][j - inc];
                j -= inc;
                if (j < inc)
                    break;
            }
            w[j] = sw;
            for (k = 0; k < m; k++)
                U.val[k][j] = su[k];
            for (k = 0; k < n; k++)
                V.val[k][j] = sv[k];
        }
    } while (inc > 1);
    for (k = 0; k < n; k++)
    { // flip signs
        s2 = 0;
        for (i = 0; i < m; i++)
            if (U.val[i][k] < 0.0)
                s2++;
        for (j = 0; j < n; j++)
            if (V.val[j][k] < 0.0)
                s2++;
        if (s2 > (m + n) / 2)
        {
            for (i = 0; i < m; i++)
                U.val[i][k] = -U.val[i][k];
            for (j = 0; j < n; j++)
                V.val[j][k] = -V.val[j][k];
        }
    }

    // create vector and copy singular values
    W = Matrix<T, MIN(ROW, COL), 1>();
    for (int32_t i = 0; i < W.m; i++)
        W.val[i][0] = w[i];

    // extract mxm submatrix U
    for (int32_t i = 0; i < m; i++)
        for (int32_t j = 0; j < MIN(m, n); j++)
            U2.val[i][j] = U.val[i][j];

    memset(w, 0, COL);
    memset(rv1, 0, COL);
    memset(su, 0, ROW);
    memset(sv, 0, COL);
}

#endif
