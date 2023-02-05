#include "Matrix.h"

#define SIGN(a, b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define SWAP(a, b) \
    {              \
        temp = a;  \
        a = b;     \
        b = temp;  \
    }
static FLOAT sqrarg;
#define SQR(a) ((sqrarg = (a)) == 0.0 ? 0.0 : sqrarg * sqrarg)
#define MAX(a, b) (a > b ? a : b)
#define MIN(a, b) (a < b ? a : b)

Matrix::Matrix()
{
    m = 0;
    n = 0;
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

/*
Matrix::Matrix(const Matrix &M)
{
    allocateMemory(M.m, M.n);
    for (int32_t i = 0; i < M.m; i++)
        memcpy(val[i], M.val[i], M.n * sizeof(FLOAT));
}*/

Matrix::~Matrix()
{
    // releaseMemory();
}

void Matrix::allocateMemory(const int32_t r_, const int32_t c_)
{
    m = abs(r_);
    n = abs(c_);
    if (m == 0 || n == 0)
    {
        val = 0;
        return;
    }
    val = (FLOAT **)malloc(m * sizeof(FLOAT *));
    val[0] = (FLOAT *)calloc(m * n, sizeof(FLOAT));
    for (int32_t i = 1; i < m; i++)
        val[i] = val[i - 1] + n;
}

void Matrix::releaseMemory()
{
    if (val != 0)
    {
        free(val[0]);
        free(val);
    }
}

Matrix Matrix::getMat(int32_t i1, int32_t j1, int32_t i2, int32_t j2)
{
    if (i2 == -1)
        i2 = m - 1;
    if (j2 == -1)
        j2 = n - 1;
    if (i1 < 0 || i2 >= m || j1 < 0 || j2 >= n || i2 < i1 || j2 < j1)
    {
        cerr << "ERROR: Cannot get submatrix [" << i1 << ".." << i2 << "] x [" << j1 << ".." << j2 << "]"
             << " of a (" << m << "x" << n << ") matrix." << endl;
        exit(0);
    }
    Matrix M(i2 - i1 + 1, j2 - j1 + 1);
    for (int32_t i = 0; i < M.m; i++)
        for (int32_t j = 0; j < M.n; j++)
            M.val[i][j] = val[i1 + i][j1 + j];
    return M;
}

void Matrix::setMat(const Matrix &M, const int32_t i1, const int32_t j1)
{
    if (i1 < 0 || j1 < 0 || i1 + M.m > m || j1 + M.n > n)
    {
        cerr << "ERROR: Cannot set submatrix [" << i1 << ".." << i1 + M.m - 1 << "] x [" << j1 << ".." << j1 + M.n - 1 << "]"
             << " of a (" << m << "x" << n << ") matrix." << endl;
        exit(0);
    }
    for (int32_t i = 0; i < M.m; i++)
        for (int32_t j = 0; j < M.n; j++)
            val[i1 + i][j1 + j] = M.val[i][j];
}

Matrix &Matrix::operator=(const Matrix &M)
{
    if (this != &M)
    {
        if (M.m != m || M.n != n)
        {
            releaseMemory();
            allocateMemory(M.m, M.n);
        }
        if (M.n > 0)
            for (int32_t i = 0; i < M.m; i++)
                memcpy(val[i], M.val[i], M.n * sizeof(FLOAT));
    }
    return *this;
}

Matrix Matrix::operator+(const Matrix &M)
{
    const Matrix &A = *this;
    const Matrix &B = M;
    if (A.m != B.m || A.n != B.n)
    {
        cerr << "ERROR: Trying to add matrices of size (" << A.m << "x" << A.n << ") and (" << B.m << "x" << B.n << ")" << endl;
        exit(0);
    }
    Matrix C(A.m, A.n);
    for (int32_t i = 0; i < m; i++)
        for (int32_t j = 0; j < n; j++)
            C.val[i][j] = A.val[i][j] + B.val[i][j];
    return C;
}

Matrix Matrix::operator-(const Matrix &M)
{
    const Matrix &A = *this;
    const Matrix &B = M;
    if (A.m != B.m || A.n != B.n)
    {
        cerr << "ERROR: Trying to subtract matrices of size (" << A.m << "x" << A.n << ") and (" << B.m << "x" << B.n << ")" << endl;
        exit(0);
    }
    Matrix C(A.m, A.n);
    for (int32_t i = 0; i < m; i++)
        for (int32_t j = 0; j < n; j++)
            C.val[i][j] = A.val[i][j] - B.val[i][j];
    return C;
}

Matrix Matrix::operator*(const Matrix &M)
{
    const Matrix &A = *this;
    const Matrix &B = M;
    if (A.n != B.m)
    {
        cerr << "ERROR: Trying to multiply matrices of size (" << A.m << "x" << A.n << ") and (" << B.m << "x" << B.n << ")" << endl;
        exit(0);
    }
    Matrix C(A.m, B.n);
    for (int32_t i = 0; i < A.m; i++)
        for (int32_t j = 0; j < B.n; j++)
            for (int32_t k = 0; k < A.n; k++)
                C.val[i][j] += A.val[i][k] * B.val[k][j];
    return C;
}

Matrix Matrix::operator*(const FLOAT &s)
{
    Matrix C(m, n);
    for (int32_t i = 0; i < m; i++)
        for (int32_t j = 0; j < n; j++)
            C.val[i][j] = val[i][j] * s;
    return C;
}

FLOAT Matrix::dot(const FLOAT *v1, const FLOAT *v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

Matrix Matrix::eye(const int32_t m)
{
    Matrix M(m, m);
    for (int32_t i = 0; i < m; i++)
        M.val[i][i] = 1;
    return M;
}

Matrix Matrix::inv()
{
    if (m != n)
    {
        cerr << "ERROR: Trying to invert matrix of size (" << m << "x" << n << ")" << endl;
        exit(0);
    }
    // Matrix A(M);
    Matrix B = eye(m);
    B.solve(*this);
    return B;
}

Matrix Matrix::trans()
{
    Matrix C(n, m);
    for (int32_t i = 0; i < m; i++)
        for (int32_t j = 0; j < n; j++)
            C.val[j][i] = val[i][j];
    return C;
}

Matrix Matrix::multrans()
{
    Matrix C(n, n);
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

bool Matrix::solve(const Matrix &M, FLOAT eps)
{

    // substitutes
    const Matrix &A = M;
    Matrix &B = *this;

    if (A.m != A.n || A.m != B.m || A.m < 1 || B.n < 1)
    {
        cerr << "ERROR: Trying to eliminate matrices of size (" << A.m << "x" << A.n << ") and (" << B.m << "x" << B.n << ")" << endl;
        exit(0);
    }

    // index vectors for bookkeeping on the pivoting
    int32_t *indxc = new int32_t[m];
    int32_t *indxr = new int32_t[m];
    int32_t *ipiv = new int32_t[m];

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
                SWAP(A.val[irow][l], A.val[icol][l])
            for (l = 0; l < n; l++)
                SWAP(B.val[irow][l], B.val[icol][l])
        }

        indxr[i] = irow; // We are now ready to divide the pivot row by the
        indxc[i] = icol; // pivot element, located at irow and icol.

        // check for singularity
        if (fabs(A.val[icol][icol]) < eps)
        {
            delete[] indxc;
            delete[] indxr;
            delete[] ipiv;
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
                SWAP(A.val[k][indxr[l]], A.val[k][indxc[l]])
    }

    // success
    delete[] indxc;
    delete[] indxr;
    delete[] ipiv;
    return true;
}

void Matrix::solve(Matrix &A, Matrix &x, Matrix &b)
{
    Matrix U, W, V;
    A.trans().svd(U, W, V);

    x = Matrix(A.n, 1);

    for (int i = 0; i < A.n; i++)
    {
        double wi = 1 / W.val[i][0];
        double s = 0;

        for (int j = 0; j < A.m; j++)
        {
            s += V.val[j][i] * b.val[j][0];
        }
        s *= wi;

        for (int j = 0; j < A.n; j++)
        {
            x.val[j][0] = (double)(x.val[j][0] + s * U.val[j][i]);
        }
    }
}

void Matrix::svd(Matrix &U2, Matrix &W, Matrix &V)
{

    Matrix U = Matrix(*this);
    U2 = Matrix(m, m);
    V = Matrix(n, n);

    FLOAT *w = (FLOAT *)malloc(n * sizeof(FLOAT));
    FLOAT *rv1 = (FLOAT *)malloc(n * sizeof(FLOAT));

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
                for (j = l; j < n; j++) // Double division to avoid possible underflow.
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
            if (its == 29)
                cerr << "ERROR in SVD: No convergence in 30 iterations" << endl;
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
    FLOAT *su = (FLOAT *)malloc(m * sizeof(FLOAT));
    FLOAT *sv = (FLOAT *)malloc(n * sizeof(FLOAT));
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
    W = Matrix(min(m, n), 1, w);

    // extract mxm submatrix U
    U2.setMat(U.getMat(0, 0, m - 1, min(m - 1, n - 1)), 0, 0);

    // release temporary memory
    free(w);
    free(rv1);
    free(su);
    free(sv);
}

FLOAT Matrix::pythag(FLOAT a, FLOAT b)
{
    FLOAT absa, absb;
    absa = fabs(a);
    absb = fabs(b);
    if (absa > absb)
        return absa * sqrt(1.0 + SQR(absb / absa));
    else
        return (absb == 0.0 ? 0.0 : absb * sqrt(1.0 + SQR(absa / absb)));
}

ostream &operator<<(ostream &out, const Matrix &M)
{
    if (M.m == 0 || M.n == 0)
    {
        out << "[empty matrix]";
    }
    else
    {
        char buffer[1024];
        for (int32_t i = 0; i < M.m; i++)
        {
            sprintf(buffer, "%d\t", i);
            out << buffer;
            for (int32_t j = 0; j < M.n; j++)
            {
                sprintf(buffer, "%lf ", M.val[i][j]);
                out << buffer;
            }
            if (i < M.m - 1)
                out << endl;
        }
    }
    return out;
}