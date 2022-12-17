#pragma once

using namespace std;

#define det2(m)   ((double)m(0,0)*m(1,1) - (double)m(0,1)*m(1,0))
#define det3(m)   (m(0,0)*((double)m(1,1)*m(2,2) - (double)m(1,2)*m(2,1)) -  \
                   m(0,1)*((double)m(1,0)*m(2,2) - (double)m(1,2)*m(2,0)) +  \
                   m(0,2)*((double)m(1,0)*m(2,1) - (double)m(1,1)*m(2,0)))

#define Sf( y, x ) ((float*)(srcdata + y*srcstep))[x]
#define Sd( y, x ) ((double*)(srcdata + y*srcstep))[x]
#define Df( y, x ) ((float*)(dstdata + y*dststep))[x]
#define Dd( y, x ) ((double*)(dstdata + y*dststep))[x]

template<typename _Tp> static inline int
LUImpl(_Tp* A, size_t astep, int m, _Tp* b, size_t bstep, int n, _Tp eps)
{
    int i, j, k, p = 1;
    astep /= sizeof(A[0]);
    bstep /= sizeof(b[0]);

    for (i = 0; i < m; i++)
    {
        k = i;

        for (j = i + 1; j < m; j++)
            if (std::abs(A[j * astep + i]) > std::abs(A[k * astep + i]))
                k = j;

        if (std::abs(A[k * astep + i]) < eps)
            return 0;

        if (k != i)
        {
            for (j = i; j < m; j++)
                std::swap(A[i * astep + j], A[k * astep + j]);
            if (b)
                for (j = 0; j < n; j++)
                    std::swap(b[i * bstep + j], b[k * bstep + j]);
            p = -p;
        }

        _Tp d = -1 / A[i * astep + i];

        for (j = i + 1; j < m; j++)
        {
            _Tp alpha = A[j * astep + i] * d;

            for (k = i + 1; k < m; k++)
                A[j * astep + k] += alpha * A[i * astep + k];

            if (b)
                for (k = 0; k < n; k++)
                    b[j * bstep + k] += alpha * b[i * bstep + k];
        }
    }

    if (b)
    {
        for (i = m - 1; i >= 0; i--)
            for (j = 0; j < n; j++)
            {
                _Tp s = b[i * bstep + j];
                for (k = i + 1; k < m; k++)
                    s -= A[i * astep + k] * b[k * bstep + j];
                b[i * bstep + j] = s / A[i * astep + i];
            }
    }

    return p;
}

int LU32f(float* A, size_t astep, int m, float* b, size_t bstep, int n)
{
    //CV_INSTRUMENT_REGION();

    int output;
   // CALL_HAL_RET(LU32f, cv_hal_LU32f, output, A, astep, m, b, bstep, n)
        output = LUImpl(A, astep, m, b, bstep, n, FLT_EPSILON * 10);
    return output;
}


int LU64f(double* A, size_t astep, int m, double* b, size_t bstep, int n)
{
    //CV_INSTRUMENT_REGION();

    int output;
    //CALL_HAL_RET(LU64f, cv_hal_LU64f, output, A, astep, m, b, bstep, n)
        output = LUImpl(A, astep, m, b, bstep, n, DBL_EPSILON * 100);
    return output;
}

bool solve_(InputArray _src, InputArray _src2arg, OutputArray _dst, int method)
{
   // CV_INSTRUMENT_REGION();

    bool result = true;
    Mat src = _src.getMat(), _src2 = _src2arg.getMat();
    int type = src.type();
    bool is_normal = (method & DECOMP_NORMAL) != 0;

    CV_Assert(type == _src2.type() && (type == CV_32F || type == CV_64F));

    method &= ~DECOMP_NORMAL;
    CV_Check(method, method == DECOMP_LU || method == DECOMP_SVD || method == DECOMP_EIG ||
        method == DECOMP_CHOLESKY || method == DECOMP_QR,
        "Unsupported method, see #DecompTypes");
    CV_Assert((method != DECOMP_LU && method != DECOMP_CHOLESKY) ||
        is_normal || src.rows == src.cols);

    // check case of a single equation and small matrix
    if ((method == DECOMP_LU || method == DECOMP_CHOLESKY) && !is_normal &&
        src.rows <= 3 && src.rows == src.cols && _src2.cols == 1)
    {
        _dst.create(src.cols, _src2.cols, src.type());
        Mat dst = _dst.getMat();

#define bf(y) ((float*)(bdata + y*src2step))[0]
#define bd(y) ((double*)(bdata + y*src2step))[0]

        const uchar* srcdata = src.ptr();
        const uchar* bdata = _src2.ptr();
        uchar* dstdata = dst.ptr();
        size_t srcstep = src.step;
        size_t src2step = _src2.step;
        size_t dststep = dst.step;

        if (src.rows == 2)
        {
            if (type == CV_32FC1)
            {
                double d = det2(Sf);
                if (d != 0.)
                {
                    double t;
                    d = 1. / d;
                    t = (float)(((double)bf(0) * Sf(1, 1) - (double)bf(1) * Sf(0, 1)) * d);
                    Df(1, 0) = (float)(((double)bf(1) * Sf(0, 0) - (double)bf(0) * Sf(1, 0)) * d);
                    Df(0, 0) = (float)t;
                }
                else
                    result = false;
            }
            else
            {
                double d = det2(Sd);
                if (d != 0.)
                {
                    double t;
                    d = 1. / d;
                    t = (bd(0) * Sd(1, 1) - bd(1) * Sd(0, 1)) * d;
                    Dd(1, 0) = (bd(1) * Sd(0, 0) - bd(0) * Sd(1, 0)) * d;
                    Dd(0, 0) = t;
                }
                else
                    result = false;
            }
        }
        else if (src.rows == 3)
        {
            if (type == CV_32FC1)
            {
                double d = det3(Sf);
                if (d != 0.)
                {
                    float t[3];
                    d = 1. / d;

                    t[0] = (float)(d *
                        (bf(0) * ((double)Sf(1, 1) * Sf(2, 2) - (double)Sf(1, 2) * Sf(2, 1)) -
                            Sf(0, 1) * ((double)bf(1) * Sf(2, 2) - (double)Sf(1, 2) * bf(2)) +
                            Sf(0, 2) * ((double)bf(1) * Sf(2, 1) - (double)Sf(1, 1) * bf(2))));

                    t[1] = (float)(d *
                        (Sf(0, 0) * (double)(bf(1) * Sf(2, 2) - (double)Sf(1, 2) * bf(2)) -
                            bf(0) * ((double)Sf(1, 0) * Sf(2, 2) - (double)Sf(1, 2) * Sf(2, 0)) +
                            Sf(0, 2) * ((double)Sf(1, 0) * bf(2) - (double)bf(1) * Sf(2, 0))));

                    t[2] = (float)(d *
                        (Sf(0, 0) * ((double)Sf(1, 1) * bf(2) - (double)bf(1) * Sf(2, 1)) -
                            Sf(0, 1) * ((double)Sf(1, 0) * bf(2) - (double)bf(1) * Sf(2, 0)) +
                            bf(0) * ((double)Sf(1, 0) * Sf(2, 1) - (double)Sf(1, 1) * Sf(2, 0))));

                    Df(0, 0) = t[0];
                    Df(1, 0) = t[1];
                    Df(2, 0) = t[2];
                }
                else
                    result = false;
            }
            else
            {
                double d = det3(Sd);
                if (d != 0.)
                {
                    double t[9];

                    d = 1. / d;

                    t[0] = ((Sd(1, 1) * Sd(2, 2) - Sd(1, 2) * Sd(2, 1)) * bd(0) +
                        (Sd(0, 2) * Sd(2, 1) - Sd(0, 1) * Sd(2, 2)) * bd(1) +
                        (Sd(0, 1) * Sd(1, 2) - Sd(0, 2) * Sd(1, 1)) * bd(2)) * d;

                    t[1] = ((Sd(1, 2) * Sd(2, 0) - Sd(1, 0) * Sd(2, 2)) * bd(0) +
                        (Sd(0, 0) * Sd(2, 2) - Sd(0, 2) * Sd(2, 0)) * bd(1) +
                        (Sd(0, 2) * Sd(1, 0) - Sd(0, 0) * Sd(1, 2)) * bd(2)) * d;

                    t[2] = ((Sd(1, 0) * Sd(2, 1) - Sd(1, 1) * Sd(2, 0)) * bd(0) +
                        (Sd(0, 1) * Sd(2, 0) - Sd(0, 0) * Sd(2, 1)) * bd(1) +
                        (Sd(0, 0) * Sd(1, 1) - Sd(0, 1) * Sd(1, 0)) * bd(2)) * d;

                    Dd(0, 0) = t[0];
                    Dd(1, 0) = t[1];
                    Dd(2, 0) = t[2];
                }
                else
                    result = false;
            }
        }
        else
        {
            assert(src.rows == 1);

            if (type == CV_32FC1)
            {
                double d = Sf(0, 0);
                if (d != 0.)
                    Df(0, 0) = (float)(bf(0) / d);
                else
                    result = false;
            }
            else
            {
                double d = Sd(0, 0);
                if (d != 0.)
                    Dd(0, 0) = (bd(0) / d);
                else
                    result = false;
            }
        }
        return result;
    }

    int m = src.rows, m_ = m, n = src.cols, nb = _src2.cols;
    size_t esz = CV_ELEM_SIZE(type), bufsize = 0;
    size_t vstep = alignSize(n * esz, 16);
    size_t astep = method == DECOMP_SVD && !is_normal ? alignSize(m * esz, 16) : vstep;
    AutoBuffer<uchar> buffer;

    Mat src2 = _src2;
    _dst.create(src.cols, src2.cols, src.type());
    Mat dst = _dst.getMat();

    if (m < n)
        CV_Error(CV_StsBadArg, "The function can not solve under-determined linear systems");

    if (m == n)
        is_normal = false;
    else if (is_normal)
    {
        m_ = n;
        if (method == DECOMP_SVD)
            method = DECOMP_EIG;
    }

    size_t asize = astep * (method == DECOMP_SVD || is_normal ? n : m);
    bufsize += asize + 32;

    if (is_normal)
        bufsize += n * nb * esz;
    if (method == DECOMP_SVD || method == DECOMP_EIG)
        bufsize += n * 5 * esz + n * vstep + nb * sizeof(double) + 32;

    buffer.allocate(bufsize);
    uchar* ptr = alignPtr(buffer.data(), 16);

    Mat a(m_, n, type, ptr, astep);

    if (is_normal)
        mulTransposed(src, a, true);
    else if (method != DECOMP_SVD)
        src.copyTo(a);
    else
    {
        a = Mat(n, m_, type, ptr, astep);
        transpose(src, a);
    }
    ptr += asize;

    if (!is_normal)
    {
        if (method == DECOMP_LU || method == DECOMP_CHOLESKY)
            src2.copyTo(dst);
    }
    else
    {
        // a'*b
        if (method == DECOMP_LU || method == DECOMP_CHOLESKY)
            gemm(src, src2, 1, Mat(), 0, dst, GEMM_1_T);
        else
        {
            Mat tmp(n, nb, type, ptr);
            ptr += n * nb * esz;
            gemm(src, src2, 1, Mat(), 0, tmp, GEMM_1_T);
            src2 = tmp;
        }
    }

    if (method == DECOMP_LU)
    {
        if (type == CV_32F)
            result = LU32f(a.ptr<float>(), a.step, n, dst.ptr<float>(), dst.step, nb) != 0;
        else
            result = LU64f(a.ptr<double>(), a.step, n, dst.ptr<double>(), dst.step, nb) != 0;
    }
    /*else if (method == DECOMP_CHOLESKY)
    {
        if (type == CV_32F)
            result = hal::Cholesky32f(a.ptr<float>(), a.step, n, dst.ptr<float>(), dst.step, nb);
        else
            result = hal::Cholesky64f(a.ptr<double>(), a.step, n, dst.ptr<double>(), dst.step, nb);
    }
    else if (method == DECOMP_QR)
    {
        Mat rhsMat;
        if (is_normal || m == n)
        {
            src2.copyTo(dst);
            rhsMat = dst;
        }
        else
        {
            rhsMat = Mat(m, nb, type);
            src2.copyTo(rhsMat);
        }

        if (type == CV_32F)
            result = hal::QR32f(a.ptr<float>(), a.step, a.rows, a.cols, rhsMat.cols, rhsMat.ptr<float>(), rhsMat.step, NULL) != 0;
        else
            result = hal::QR64f(a.ptr<double>(), a.step, a.rows, a.cols, rhsMat.cols, rhsMat.ptr<double>(), rhsMat.step, NULL) != 0;

        if (rhsMat.rows != dst.rows)
            rhsMat.rowRange(0, dst.rows).copyTo(dst);
    }
    else
    {
        ptr = alignPtr(ptr, 16);
        Mat v(n, n, type, ptr, vstep), w(n, 1, type, ptr + vstep * n), u;
        ptr += n * (vstep + esz);

        if (method == DECOMP_EIG)
        {
            if (type == CV_32F)
                Jacobi(a.ptr<float>(), a.step, w.ptr<float>(), v.ptr<float>(), v.step, n, ptr);
            else
                Jacobi(a.ptr<double>(), a.step, w.ptr<double>(), v.ptr<double>(), v.step, n, ptr);
            u = v;
        }
        else
        {
            if (type == CV_32F)
                JacobiSVD(a.ptr<float>(), a.step, w.ptr<float>(), v.ptr<float>(), v.step, m_, n);
            else
                JacobiSVD(a.ptr<double>(), a.step, w.ptr<double>(), v.ptr<double>(), v.step, m_, n);
            u = a;
        }

        if (type == CV_32F)
        {
            SVBkSb(m_, n, w.ptr<float>(), 0, u.ptr<float>(), u.step, true,
                v.ptr<float>(), v.step, true, src2.ptr<float>(),
                src2.step, nb, dst.ptr<float>(), dst.step, ptr);
        }
        else
        {
            SVBkSb(m_, n, w.ptr<double>(), 0, u.ptr<double>(), u.step, true,
                v.ptr<double>(), v.step, true, src2.ptr<double>(),
                src2.step, nb, dst.ptr<double>(), dst.step, ptr);
        }
        result = true;
    }*/

    if (!result)
        dst = Scalar(0);

    return result;
}

template<typename _Tp, int cn> class Vec__;

template<typename _Tp, int m, int n> class Matx__
{
public:
    enum {
        rows = m,
        cols = n,
        channels = rows * cols,
#ifdef OPENCV_TRAITS_ENABLE_DEPRECATED
        depth = traits::Type<_Tp>::value,
        type = CV_MAKETYPE(depth, channels),
#endif
        shortdim = (m < n ? m : n)
    };

    typedef _Tp                           value_type;
    typedef Matx__<_Tp, m, n>               mat_type;
    typedef Matx__<_Tp, shortdim, 1> diag_type;

    //! default constructor
    Matx__();

     Matx__(_Tp v0); //!< 1x1 matrix
    Matx__(_Tp v0, _Tp v1); //!< 1x2 or 2x1 matrix
    Matx__(_Tp v0, _Tp v1, _Tp v2); //!< 1x3 or 3x1 matrix
    Matx__(_Tp v0, _Tp v1, _Tp v2, _Tp v3); //!< 1x4, 2x2 or 4x1 matrix
    Matx__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4); //!< 1x5 or 5x1 matrix
    Matx__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5); //!< 1x6, 2x3, 3x2 or 6x1 matrix
    Matx__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5, _Tp v6); //!< 1x7 or 7x1 matrix
    Matx__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5, _Tp v6, _Tp v7); //!< 1x8, 2x4, 4x2 or 8x1 matrix
    Matx__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5, _Tp v6, _Tp v7, _Tp v8); //!< 1x9, 3x3 or 9x1 matrix
    Matx__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5, _Tp v6, _Tp v7, _Tp v8, _Tp v9); //!< 1x10, 2x5 or 5x2 or 10x1 matrix
    Matx__(_Tp v0, _Tp v1, _Tp v2, _Tp v3,
        _Tp v4, _Tp v5, _Tp v6, _Tp v7,
        _Tp v8, _Tp v9, _Tp v10, _Tp v11); //!< 1x12, 2x6, 3x4, 4x3, 6x2 or 12x1 matrix
    Matx__(_Tp v0, _Tp v1, _Tp v2, _Tp v3,
        _Tp v4, _Tp v5, _Tp v6, _Tp v7,
        _Tp v8, _Tp v9, _Tp v10, _Tp v11,
        _Tp v12, _Tp v13); //!< 1x14, 2x7, 7x2 or 14x1 matrix
    Matx__(_Tp v0, _Tp v1, _Tp v2, _Tp v3,
        _Tp v4, _Tp v5, _Tp v6, _Tp v7,
        _Tp v8, _Tp v9, _Tp v10, _Tp v11,
        _Tp v12, _Tp v13, _Tp v14, _Tp v15); //!< 1x16, 4x4 or 16x1 matrix
     Matx__(const _Tp* vals); //!< initialize from a plain array

    Matx__(std::initializer_list<_Tp>); //!< initialize from an initializer list

    static Matx__ all(_Tp alpha);
    static Matx__ zeros();
    static Matx__ ones();
    static Matx__ eye();
    static Matx__ diag(const diag_type& d);
    // @brief Generates uniformly distributed random numbers
   // @param a Range boundary.
    //@param b The other range boundary (boundaries don't have to be ordered, the lower boundary is inclusive,
   // the upper one is exclusive).
     
    static Matx__ randu(_Tp a, _Tp b);
    // @brief Generates normally distributed random numbers
    //@param a Mean value.
    //@param b Standard deviation.
     
    static Matx__ randn(_Tp a, _Tp b);

    //! dot product computed with the default precision
    _Tp dot(const Matx__<_Tp, m, n>& v) const;

    //! dot product computed in double-precision arithmetics
    double ddot(const Matx__<_Tp, m, n>& v) const;

    //! conversion to another data type
    template<typename T2> operator Matx__<T2, m, n>() const;

    //! change the matrix shape
    template<int m1, int n1> Matx__<_Tp, m1, n1> reshape() const;

    //! extract part of the matrix
    template<int m1, int n1> Matx__<_Tp, m1, n1> get_minor(int base_row, int base_col) const;

    //! extract the matrix row
    Matx__<_Tp, 1, n> row(int i) const;

    //! extract the matrix column
    Matx__<_Tp, m, 1> col(int i) const;

    //! extract the matrix diagonal
    diag_type diag() const;

    //! transpose the matrix
    Matx__<_Tp, n, m> t() const;

    //! invert the matrix
    Matx__<_Tp, n, m> inv(int method = DECOMP_LU, bool* p_is_ok = NULL) const;

    //! solve linear system
    template<int l> Matx__<_Tp, n, l> solve(const Matx__<_Tp, m, l>& rhs, int flags = DECOMP_LU) const;
    Vec__<_Tp, n> solve(const Vec__<_Tp, m>& rhs, int method) const;

    //! multiply two matrices element-wise
    Matx__<_Tp, m, n> mul(const Matx__<_Tp, m, n>& a) const;

    //! divide two matrices element-wise
    Matx__<_Tp, m, n> div(const Matx__<_Tp, m, n>& a) const;

    //! element access
    const _Tp& operator ()(int row, int col) const;
    _Tp& operator ()(int row, int col);

    //! 1D element access
    const _Tp& operator ()(int i) const;
    _Tp& operator ()(int i);

    Matx__(const Matx__<_Tp, m, n>& a, const Matx__<_Tp, m, n>& b, Matx_AddOp);
    Matx__(const Matx__<_Tp, m, n>& a, const Matx__<_Tp, m, n>& b, Matx_SubOp);
    template<typename _T2> Matx__(const Matx__<_Tp, m, n>& a, _T2 alpha, Matx_ScaleOp);
    Matx__(const Matx__<_Tp, m, n>& a, const Matx__<_Tp, m, n>& b, Matx_MulOp);
    Matx__(const Matx__<_Tp, m, n>& a, const Matx__<_Tp, m, n>& b, Matx_DivOp);
    template<int l> Matx__(const Matx__<_Tp, m, l>& a, const Matx__<_Tp, l, n>& b, Matx_MatMulOp);
    Matx__(const Matx__<_Tp, n, m>& a, Matx_TOp);

    _Tp val[m * n]; //< matrix elements
};

template<typename _Tp, int m, int n> inline
Matx__<_Tp, m, n>::Matx__()
{
    for (int i = 0; i < channels; i++) val[i] = _Tp(0);
}

template<typename _Tp, int m, int n> inline
Matx__<_Tp, m, n>::Matx__(_Tp v0, _Tp v1, _Tp v2)
{
    CV_StaticAssert(channels >= 3, "Matx__ should have at least 3 elements.");
    val[0] = v0; val[1] = v1; val[2] = v2;
    for (int i = 3; i < channels; i++) val[i] = _Tp(0);
}

template<typename _Tp, int m, int n> inline
Matx__<_Tp, m, n>::Matx__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5, _Tp v6, _Tp v7, _Tp v8)
{
    CV_StaticAssert(channels >= 9, "Matx__ should have at least 9 elements.");
    val[0] = v0; val[1] = v1; val[2] = v2; val[3] = v3;
    val[4] = v4; val[5] = v5; val[6] = v6; val[7] = v7;
    val[8] = v8;
    for (int i = 9; i < channels; i++) val[i] = _Tp(0);
}



template<typename _Tp, int m, int n> inline
Matx__<_Tp, m, n>::Matx__(const _Tp* values)
{
    for (int i = 0; i < channels; i++) val[i] = values[i];
}

template<typename _Tp, int m, int n> inline
Matx__<_Tp, m, n> Matx__<_Tp, m, n>::all(_Tp alpha)
{
    Matx__<_Tp, m, n> M;
    for (int i = 0; i < m * n; i++) M.val[i] = alpha;
    return M;
}

template<typename _Tp, int m, int n> inline
Matx__<_Tp, m, n> Matx__<_Tp, m, n>::zeros()
{
    return all(0);
}

template<typename _Tp, int m, int l, int n> struct Matx_FastSolveOp_
{
    bool operator()(const Matx__<_Tp, m, l>& a, const Matx__<_Tp, m, n>& b,
        Matx__<_Tp, l, n>& x, int method) const
    {
        return cv::solve(a, b, x, method);
    }
};

template<typename _Tp, int m, int n> template<int l> inline
Matx__<_Tp, n, l> Matx__<_Tp, m, n>::solve(const Matx__<_Tp, m, l>& rhs, int method) const
{
    Matx__<_Tp, n, l> x;
    bool ok = Matx_FastSolveOp_<_Tp, m, n, l>()(*this, rhs, x, method);
   // return ok ? x : Matx__<_Tp, n, l>::zeros();

    return x;
}


template<typename _Tp, int m, int n> inline
Vec__<_Tp, n> Matx__<_Tp, m, n>::solve(const Vec__<_Tp, m>& rhs, int method) const
{
    Matx__<_Tp, n, 1> x = solve((const Matx__<_Tp, m, 1>&)(rhs), method);
    return (Vec__<_Tp, n>&)(x);
}

template<typename _Tp, int m, int n> inline
const _Tp& Matx__<_Tp, m, n>::operator()(int row_idx, int col_idx) const
{
    CV_DbgAssert((unsigned)row_idx < (unsigned)m && (unsigned)col_idx < (unsigned)n);
    return this->val[row_idx * n + col_idx];
}

template<typename _Tp, int m, int n> inline
_Tp& Matx__<_Tp, m, n>::operator ()(int row_idx, int col_idx)
{
    CV_DbgAssert((unsigned)row_idx < (unsigned)m && (unsigned)col_idx < (unsigned)n);
    return val[row_idx * n + col_idx];
}

template<typename _Tp, int m, int n> inline
const _Tp& Matx__<_Tp, m, n>::operator ()(int i) const
{
    CV_StaticAssert(m == 1 || n == 1, "Single index indexation requires matrix to be a column or a row");
    CV_DbgAssert((unsigned)i < (unsigned)(m + n - 1));
    return val[i];
}

template<typename _Tp, int m, int n> inline
_Tp& Matx__<_Tp, m, n>::operator ()(int i)
{
    CV_StaticAssert(m == 1 || n == 1, "Single index indexation requires matrix to be a column or a row");
    CV_DbgAssert((unsigned)i < (unsigned)(m + n - 1));
    return val[i];
}



template<typename _Tp, int cn> class Vec__ : public Matx__<_Tp, cn, 1>
{
public:
    typedef _Tp value_type;
    enum {
        channels = cn,
#ifdef OPENCV_TRAITS_ENABLE_DEPRECATED
        depth = Matx<_Tp, cn, 1>::depth,
        type = CV_MAKETYPE(depth, channels),
#endif
        _dummy_enum_finalizer = 0
    };

    //! default constructor
    Vec__();

    Vec__(_Tp v0); //!< 1-element Vec__tor constructor
    Vec__(_Tp v0, _Tp v1); //!< 2-element Vec__tor constructor
    Vec__(_Tp v0, _Tp v1, _Tp v2); //!< 3-element Vec__tor constructor
    Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3); //!< 4-element Vec__tor constructor
    Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4); //!< 5-element Vec__tor constructor
    Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5); //!< 6-element Vec__tor constructor
    Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5, _Tp v6); //!< 7-element Vec__tor constructor
    Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5, _Tp v6, _Tp v7); //!< 8-element Vec__tor constructor
    Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5, _Tp v6, _Tp v7, _Tp v8); //!< 9-element Vec__tor constructor
    Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5, _Tp v6, _Tp v7, _Tp v8, _Tp v9); //!< 10-element Vec__tor constructor
    Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5, _Tp v6, _Tp v7, _Tp v8, _Tp v9, _Tp v10, _Tp v11, _Tp v12, _Tp v13); //!< 14-element Vec__tor constructor
     Vec__(const _Tp* values);

    Vec__(std::initializer_list<_Tp>);

    Vec__(const Vec__<_Tp, cn>& v);

    static Vec__ all(_Tp alpha);

    //! per-element multiplication
    Vec__ mul(const Vec__<_Tp, cn>& v) const;

    //! conjugation (makes sense for complex numbers and quaternions)
    Vec__ conj() const;

    /*!
      cross product of the two 3D Vec__tors.

      For other dimensionalities the exception is raised
    */
    Vec__ cross(const Vec__& v) const;
    //! conversion to another data type
    template<typename T2> operator Vec__<T2, cn>() const;

    /*! element access */
    const _Tp& operator [](int i) const;
    _Tp& operator[](int i);
    const _Tp& operator ()(int i) const;
    _Tp& operator ()(int i);

#ifdef CV_CXX11
    Vec__<_Tp, cn>& operator=(const Vec__<_Tp, cn>& rhs) = default;
#endif

    Vec__(const Matx<_Tp, cn, 1>& a, const Matx<_Tp, cn, 1>& b, Matx_AddOp);
    Vec__(const Matx<_Tp, cn, 1>& a, const Matx<_Tp, cn, 1>& b, Matx_SubOp);
    template<typename _T2> Vec__(const Matx<_Tp, cn, 1>& a, _T2 alpha, Matx_ScaleOp);
};





template<typename _Tp, int cn> inline
Vec__<_Tp, cn>::Vec__(_Tp v0, _Tp v1, _Tp v2)
    : Matx__<_Tp, cn, 1>(v0, v1, v2) {}



template<typename _Tp, int cn> inline
_Tp& Vec__<_Tp, cn>::operator [](int i)
{
    CV_DbgAssert((unsigned)i < (unsigned)cn);
    return this->val[i];
}

template<typename _Tp, int cn> inline
Vec__<_Tp, cn>::Vec__(const Vec__<_Tp, cn>& m)
    : Matx__<_Tp, cn, 1>(m.val) {}





typedef Vec__<float, 3> Vec3f__;
typedef Matx__<float, 3, 3> Matx33f__;