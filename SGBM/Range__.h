#pragma once
#include <opencv2/opencv.hpp>
using namespace std;

class CV_EXPORTS Range__
{
public:
    Range__();
    Range__(int _start, int _end);
    int size() const;
    bool empty() const;
    static Range__ all();

    int start, end;
};

inline
Range__::Range__()
    : start(0), end(0) {}

inline
Range__::Range__(int _start, int _end)
    : start(_start), end(_end) {}

inline
int Range__::size() const
{
    return end - start;
}

inline
bool Range__::empty() const
{
    return start == end;
}

inline
Range__ Range__::all()
{
    return Range__(INT_MIN, INT_MAX);
}


static inline
bool operator == (const Range__& r1, const Range__& r2)
{
    return r1.start == r2.start && r1.end == r2.end;
}

static inline
bool operator != (const Range__& r1, const Range__& r2)
{
    return !(r1 == r2);
}

static inline
bool operator !(const Range__& r)
{
    return r.start == r.end;
}

static inline
Range__ operator & (const Range__& r1, const Range__& r2)
{
    Range__ r(max(r1.start, r2.start), min(r1.end, r2.end));
    r.end = max(r.end, r.start);
    return r;
}

static inline
Range__& operator &= (Range__& r1, const Range__& r2)
{
    r1 = r1 & r2;
    return r1;
}

static inline
Range__ operator + (const Range__& r1, int delta)
{
    return Range__(r1.start + delta, r1.end + delta);
}

static inline
Range__ operator + (int delta, const Range__& r1)
{
    return Range__(r1.start + delta, r1.end + delta);
}

static inline
Range__ operator - (const Range__& r1, int delta)
{
    return r1 + (-delta);
}




//-----------------------------------------------------------------------------//
template<typename _Tp, int cn> class Vec__ : public Matx<_Tp, cn, 1>
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

    Vec__(_Tp v0); //!< 1-element vector constructor
    Vec__(_Tp v0, _Tp v1); //!< 2-element vector constructor
    Vec__(_Tp v0, _Tp v1, _Tp v2); //!< 3-element vector constructor
    Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3); //!< 4-element vector constructor
    Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4); //!< 5-element vector constructor
    Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5); //!< 6-element vector constructor
    Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5, _Tp v6); //!< 7-element vector constructor
    Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5, _Tp v6, _Tp v7); //!< 8-element vector constructor
    Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5, _Tp v6, _Tp v7, _Tp v8); //!< 9-element vector constructor
    Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5, _Tp v6, _Tp v7, _Tp v8, _Tp v9); //!< 10-element vector constructor
    Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5, _Tp v6, _Tp v7, _Tp v8, _Tp v9, _Tp v10, _Tp v11, _Tp v12, _Tp v13); //!< 14-element vector constructor
    explicit Vec__(const _Tp* values);

    Vec__(std::initializer_list<_Tp>);

    Vec__(const Vec__<_Tp, cn>& v);

    static Vec__ all(_Tp alpha);

    //! per-element multiplication
    Vec__ mul(const Vec__<_Tp, cn>& v) const;

    //! conjugation (makes sense for complex numbers and quaternions)
    Vec__ conj() const;

    //!
      //cross product of the two 3D vectors.

      //For other dimensionalities the exception is raised
    
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
Vec__<_Tp, cn>::Vec__() {}

template<typename _Tp, int cn> inline
Vec__<_Tp, cn>::Vec__(_Tp v0)
    : Matx<_Tp, cn, 1>(v0) {}

template<typename _Tp, int cn> inline
Vec__<_Tp, cn>::Vec__(_Tp v0, _Tp v1)
    : Matx<_Tp, cn, 1>(v0, v1) {}

template<typename _Tp, int cn> inline
Vec__<_Tp, cn>::Vec__(_Tp v0, _Tp v1, _Tp v2)
    : Matx<_Tp, cn, 1>(v0, v1, v2) {}

template<typename _Tp, int cn> inline
Vec__<_Tp, cn>::Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3)
    : Matx<_Tp, cn, 1>(v0, v1, v2, v3) {}

template<typename _Tp, int cn> inline
Vec__<_Tp, cn>::Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4)
    : Matx<_Tp, cn, 1>(v0, v1, v2, v3, v4) {}

template<typename _Tp, int cn> inline
Vec__<_Tp, cn>::Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5)
    : Matx<_Tp, cn, 1>(v0, v1, v2, v3, v4, v5) {}

template<typename _Tp, int cn> inline
Vec__<_Tp, cn>::Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5, _Tp v6)
    : Matx<_Tp, cn, 1>(v0, v1, v2, v3, v4, v5, v6) {}

template<typename _Tp, int cn> inline
Vec__<_Tp, cn>::Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5, _Tp v6, _Tp v7)
    : Matx<_Tp, cn, 1>(v0, v1, v2, v3, v4, v5, v6, v7) {}

template<typename _Tp, int cn> inline
Vec__<_Tp, cn>::Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5, _Tp v6, _Tp v7, _Tp v8)
    : Matx<_Tp, cn, 1>(v0, v1, v2, v3, v4, v5, v6, v7, v8) {}

template<typename _Tp, int cn> inline
Vec__<_Tp, cn>::Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5, _Tp v6, _Tp v7, _Tp v8, _Tp v9)
    : Matx<_Tp, cn, 1>(v0, v1, v2, v3, v4, v5, v6, v7, v8, v9) {}

template<typename _Tp, int cn> inline
Vec__<_Tp, cn>::Vec__(_Tp v0, _Tp v1, _Tp v2, _Tp v3, _Tp v4, _Tp v5, _Tp v6, _Tp v7, _Tp v8, _Tp v9, _Tp v10, _Tp v11, _Tp v12, _Tp v13)
    : Matx<_Tp, cn, 1>(v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13) {}

template<typename _Tp, int cn> inline
Vec__<_Tp, cn>::Vec__(const _Tp* values)
    : Matx<_Tp, cn, 1>(values) {}

template<typename _Tp, int cn> inline
Vec__<_Tp, cn>::Vec__(std::initializer_list<_Tp> list)
    : Matx<_Tp, cn, 1>(list) {}

template<typename _Tp, int cn> inline
Vec__<_Tp, cn>::Vec__(const Vec__<_Tp, cn>& m)
    : Matx<_Tp, cn, 1>(m.val) {}

template<typename _Tp, int cn> inline
Vec__<_Tp, cn>::Vec__(const Matx<_Tp, cn, 1>& a, const Matx<_Tp, cn, 1>& b, Matx_AddOp op)
    : Matx<_Tp, cn, 1>(a, b, op) {}

template<typename _Tp, int cn> inline
Vec__<_Tp, cn>::Vec__(const Matx<_Tp, cn, 1>& a, const Matx<_Tp, cn, 1>& b, Matx_SubOp op)
    : Matx<_Tp, cn, 1>(a, b, op) {}

template<typename _Tp, int cn> template<typename _T2> inline
Vec__<_Tp, cn>::Vec__(const Matx<_Tp, cn, 1>& a, _T2 alpha, Matx_ScaleOp op)
    : Matx<_Tp, cn, 1>(a, alpha, op) {}

template<typename _Tp, int cn> inline
Vec__<_Tp, cn> Vec__<_Tp, cn>::all(_Tp alpha)
{
    Vec v;
    for (int i = 0; i < cn; i++) v.val[i] = alpha;
    return v;
}

template<typename _Tp, int cn> inline
Vec__<_Tp, cn> Vec__<_Tp, cn>::mul(const Vec__<_Tp, cn>& v) const
{
    Vec<_Tp, cn> w;
    for (int i = 0; i < cn; i++) w.val[i] = saturate_cast<_Tp>(this->val[i] * v.val[i]);
    return w;
}
/*
template<> inline
Vec__<float, 2> Vec__<float, 2>::conj() const
{
    return cv::internal::conjugate(*this);
}

template<> inline
Vec__<double, 2> Vec__<double, 2>::conj() const
{
    return cv::internal::conjugate(*this);
}

template<> inline
Vec__<float, 4> Vec__<float, 4>::conj() const
{
    return cv::internal::conjugate(*this);
}

template<> inline
Vec__<double, 4> Vec__<double, 4>::conj() const
{
    return cv::internal::conjugate(*this);
}*/

template<typename _Tp, int cn> inline
Vec__<_Tp, cn> Vec__<_Tp, cn>::cross(const Vec__<_Tp, cn>&) const
{
    CV_StaticAssert(cn == 3, "for arbitrary-size vector there is no cross-product defined");
    return Vec<_Tp, cn>();
}

template<> inline
Vec__<float, 3> Vec__<float, 3>::cross(const Vec__<float, 3>& v) const
{
    return Vec__<float, 3>(this->val[1] * v.val[2] - this->val[2] * v.val[1],
        this->val[2] * v.val[0] - this->val[0] * v.val[2],
        this->val[0] * v.val[1] - this->val[1] * v.val[0]);
}

template<> inline
Vec__<double, 3> Vec__<double, 3>::cross(const Vec__<double, 3>& v) const
{
    return Vec__<double, 3>(this->val[1] * v.val[2] - this->val[2] * v.val[1],
        this->val[2] * v.val[0] - this->val[0] * v.val[2],
        this->val[0] * v.val[1] - this->val[1] * v.val[0]);
}

template<typename _Tp, int cn> template<typename T2> inline
Vec__<_Tp, cn>::operator Vec__<T2, cn>() const
{
    Vec<T2, cn> v;
    for (int i = 0; i < cn; i++) v.val[i] = saturate_cast<T2>(this->val[i]);
    return v;
}

template<typename _Tp, int cn> inline
const _Tp& Vec__<_Tp, cn>::operator [](int i) const
{
    CV_DbgAssert((unsigned)i < (unsigned)cn);
    return this->val[i];
}

template<typename _Tp, int cn> inline
_Tp& Vec__<_Tp, cn>::operator [](int i)
{
    CV_DbgAssert((unsigned)i < (unsigned)cn);
    return this->val[i];
}

template<typename _Tp, int cn> inline
const _Tp& Vec__<_Tp, cn>::operator ()(int i) const
{
    CV_DbgAssert((unsigned)i < (unsigned)cn);
    return this->val[i];
}

template<typename _Tp, int cn> inline
_Tp& Vec__<_Tp, cn>::operator ()(int i)
{
    CV_DbgAssert((unsigned)i < (unsigned)cn);
    return this->val[i];
}

template<typename _Tp, int cn> inline
Vec__<_Tp, cn> normalize(const Vec__<_Tp, cn>& v)
{
    double nv = norm(v);
    return v * (nv ? 1. / nv : 0.);
}




//-----------------------------------------------------------------------------//

template<typename _Tp> class Scalar___ : public Vec<_Tp, 4>
{
public:
    //! default constructor
    Scalar___();
    Scalar___(_Tp v0, _Tp v1, _Tp v2 = 0, _Tp v3 = 0);
    Scalar___(_Tp v0);

    Scalar___(const Scalar___& s);
    Scalar___(Scalar___&& s) CV_NOEXCEPT;

    Scalar___& operator=(const Scalar___& s);
    Scalar___& operator=(Scalar___&& s) CV_NOEXCEPT;

    template<typename _Tp2, int cn>
    Scalar___(const Vec<_Tp2, cn>& v);

    //! returns a scalar with all elements set to v0
    static Scalar___<_Tp> all(_Tp v0);

    //! conversion to another data type
    template<typename T2> operator Scalar___<T2>() const;

    //! per-element product
    Scalar___<_Tp> mul(const Scalar___<_Tp>& a, double scale = 1) const;

    //! returns (v0, -v1, -v2, -v3)
    Scalar___<_Tp> conj() const;

    //! returns true iff v1 == v2 == v3 == 0
    bool isReal() const;
};

typedef Scalar___<double> Scalar__;

template<typename _Tp> inline
Scalar___<_Tp>::Scalar___()
{
    this->val[0] = this->val[1] = this->val[2] = this->val[3] = 0;
}

template<typename _Tp> inline
Scalar___<_Tp>::Scalar___(_Tp v0, _Tp v1, _Tp v2, _Tp v3)
{
    this->val[0] = v0;
    this->val[1] = v1;
    this->val[2] = v2;
    this->val[3] = v3;
}

template<typename _Tp> inline
Scalar___<_Tp>::Scalar___(const Scalar___<_Tp>& s) : Vec<_Tp, 4>(s) {
}

template<typename _Tp> inline
Scalar___<_Tp>::Scalar___(Scalar___<_Tp>&& s) CV_NOEXCEPT {
    this->val[0] = std::move(s.val[0]);
    this->val[1] = std::move(s.val[1]);
    this->val[2] = std::move(s.val[2]);
    this->val[3] = std::move(s.val[3]);
}

template<typename _Tp> inline
Scalar___<_Tp>& Scalar___<_Tp>::operator=(const Scalar___<_Tp>& s) {
    this->val[0] = s.val[0];
    this->val[1] = s.val[1];
    this->val[2] = s.val[2];
    this->val[3] = s.val[3];
    return *this;
}

template<typename _Tp> inline
Scalar___<_Tp>& Scalar___<_Tp>::operator=(Scalar___<_Tp>&& s) CV_NOEXCEPT {
    this->val[0] = std::move(s.val[0]);
    this->val[1] = std::move(s.val[1]);
    this->val[2] = std::move(s.val[2]);
    this->val[3] = std::move(s.val[3]);
    return *this;
}

template<typename _Tp> template<typename _Tp2, int cn> inline
Scalar___<_Tp>::Scalar___(const Vec<_Tp2, cn>& v)
{
    int i;
    for (i = 0; i < (cn < 4 ? cn : 4); i++)
        this->val[i] = cv::saturate_cast<_Tp>(v.val[i]);
    for (; i < 4; i++)
        this->val[i] = 0;
}

template<typename _Tp> inline
Scalar___<_Tp>::Scalar___(_Tp v0)
{
    this->val[0] = v0;
    this->val[1] = this->val[2] = this->val[3] = 0;
}

template<typename _Tp> inline
Scalar___<_Tp> Scalar___<_Tp>::all(_Tp v0)
{
    return Scalar___<_Tp>(v0, v0, v0, v0);
}


template<typename _Tp> inline
Scalar___<_Tp> Scalar___<_Tp>::mul(const Scalar___<_Tp>& a, double scale) const
{
    return Scalar___<_Tp>(saturate_cast<_Tp>(this->val[0] * a.val[0] * scale),
        saturate_cast<_Tp>(this->val[1] * a.val[1] * scale),
        saturate_cast<_Tp>(this->val[2] * a.val[2] * scale),
        saturate_cast<_Tp>(this->val[3] * a.val[3] * scale));
}

template<typename _Tp> inline
Scalar___<_Tp> Scalar___<_Tp>::conj() const
{
    return Scalar___<_Tp>(saturate_cast<_Tp>(this->val[0]),
        saturate_cast<_Tp>(-this->val[1]),
        saturate_cast<_Tp>(-this->val[2]),
        saturate_cast<_Tp>(-this->val[3]));
}

template<typename _Tp> inline
bool Scalar___<_Tp>::isReal() const
{
    return this->val[1] == 0 && this->val[2] == 0 && this->val[3] == 0;
}


template<typename _Tp> template<typename T2> inline
Scalar___<_Tp>::operator Scalar___<T2>() const
{
    return Scalar___<T2>(saturate_cast<T2>(this->val[0]),
        saturate_cast<T2>(this->val[1]),
        saturate_cast<T2>(this->val[2]),
        saturate_cast<T2>(this->val[3]));
}


template<typename _Tp> static inline
Scalar___<_Tp>& operator += (Scalar___<_Tp>& a, const Scalar___<_Tp>& b)
{
    a.val[0] += b.val[0];
    a.val[1] += b.val[1];
    a.val[2] += b.val[2];
    a.val[3] += b.val[3];
    return a;
}

template<typename _Tp> static inline
Scalar___<_Tp>& operator -= (Scalar___<_Tp>& a, const Scalar___<_Tp>& b)
{
    a.val[0] -= b.val[0];
    a.val[1] -= b.val[1];
    a.val[2] -= b.val[2];
    a.val[3] -= b.val[3];
    return a;
}

template<typename _Tp> static inline
Scalar___<_Tp>& operator *= (Scalar___<_Tp>& a, _Tp v)
{
    a.val[0] *= v;
    a.val[1] *= v;
    a.val[2] *= v;
    a.val[3] *= v;
    return a;
}

template<typename _Tp> static inline
bool operator == (const Scalar___<_Tp>& a, const Scalar___<_Tp>& b)
{
    return a.val[0] == b.val[0] && a.val[1] == b.val[1] &&
        a.val[2] == b.val[2] && a.val[3] == b.val[3];
}

template<typename _Tp> static inline
bool operator != (const Scalar___<_Tp>& a, const Scalar___<_Tp>& b)
{
    return a.val[0] != b.val[0] || a.val[1] != b.val[1] ||
        a.val[2] != b.val[2] || a.val[3] != b.val[3];
}

template<typename _Tp> static inline
Scalar___<_Tp> operator + (const Scalar___<_Tp>& a, const Scalar___<_Tp>& b)
{
    return Scalar___<_Tp>(a.val[0] + b.val[0],
        a.val[1] + b.val[1],
        a.val[2] + b.val[2],
        a.val[3] + b.val[3]);
}

template<typename _Tp> static inline
Scalar___<_Tp> operator - (const Scalar___<_Tp>& a, const Scalar___<_Tp>& b)
{
    return Scalar___<_Tp>(saturate_cast<_Tp>(a.val[0] - b.val[0]),
        saturate_cast<_Tp>(a.val[1] - b.val[1]),
        saturate_cast<_Tp>(a.val[2] - b.val[2]),
        saturate_cast<_Tp>(a.val[3] - b.val[3]));
}

template<typename _Tp> static inline
Scalar___<_Tp> operator * (const Scalar___<_Tp>& a, _Tp alpha)
{
    return Scalar___<_Tp>(a.val[0] * alpha,
        a.val[1] * alpha,
        a.val[2] * alpha,
        a.val[3] * alpha);
}

template<typename _Tp> static inline
Scalar___<_Tp> operator * (_Tp alpha, const Scalar___<_Tp>& a)
{
    return a * alpha;
}

template<typename _Tp> static inline
Scalar___<_Tp> operator - (const Scalar___<_Tp>& a)
{
    return Scalar___<_Tp>(saturate_cast<_Tp>(-a.val[0]),
        saturate_cast<_Tp>(-a.val[1]),
        saturate_cast<_Tp>(-a.val[2]),
        saturate_cast<_Tp>(-a.val[3]));
}


template<typename _Tp> static inline
Scalar___<_Tp> operator * (const Scalar___<_Tp>& a, const Scalar___<_Tp>& b)
{
    return Scalar___<_Tp>(saturate_cast<_Tp>(a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3]),
        saturate_cast<_Tp>(a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2]),
        saturate_cast<_Tp>(a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1]),
        saturate_cast<_Tp>(a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]));
}

template<typename _Tp> static inline
Scalar___<_Tp>& operator *= (Scalar___<_Tp>& a, const Scalar___<_Tp>& b)
{
    a = a * b;
    return a;
}

template<typename _Tp> static inline
Scalar___<_Tp> operator / (const Scalar___<_Tp>& a, _Tp alpha)
{
    return Scalar___<_Tp>(a.val[0] / alpha,
        a.val[1] / alpha,
        a.val[2] / alpha,
        a.val[3] / alpha);
}

template<typename _Tp> static inline
Scalar___<float> operator / (const Scalar___<float>& a, float alpha)
{
    float s = 1 / alpha;
    return Scalar___<float>(a.val[0] * s, a.val[1] * s, a.val[2] * s, a.val[3] * s);
}

template<typename _Tp> static inline
Scalar___<double> operator / (const Scalar___<double>& a, double alpha)
{
    double s = 1 / alpha;
    return Scalar___<double>(a.val[0] * s, a.val[1] * s, a.val[2] * s, a.val[3] * s);
}

template<typename _Tp> static inline
Scalar___<_Tp>& operator /= (Scalar___<_Tp>& a, _Tp alpha)
{
    a = a / alpha;
    return a;
}

template<typename _Tp> static inline
Scalar___<_Tp> operator / (_Tp a, const Scalar___<_Tp>& b)
{
    _Tp s = a / (b[0] * b[0] + b[1] * b[1] + b[2] * b[2] + b[3] * b[3]);
    return b.conj() * s;
}

template<typename _Tp> static inline
Scalar___<_Tp> operator / (const Scalar___<_Tp>& a, const Scalar___<_Tp>& b)
{
    return a * ((_Tp)1 / b);
}

template<typename _Tp> static inline
Scalar___<_Tp>& operator /= (Scalar___<_Tp>& a, const Scalar___<_Tp>& b)
{
    a = a / b;
    return a;
}
/*
template<typename _Tp> static inline
Scalar__ operator * (const Matx<_Tp, 4, 4>& a, const Scalar__& b)
{
    Matx<double, 4, 1> c((Matx<double, 4, 4>)a, b, Matx_MatMulOp());
    return reinterpret_cast<const Scalar__&>(c);
}

template<> inline
Scalar__ operator * (const Matx<double, 4, 4>& a, const Scalar__& b)
{
    Matx<double, 4, 1> c(a, b, Matx_MatMulOp());
    return reinterpret_cast<const Scalar__&>(c);
}
*/

