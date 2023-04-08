#pragma once
//#include <opencv2/opencv.hpp>
#include <utility>
#include "saturate_cast.h"

using namespace std;

template<typename _Tp> class Point___
{
public:
    typedef _Tp value_type;

    //! default constructor
    Point___();
    Point___(_Tp _x, _Tp _y);
    Point___(const Point___& pt);
    Point___(Point___&& pt) ;
    //Point___(const Size_<_Tp>& sz);
    //Point___(const Vec<_Tp, 2>& v);

    Point___& operator = (const Point___& pt);
    Point___& operator = (Point___&& pt) ;
    //! conversion to another data type
    /*template<typename _Tp2> operator Point___<_Tp2>() const;

    //! conversion to the old-style C structures
    operator Vec<_Tp, 2>() const;
    
    //! dot product
    _Tp dot(const Point___& pt) const;
    //! dot product computed in double-precision arithmetics
    double ddot(const Point___& pt) const;
    //! cross-product
    double cross(const Point___& pt) const;
    //! checks whether the point is inside the specified rectangle
    bool inside(const Rect_<_Tp>& r) const;*/
    _Tp x; //!< x coordinate of the point
    _Tp y; //!< y coordinate of the point
};

typedef Point___<int> Point2i_;
//typedef Point___<int64> Point2l;
typedef Point___<float> Point2f_;
//typedef Point___<double> Point2d;
typedef Point2i_ Point__;



template<typename _Tp> inline
Point___<_Tp>::Point___()
    : x(0), y(0) {}

template<typename _Tp> inline
Point___<_Tp>::Point___(_Tp _x, _Tp _y)
    : x(_x), y(_y) {}

template<typename _Tp> inline
Point___<_Tp>::Point___(const Point___& pt)
    : x(pt.x), y(pt.y) {}

template<typename _Tp> inline
Point___<_Tp>::Point___(Point___&& pt)
    : x(std::move(pt.x)), y(std::move(pt.y)) {}


template<typename _Tp> inline
Point___<_Tp>& Point___<_Tp>::operator = (const Point___& pt)
{
    x = pt.x; y = pt.y;
    return *this;
}

template<typename _Tp> inline
Point___<_Tp>& Point___<_Tp>::operator = (Point___&& pt) 
{
    x = std::move(pt.x); y = std::move(pt.y);
    return *this;
}

/*template<typename _Tp> template<typename _Tp2> inline
Point___<_Tp>::operator Point___<_Tp2>() const
{
    return Point_<_Tp2>(saturate_cast<_Tp2>(x), saturate_cast<_Tp2>(y));
}

template<typename _Tp> inline
Point___<_Tp>::operator Vec<_Tp, 2>() const
{
    return Vec<_Tp, 2>(x, y);
}*/

template<typename _Tp> static inline
bool operator == (const Point___<_Tp>& a, const Point___<_Tp>& b)
{
    return a.x == b.x && a.y == b.y;
}

template<typename _Tp> static inline
Point___<_Tp>& operator *= (Point___<_Tp>& a, float b)
{
    a.x = saturate_cast<_Tp>(a.x * b);
    a.y = saturate_cast<_Tp>(a.y * b);
    return a;
}
