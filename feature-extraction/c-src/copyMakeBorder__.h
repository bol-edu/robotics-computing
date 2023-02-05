#pragma once
#include <opencv2/opencv.hpp>

#include "Point__.h"
using namespace std;

#define CV_CN_MAX     512
#define CV_CN_SHIFT   3
#define CV_DEPTH_MAX  (1 << CV_CN_SHIFT)


#define CV_MAT_DEPTH_MASK       (CV_DEPTH_MAX - 1)
#define CV_MAT_DEPTH(flags)     ((flags) & CV_MAT_DEPTH_MASK)

#define CV_MAKETYPE(depth,cn) (CV_MAT_DEPTH(depth) + (((cn)-1) << CV_CN_SHIFT))

#define CV_MAT_CN_MASK          ((CV_CN_MAX - 1) << CV_CN_SHIFT)
#define CV_MAT_CN(flags)        ((((flags) & CV_MAT_CN_MASK) >> CV_CN_SHIFT) + 1)

#  define CV_OVERRIDE override
#    define CV_INLINE static inline
#  define CV_ENABLE_UNROLLED 1

//typedef unsigned char uchar;

enum BorderTypes {
    BORDER_CONSTANT_ = 0, //!< `iiiiii|abcdefgh|iiiiiii`  with some specified `i`
    BORDER_REPLICATE_ = 1, //!< `aaaaaa|abcdefgh|hhhhhhh`
    BORDER_REFLECT_ = 2, //!< `fedcba|abcdefgh|hgfedcb`
    BORDER_WRAP_ = 3, //!< `cdefgh|abcdefgh|abcdefg`
    BORDER_REFLECT_101_ = 4, //!< `gfedcb|abcdefgh|gfedcba`
    BORDER_TRANSPARENT_ = 5, //!< `uvwxyz|abcdefgh|ijklmno`

    BORDER_REFLECT101_ = BORDER_REFLECT_101_, //!< same as BORDER_REFLECT_101
    BORDER_DEFAULT_ = BORDER_REFLECT_101_, //!< same as BORDER_REFLECT_101
    BORDER_ISOLATED_ = 16 //!< do not look outside of ROI
};

CV_EXPORTS_W void copyMakeBorder__(InputArray src, OutputArray dst,
    int top, int bottom, int left, int right,
    int borderType, const Scalar& value = Scalar());


int borderInterpolate_(int p, int len, int borderType)
{
    //CV_TRACE_FUNCTION_VERBOSE();

   // CV_DbgAssert(len > 0);

#ifdef CV_STATIC_ANALYSIS
    if (p >= 0 && p < len)
#else
    if ((unsigned)p < (unsigned)len)
#endif
        ;
    else if (borderType == 1)   //  else if (borderType == BORDER_REPLICATE)
        p = p < 0 ? 0 : len - 1;
    else if (borderType == 2 || borderType == 4)    //else if (borderType == BORDER_REFLECT || borderType == BORDER_REFLECT_101)
    {
        int delta = borderType == 4;    // int delta = borderType == BORDER_REFLECT_101;
        if (len == 1)
            return 0;
        do
        {
            if (p < 0)
                p = -p - 1 + delta;
            else
                p = len - 1 - (p - len) - delta;
        }
#ifdef CV_STATIC_ANALYSIS
        while (p < 0 || p >= len);
#else
        while ((unsigned)p >= (unsigned)len);
#endif
    }
    else if (borderType == 3)  //else if (borderType == BORDER_WRAP)
    {
        //CV_Assert(len > 0);
        if (p < 0)
            p -= ((p - len + 1) / len) * len;
        if (p >= len)
            p %= len;
    }
    else if (borderType == 0)  // else if (borderType == BORDER_CONSTANT)
        p = -1;
    else
        /*CV_Error(CV_StsBadArg, "Unknown/unsupported border type")*/;
    return p;
}


void copyMakeBorder_8u(const uchar* src, size_t srcstep, cv::Size srcroi,
    uchar* dst, size_t dststep, cv::Size dstroi,
    int top, int left, int cn, int borderType)
{
    const int isz = (int)sizeof(int);
    int i, j, k, elemSize = 1;
    bool intMode = false;

    if ((cn | srcstep | dststep | (size_t)src | (size_t)dst) % isz == 0)
    {
        cn /= isz;
        elemSize = isz;
        intMode = true;
    }

    cv::AutoBuffer<int> _tab((dstroi.width - srcroi.width) * cn);
    int* tab = _tab.data();
    int right = dstroi.width - srcroi.width - left;
    int bottom = dstroi.height - srcroi.height - top;

    for (i = 0; i < left; i++)
    {
        j = borderInterpolate_(i - left, srcroi.width, borderType) * cn;
        for (k = 0; k < cn; k++)
            tab[i * cn + k] = j + k;
    }

    for (i = 0; i < right; i++)
    {
        j = borderInterpolate_(srcroi.width + i, srcroi.width, borderType) * cn;
        for (k = 0; k < cn; k++)
            tab[(i + left) * cn + k] = j + k;
    }

    srcroi.width *= cn;
    dstroi.width *= cn;
    left *= cn;
    right *= cn;

    uchar* dstInner = dst + dststep * top + left * elemSize;

    for (i = 0; i < srcroi.height; i++, dstInner += dststep, src += srcstep)
    {
        if (dstInner != src)
            memcpy(dstInner, src, srcroi.width * elemSize);

        if (intMode)
        {
            const int* isrc = (int*)src;
            int* idstInner = (int*)dstInner;
            for (j = 0; j < left; j++)
                idstInner[j - left] = isrc[tab[j]];
            for (j = 0; j < right; j++)
                idstInner[j + srcroi.width] = isrc[tab[j + left]];
        }
        else
        {
            for (j = 0; j < left; j++)
                dstInner[j - left] = src[tab[j]];
            for (j = 0; j < right; j++)
                dstInner[j + srcroi.width] = src[tab[j + left]];
        }
    }

    dstroi.width *= elemSize;
    dst += dststep * top;

    for (i = 0; i < top; i++)
    {
        j = borderInterpolate_(i - top, srcroi.height, borderType);
        memcpy(dst + (i - top) * dststep, dst + j * dststep, dstroi.width);
    }

    for (i = 0; i < bottom; i++)
    {
        j = borderInterpolate_(i + srcroi.height, srcroi.height, borderType);
        memcpy(dst + (i + srcroi.height) * dststep, dst + j * dststep, dstroi.width);
    }
}

template <typename T> static inline
void scalarToRawData_(const Scalar& s, T* const buf, const int cn, const int unroll_to)
{
    int i = 0;
    for (; i < cn; i++)
        buf[i] = saturate_cast<T>(s.val[i]);
    for (; i < unroll_to; i++)
        buf[i] = buf[i - cn];
}

void scalarToRawData(const Scalar& s, void* _buf, int type, int unroll_to)
{
    //CV_INSTRUMENT_REGION();

    const int depth = CV_MAT_DEPTH(type), cn = CV_MAT_CN(type);
    //CV_Assert(cn <= 4);
    //cout << "depth " << depth << endl; 0
    switch (depth)
    {
    case CV_8U:
        scalarToRawData_<uchar>(s, (uchar*)_buf, cn, unroll_to);
        break;
    /*case CV_8S:
        scalarToRawData_<schar>(s, (schar*)_buf, cn, unroll_to);
        break;
    case CV_16U:
        scalarToRawData_<ushort>(s, (ushort*)_buf, cn, unroll_to);
        break;
    case CV_16S:
        scalarToRawData_<short>(s, (short*)_buf, cn, unroll_to);
        break;
    case CV_32S:
        scalarToRawData_<int>(s, (int*)_buf, cn, unroll_to);
        break;
    case CV_32F:
        scalarToRawData_<float>(s, (float*)_buf, cn, unroll_to);
        break;
    case CV_64F:
        scalarToRawData_<double>(s, (double*)_buf, cn, unroll_to);
        break;
    case CV_16F:
        scalarToRawData_<float16_t>(s, (float16_t*)_buf, cn, unroll_to);
        break;*/
    default:
        ;//CV_Error(CV_StsUnsupportedFormat, "");
    }
}


void copyMakeConstBorder_8u(const uchar* src, size_t srcstep, cv::Size srcroi,
    uchar* dst, size_t dststep, cv::Size dstroi,
    int top, int left, int cn, const uchar* value)
{
    int i, j;
    cv::AutoBuffer<uchar> _constBuf(dstroi.width * cn);
    uchar* constBuf = _constBuf.data();
    int right = dstroi.width - srcroi.width - left;
    int bottom = dstroi.height - srcroi.height - top;

    for (i = 0; i < dstroi.width; i++)
    {
        for (j = 0; j < cn; j++)
            constBuf[i * cn + j] = value[j];
    }

    srcroi.width *= cn;
    dstroi.width *= cn;
    left *= cn;
    right *= cn;

    uchar* dstInner = dst + dststep * top + left;

    for (i = 0; i < srcroi.height; i++, dstInner += dststep, src += srcstep)
    {
        if (dstInner != src)
            memcpy(dstInner, src, srcroi.width);
        memcpy(dstInner - left, constBuf, left);
        memcpy(dstInner + srcroi.width, constBuf, right);
    }

    dst += dststep * top;

    for (i = 0; i < top; i++)
        memcpy(dst + (i - top) * dststep, constBuf, dstroi.width);

    for (i = 0; i < bottom; i++)
        memcpy(dst + (i + srcroi.height) * dststep, constBuf, dstroi.width);
}

void copyMakeBorder__(InputArray _src, OutputArray _dst, int top, int bottom,
    int left, int right, int borderType, const Scalar& value)
{
   // CV_INSTRUMENT_REGION();

    //CV_Assert(top >= 0 && bottom >= 0 && left >= 0 && right >= 0 && _src.dims() <= 2);

    //CV_OCL_RUN(_dst.isUMat(),
    //    ocl_copyMakeBorder(_src, _dst, top, bottom, left, right, borderType, value))

        Mat src = _src.getMat();
    int type = src.type();

    if (src.isSubmatrix() && (borderType & BORDER_ISOLATED_) == 0)
    {
        Size wholeSize;
        Point ofs;
        src.locateROI(wholeSize, ofs);
        int dtop = min(ofs.y, top);
        int dbottom = min(wholeSize.height - src.rows - ofs.y, bottom);
        int dleft = min(ofs.x, left);
        int dright = min(wholeSize.width - src.cols - ofs.x, right);
        src.adjustROI(dtop, dbottom, dleft, dright);
        top -= dtop;
        left -= dleft;
        bottom -= dbottom;
        right -= dright;
    }

    _dst.create(src.rows + top + bottom, src.cols + left + right, type);
    Mat dst = _dst.getMat();

    if (top == 0 && left == 0 && bottom == 0 && right == 0)
    {
        if (src.data != dst.data || src.step != dst.step)
            src.copyTo(dst);
        return;
    }

    borderType &= ~BORDER_ISOLATED_;
   // cout << "borderType: " << borderType << endl;  4, 0

    //CV_IPP_RUN_FAST(ipp_copyMakeBorder(src, dst, top, bottom, left, right, borderType, value))

        if (borderType != 0){     // if (borderType != BORDER_CONSTANT){
            copyMakeBorder_8u(src.ptr(), src.step, src.size(),
                dst.ptr(), dst.step, dst.size(),
                top, left, (int)src.elemSize(), borderType);
            
        }
        else
        {
            
            int cn = src.channels(), cn1 = cn;
            AutoBuffer<double> buf(cn);
            if (cn > 4)
            {
                //CV_Assert(value[0] == value[1] && value[0] == value[2] && value[0] == value[3]);
                cn1 = 1;
            }
            scalarToRawData(value, buf.data(), CV_MAKETYPE(src.depth(), cn1), cn);
            copyMakeConstBorder_8u(src.ptr(), src.step, src.size(),
                dst.ptr(), dst.step, dst.size(),
                top, left, (int)src.elemSize(), (uchar*)buf.data());
        }
}