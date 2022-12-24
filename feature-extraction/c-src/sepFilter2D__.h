#pragma once
#include <opencv2/opencv.hpp>
#include "Point__.h"
//#include "v_float32x4.h"

using namespace std;

enum
{
    KERNEL_GENERAL = 0, // the kernel is generic. No any type of symmetry or other properties.
    KERNEL_SYMMETRICAL = 1, // kernel[i] == kernel[ksize-i-1] , and the anchor is at the center
    KERNEL_ASYMMETRICAL = 2, // kernel[i] == -kernel[ksize-i-1] , and the anchor is at the center
    KERNEL_SMOOTH = 4, // all the kernel elements are non-negative and summed to 1
    KERNEL_INTEGER = 8  // all the kernel coefficients are integer numbers
};



struct cvhalFilter2D {};

#define cv_hal_sepFilterInit hal_ni_sepFilterInit
#define cv_hal_sepFilter hal_ni_sepFilter
#define cv_hal_sepFilterFree hal_ni_sepFilterFree


inline int hal_ni_sepFilterInit(cvhalFilter2D** context, int src_type, int dst_type, int kernel_type, uchar* kernelx_data, int kernelx_length, uchar* kernely_data, int kernely_length, int anchor_x, int anchor_y, double delta, int borderType) { return CV_HAL_ERROR_NOT_IMPLEMENTED; }
inline int hal_ni_sepFilter(cvhalFilter2D* context, uchar* src_data, size_t src_step, uchar* dst_data, size_t dst_step, int width, int height, int full_width, int full_height, int offset_x, int offset_y) { return CV_HAL_ERROR_NOT_IMPLEMENTED; }

inline int hal_ni_sepFilterFree(cvhalFilter2D* context) { return CV_HAL_ERROR_NOT_IMPLEMENTED; }

static bool replacementSepFilter(int stype, int dtype, int ktype,
    uchar* src_data, size_t src_step, uchar* dst_data, size_t dst_step,
    int width, int height, int full_width, int full_height,
    int offset_x, int offset_y,
    uchar* kernelx_data, int kernelx_len,
    uchar* kernely_data, int kernely_len,
    int anchor_x, int anchor_y, double delta, int borderType)
{
    cvhalFilter2D* ctx;
    int res = cv_hal_sepFilterInit(&ctx, stype, dtype, ktype,
        kernelx_data, kernelx_len,
        kernely_data, kernely_len,
        anchor_x, anchor_y, delta, borderType);
    if (res != CV_HAL_ERROR_OK)
        return false;
    res = cv_hal_sepFilter(ctx, src_data, src_step, dst_data, dst_step, width, height, full_width, full_height, offset_x, offset_y);
    bool success = (res == CV_HAL_ERROR_OK);
    res = cv_hal_sepFilterFree(ctx);
    if (res != CV_HAL_ERROR_OK)
        return false;
    return success;
}



class BaseFilter
{
public:
    //! the default constructor
    BaseFilter();
    //! the destructor
    virtual ~BaseFilter();
    //! the filtering operator. The horizontal and the vertical border interpolation is done outside of the class.
    virtual void operator()(const uchar** src, uchar* dst, int dststep, int dstcount, int width, int cn) = 0;
    //! resets the internal buffers, if any
    virtual void reset();

    Size ksize;
    Point anchor;
};

BaseFilter::BaseFilter() { ksize = Size(-1, -1); anchor = Point(-1, -1); }
BaseFilter::~BaseFilter() {}
void BaseFilter::reset() {}


class BaseRowFilter
{
public:
    //! the default constructor
    BaseRowFilter();
    //! the destructor
    virtual ~BaseRowFilter();
    //! the filtering operator. Must be overridden in the derived classes. The horizontal border interpolation is done outside of the class.
    virtual void operator()(const uchar* src, uchar* dst, int width, int cn) = 0;

    int ksize;
    int anchor;
};

BaseRowFilter::BaseRowFilter() { ksize = anchor = -1; }
BaseRowFilter::~BaseRowFilter() {}

class BaseColumnFilter
{
public:
    //! the default constructor
    BaseColumnFilter();
    //! the destructor
    virtual ~BaseColumnFilter();
    //! the filtering operator. Must be overridden in the derived classes. The vertical border interpolation is done outside of the class.
    virtual void operator()(const uchar** src, uchar* dst, int dststep, int dstcount, int width) = 0;
    //! resets the internal buffers, if any
    virtual void reset();

    int ksize;
    int anchor;
};

BaseColumnFilter::BaseColumnFilter() { ksize = anchor = -1; }
BaseColumnFilter::~BaseColumnFilter() {}
void BaseColumnFilter::reset() {}

class FilterEngine
{
public:
    //! the default constructor
    FilterEngine();
    //! the full constructor. Either _filter2D or both _rowFilter and _columnFilter must be non-empty.
    FilterEngine(const Ptr<BaseFilter>& _filter2D,
        const Ptr<BaseRowFilter>& _rowFilter,
        const Ptr<BaseColumnFilter>& _columnFilter,
        int srcType, int dstType, int bufType,
        int _rowBorderType = BORDER_REPLICATE,
        int _columnBorderType = -1,
        const Scalar& _borderValue = Scalar());
    //! the destructor
    //virtual ~FilterEngine();
    //! reinitializes the engine. The previously assigned filters are released.
    void init(const Ptr<BaseFilter>& _filter2D,
        const Ptr<BaseRowFilter>& _rowFilter,
        const Ptr<BaseColumnFilter>& _columnFilter,
        int srcType, int dstType, int bufType,
        int _rowBorderType = BORDER_REPLICATE,
        int _columnBorderType = -1,
        const Scalar& _borderValue = Scalar());

    //! starts filtering of the specified ROI of an image of size wholeSize.
    /*virtual int start(const cv::Size& wholeSize, const cv::Size& sz, const cv::Point& ofs);
    //! starts filtering of the specified ROI of the specified image.
    virtual int start(const Mat& src, const cv::Size& wsz, const cv::Point& ofs);
    //! processes the next srcCount rows of the image.
    virtual int proceed(const uchar* src, int srcStep, int srcCount,
        uchar* dst, int dstStep);*/
        //! applies filter to the specified ROI of the image. if srcRoi=(0,0,-1,-1), the whole image is filtered.
    virtual void apply(const Mat& src, Mat& dst, const cv::Size& wsz, const cv::Point& ofs);

    //! returns true if the filter is separable
    bool isSeparable() const { return !filter2D; }
    //! returns the number
    int remainingInputRows() const;
    int remainingOutputRows() const;

    int srcType;
    int dstType;
    int bufType;
    Size ksize;
    Point anchor;
    int maxWidth;
    Size wholeSize;
    Rect roi;
    int dx1;
    int dx2;
    int rowBorderType;
    int columnBorderType;
    std::vector<int> borderTab;
    int borderElemSize;
    std::vector<uchar> ringBuf;
    std::vector<uchar> srcRow;
    std::vector<uchar> constBorderValue;
    std::vector<uchar> constBorderRow;
    int bufStep;
    int startY;
    int startY0;
    int endY;
    int rowCount;
    int dstY;
    std::vector<uchar*> rows;

    Ptr<BaseFilter> filter2D;
    Ptr<BaseRowFilter> rowFilter;
    Ptr<BaseColumnFilter> columnFilter;
};

FilterEngine::FilterEngine()
    : srcType(-1), dstType(-1), bufType(-1), maxWidth(0), wholeSize(-1, -1), dx1(0), dx2(0),
    rowBorderType(BORDER_REPLICATE), columnBorderType(BORDER_REPLICATE),
    borderElemSize(0), bufStep(0), startY(0), startY0(0), endY(0), rowCount(0), dstY(0)
{
}


FilterEngine::FilterEngine(const Ptr<BaseFilter>& _filter2D,
    const Ptr<BaseRowFilter>& _rowFilter,
    const Ptr<BaseColumnFilter>& _columnFilter,
    int _srcType, int _dstType, int _bufType,
    int _rowBorderType, int _columnBorderType,
    const Scalar& _borderValue)
    : srcType(-1), dstType(-1), bufType(-1), maxWidth(0), wholeSize(-1, -1), dx1(0), dx2(0),
    rowBorderType(BORDER_REPLICATE), columnBorderType(BORDER_REPLICATE),
    borderElemSize(0), bufStep(0), startY(0), startY0(0), endY(0), rowCount(0), dstY(0)
{
    init(_filter2D, _rowFilter, _columnFilter, _srcType, _dstType, _bufType,
        _rowBorderType, _columnBorderType, _borderValue);
}

/*FilterEngine::~FilterEngine()
{
}*/


void FilterEngine::init(const Ptr<BaseFilter>& _filter2D,
    const Ptr<BaseRowFilter>& _rowFilter,
    const Ptr<BaseColumnFilter>& _columnFilter,
    int _srcType, int _dstType, int _bufType,
    int _rowBorderType, int _columnBorderType,
    const Scalar& _borderValue)
{
    _srcType = CV_MAT_TYPE(_srcType);
    _bufType = CV_MAT_TYPE(_bufType);
    _dstType = CV_MAT_TYPE(_dstType);

    srcType = _srcType;
    int srcElemSize = (int)getElemSize(srcType);
    dstType = _dstType;
    bufType = _bufType;

    filter2D = _filter2D;
    rowFilter = _rowFilter;
    columnFilter = _columnFilter;

    if (_columnBorderType < 0)
        _columnBorderType = _rowBorderType;

    rowBorderType = _rowBorderType;
    columnBorderType = _columnBorderType;

    CV_Assert(columnBorderType != BORDER_WRAP);

    if (isSeparable())
    {
        CV_Assert(rowFilter && columnFilter);
        ksize = Size(rowFilter->ksize, columnFilter->ksize);
        anchor = Point(rowFilter->anchor, columnFilter->anchor);
    }
    else
    {
        CV_Assert(bufType == srcType);
        ksize = filter2D->ksize;
        anchor = filter2D->anchor;
    }

    CV_Assert(0 <= anchor.x && anchor.x < ksize.width &&
        0 <= anchor.y && anchor.y < ksize.height);

    borderElemSize = srcElemSize / (CV_MAT_DEPTH(srcType) >= CV_32S ? sizeof(int) : 1);
    int borderLength = max(ksize.width - 1, 1);
    borderTab.resize(borderLength * borderElemSize);

    maxWidth = bufStep = 0;
    constBorderRow.clear();

    if (rowBorderType == BORDER_CONSTANT || columnBorderType == BORDER_CONSTANT)
    {
        constBorderValue.resize(srcElemSize * borderLength);
        int srcType1 = CV_MAKETYPE(CV_MAT_DEPTH(srcType), MIN(CV_MAT_CN(srcType), 4));
        scalarToRawData(_borderValue, &constBorderValue[0], srcType1,
            borderLength * CV_MAT_CN(srcType));
    }

    wholeSize = Size(-1, -1);
}


int FilterEngine::remainingInputRows() const
{
    return endY - startY - rowCount;
}

int FilterEngine::remainingOutputRows() const
{
    return roi.height - dstY;
}


/*
//#ifndef __CV_EXPAND
#define __CV_EXPAND(x) x
//#endif

#define __CV_CPU_DISPATCH_CHAIN_END(fn, args, mode, ...)  // done 
#define __CV_CPU_DISPATCH(fn, args, mode, ...) __CV_EXPAND(__CV_CPU_DISPATCH_CHAIN_ ## mode(fn, args, __VA_ARGS__))
#define __CV_CPU_DISPATCH_EXPAND(fn, args, ...) __CV_EXPAND(__CV_CPU_DISPATCH(fn, args, __VA_ARGS__))
#define CV_CPU_DISPATCH(fn, args, ...) __CV_CPU_DISPATCH_EXPAND(fn, args, __VA_ARGS__, END) // expand macros
*/

#define VEC_ALIGN 64

int FilterEngine__start(FilterEngine& this_, const Size& _wholeSize, const Size& sz, const Point& ofs)
{
    //CV_INSTRUMENT_REGION();

    int i, j;

    this_.wholeSize = _wholeSize;
    this_.roi = Rect(ofs, sz);
    CV_Assert(this_.roi.x >= 0 && this_.roi.y >= 0 && this_.roi.width >= 0 && this_.roi.height >= 0 &&
        this_.roi.x + this_.roi.width <= this_.wholeSize.width &&
        this_.roi.y + this_.roi.height <= this_.wholeSize.height);

    int esz = (int)getElemSize(this_.srcType);
    int bufElemSize = (int)getElemSize(this_.bufType);
    const uchar* constVal = !this_.constBorderValue.empty() ? &this_.constBorderValue[0] : 0;

    int _maxBufRows = max(this_.ksize.height + 3,
        max(this_.anchor.y,
            this_.ksize.height - this_.anchor.y - 1) * 2 + 1);

    if (this_.maxWidth < this_.roi.width || _maxBufRows != (int)this_.rows.size())
    {
        this_.rows.resize(_maxBufRows);
        this_.maxWidth = max(this_.maxWidth, this_.roi.width);
        int cn = CV_MAT_CN(this_.srcType);
        this_.srcRow.resize(esz * (this_.maxWidth + this_.ksize.width - 1));
        if (this_.columnBorderType == BORDER_CONSTANT)
        {
            CV_Assert(constVal != NULL);
            this_.constBorderRow.resize(getElemSize(this_.bufType) * (this_.maxWidth + this_.ksize.width - 1 + VEC_ALIGN));
            uchar* dst = alignPtr(&this_.constBorderRow[0], VEC_ALIGN);
            int n = (int)this_.constBorderValue.size();
            int N = (this_.maxWidth + this_.ksize.width - 1) * esz;
            uchar* tdst = this_.isSeparable() ? &this_.srcRow[0] : dst;

            for (i = 0; i < N; i += n)
            {
                n = min(n, N - i);
                for (j = 0; j < n; j++)
                    tdst[i + j] = constVal[j];
            }

            if (this_.isSeparable())
                (*this_.rowFilter)(&this_.srcRow[0], dst, this_.maxWidth, cn);
        }

        int maxBufStep = bufElemSize * (int)alignSize(this_.maxWidth +
            (!this_.isSeparable() ? this_.ksize.width - 1 : 0), VEC_ALIGN);
        this_.ringBuf.resize(maxBufStep * this_.rows.size() + VEC_ALIGN);
    }

    // adjust bufstep so that the used part of the ring buffer stays compact in memory
    this_.bufStep = bufElemSize * (int)alignSize(this_.roi.width + (!this_.isSeparable() ? this_.ksize.width - 1 : 0), VEC_ALIGN);

    this_.dx1 = max(this_.anchor.x - this_.roi.x, 0);
    this_.dx2 = max(this_.ksize.width - this_.anchor.x - 1 + this_.roi.x + this_.roi.width - this_.wholeSize.width, 0);

    // recompute border tables
    if (this_.dx1 > 0 || this_.dx2 > 0)
    {
        if (this_.rowBorderType == BORDER_CONSTANT)
        {
            CV_Assert(constVal != NULL);
            int nr = this_.isSeparable() ? 1 : (int)this_.rows.size();
            for (i = 0; i < nr; i++)
            {
                uchar* dst = this_.isSeparable() ? &this_.srcRow[0] : alignPtr(&this_.ringBuf[0], VEC_ALIGN) + this_.bufStep * i;
                memcpy(dst, constVal, this_.dx1 * esz);
                memcpy(dst + (this_.roi.width + this_.ksize.width - 1 - this_.dx2) * esz, constVal, this_.dx2 * esz);
            }
        }
        else
        {
            int xofs1 = min(this_.roi.x, this_.anchor.x) - this_.roi.x;

            int btab_esz = this_.borderElemSize, wholeWidth = this_.wholeSize.width;
            int* btab = (int*)&this_.borderTab[0];

            for (i = 0; i < this_.dx1; i++)
            {
                int p0 = (borderInterpolate(i - this_.dx1, wholeWidth, this_.rowBorderType) + xofs1) * btab_esz;
                for (j = 0; j < btab_esz; j++)
                    btab[i * btab_esz + j] = p0 + j;
            }

            for (i = 0; i < this_.dx2; i++)
            {
                int p0 = (borderInterpolate(wholeWidth + i, wholeWidth, this_.rowBorderType) + xofs1) * btab_esz;
                for (j = 0; j < btab_esz; j++)
                    btab[(i + this_.dx1) * btab_esz + j] = p0 + j;
            }
        }
    }

    this_.rowCount = this_.dstY = 0;
    this_.startY = this_.startY0 = max(this_.roi.y - this_.anchor.y, 0);
    this_.endY = min(this_.roi.y + this_.roi.height + this_.ksize.height - this_.anchor.y - 1, this_.wholeSize.height);

    if (this_.columnFilter)
        this_.columnFilter->reset();
    if (this_.filter2D)
        this_.filter2D->reset();

    return this_.startY;
}


int FilterEngine__proceed(FilterEngine& this_, const uchar* src, int srcstep, int count,
    uchar* dst, int dststep)
{
    //CV_INSTRUMENT_REGION();

    CV_DbgAssert(this_.wholeSize.width > 0 && this_.wholeSize.height > 0);

    const int* btab = &this_.borderTab[0];
    int esz = (int)getElemSize(this_.srcType), btab_esz = this_.borderElemSize;
    uchar** brows = &this_.rows[0];
    int bufRows = (int)this_.rows.size();
    int cn = CV_MAT_CN(this_.bufType);
    int width = this_.roi.width, kwidth = this_.ksize.width;
    int kheight = this_.ksize.height, ay = this_.anchor.y;
    int _dx1 = this_.dx1, _dx2 = this_.dx2;
    int width1 = this_.roi.width + kwidth - 1;
    int xofs1 = min(this_.roi.x, this_.anchor.x);
    bool isSep = this_.isSeparable();
    bool makeBorder = (_dx1 > 0 || _dx2 > 0) && this_.rowBorderType != BORDER_CONSTANT;
    int dy = 0, i = 0;

    src -= xofs1 * esz;
    count = min(count, this_.remainingInputRows());

    CV_Assert(src && dst && count > 0);

    for (;; dst += dststep * i, dy += i)
    {
        int dcount = bufRows - ay - this_.startY - this_.rowCount + this_.roi.y;
        dcount = dcount > 0 ? dcount : bufRows - kheight + 1;
        dcount = min(dcount, count);
        count -= dcount;
        for (; dcount-- > 0; src += srcstep)
        {
            int bi = (this_.startY - this_.startY0 + this_.rowCount) % bufRows;
            uchar* brow = alignPtr(&this_.ringBuf[0], VEC_ALIGN) + bi * this_.bufStep;
            uchar* row = isSep ? &this_.srcRow[0] : brow;

            if (++this_.rowCount > bufRows)
            {
                --this_.rowCount;
                ++this_.startY;
            }

            memcpy(row + _dx1 * esz, src, (width1 - _dx2 - _dx1) * esz);

            if (makeBorder)
            {
                if (btab_esz * (int)sizeof(int) == esz)
                {
                    const int* isrc = (const int*)src;
                    int* irow = (int*)row;

                    for (i = 0; i < _dx1 * btab_esz; i++)
                        irow[i] = isrc[btab[i]];
                    for (i = 0; i < _dx2 * btab_esz; i++)
                        irow[i + (width1 - _dx2) * btab_esz] = isrc[btab[i + _dx1 * btab_esz]];
                }
                else
                {
                    for (i = 0; i < _dx1 * esz; i++)
                        row[i] = src[btab[i]];
                    for (i = 0; i < _dx2 * esz; i++)
                        row[i + (width1 - _dx2) * esz] = src[btab[i + _dx1 * esz]];
                }
            }

            if (isSep)
                (*this_.rowFilter)(row, brow, width, CV_MAT_CN(this_.srcType));
        }

        int max_i = min(bufRows, this_.roi.height - (this_.dstY + dy) + (kheight - 1));
        for (i = 0; i < max_i; i++)
        {
            int srcY = borderInterpolate(this_.dstY + dy + i + this_.roi.y - ay,
                this_.wholeSize.height, this_.columnBorderType);
            if (srcY < 0) // can happen only with constant border type
                brows[i] = alignPtr(&this_.constBorderRow[0], VEC_ALIGN);
            else
            {
                CV_Assert(srcY >= this_.startY);
                if (srcY >= this_.startY + this_.rowCount)
                    break;
                int bi = (srcY - this_.startY0) % bufRows;
                brows[i] = alignPtr(&this_.ringBuf[0], VEC_ALIGN) + bi * this_.bufStep;
            }
        }
        if (i < kheight)
            break;
        i -= kheight - 1;
        if (isSep)
            (*this_.columnFilter)((const uchar**)brows, dst, dststep, i, this_.roi.width * cn);
        else
            (*this_.filter2D)((const uchar**)brows, dst, dststep, i, this_.roi.width, cn);
    }

    this_.dstY += dy;
    CV_Assert(this_.dstY <= this_.roi.height);
    return dy;
}

void FilterEngine__apply(FilterEngine& this_, const Mat& src, Mat& dst, const Size& wsz, const Point& ofs)
{
    //CV_INSTRUMENT_REGION();

    CV_DbgAssert(src.type() == this_.srcType && dst.type() == this_.dstType);

    FilterEngine__start(this_, wsz, src.size(), ofs);
    int y = this_.startY - ofs.y;
    FilterEngine__proceed(this_,
        src.ptr() + y * src.step,
        (int)src.step,
        this_.endY - this_.startY,
        dst.ptr(),
        (int)dst.step);
}

void FilterEngine::apply(const Mat& src, Mat& dst, const Size& wsz, const Point& ofs)
{
    //CV_INSTRUMENT_REGION();

    CV_CheckTypeEQ(src.type(), srcType, "");
    CV_CheckTypeEQ(dst.type(), dstType, "");
    FilterEngine__apply(*this, src, dst, wsz, ofs);
    /*CV_CPU_DISPATCH(FilterEngine__apply, (*this, src, dst, wsz, ofs),
        CV_CPU_DISPATCH_MODES_ALL);*/
}

template<typename ST, typename DT, class VecOp> struct RowFilter : public BaseRowFilter
{
    RowFilter(const Mat& _kernel, int _anchor, const VecOp& _vecOp = VecOp())
    {
        if (_kernel.isContinuous())
            kernel = _kernel;
        else
            _kernel.copyTo(kernel);
        anchor = _anchor;
        ksize = kernel.rows + kernel.cols - 1;
        CV_Assert(kernel.type() == DataType<DT>::type &&
            (kernel.rows == 1 || kernel.cols == 1));
        vecOp = _vecOp;
    }

    void operator()(const uchar* src, uchar* dst, int width, int cn) CV_OVERRIDE
    {
        //CV_INSTRUMENT_REGION();

        int _ksize = ksize;
        const DT* kx = kernel.ptr<DT>();
        const ST* S;
        DT* D = (DT*)dst;
        int i, k;

        i = vecOp(src, dst, width, cn);
        width *= cn;
#if CV_ENABLE_UNROLLED
        for (; i <= width - 4; i += 4)
        {
            S = (const ST*)src + i;
            DT f = kx[0];
            DT s0 = f * S[0], s1 = f * S[1], s2 = f * S[2], s3 = f * S[3];

            for (k = 1; k < _ksize; k++)
            {
                S += cn;
                f = kx[k];
                s0 += f * S[0]; s1 += f * S[1];
                s2 += f * S[2]; s3 += f * S[3];
            }

            D[i] = s0; D[i + 1] = s1;
            D[i + 2] = s2; D[i + 3] = s3;
        }
#endif
        for (; i < width; i++)
        {
            S = (const ST*)src + i;
            DT s0 = kx[0] * S[0];
            for (k = 1; k < _ksize; k++)
            {
                S += cn;
                s0 += kx[k] * S[0];
            }
            D[i] = s0;
        }
    }

    Mat kernel;
    VecOp vecOp;
};

template<class CastOp, class VecOp> struct ColumnFilter : public BaseColumnFilter
{
    typedef typename CastOp::type1 ST;
    typedef typename CastOp::rtype DT;

    ColumnFilter(const Mat& _kernel, int _anchor,
        double _delta, const CastOp& _castOp = CastOp(),
        const VecOp& _vecOp = VecOp())
    {
        if (_kernel.isContinuous())
            kernel = _kernel;
        else
            _kernel.copyTo(kernel);
        anchor = _anchor;
        ksize = kernel.rows + kernel.cols - 1;
        delta = saturate_cast<ST>(_delta);
        castOp0 = _castOp;
        vecOp = _vecOp;
        CV_Assert(kernel.type() == DataType<ST>::type &&
            (kernel.rows == 1 || kernel.cols == 1));
    }

    void operator()(const uchar** src, uchar* dst, int dststep, int count, int width) CV_OVERRIDE
    {
        //CV_INSTRUMENT_REGION();

        const ST* ky = kernel.template ptr<ST>();
        ST _delta = delta;
        int _ksize = ksize;
        int i, k;
        CastOp castOp = castOp0;

        for (; count--; dst += dststep, src++)
        {
            DT* D = (DT*)dst;
            i = vecOp(src, dst, width);
#if CV_ENABLE_UNROLLED
            for (; i <= width - 4; i += 4)
            {
                ST f = ky[0];
                const ST* S = (const ST*)src[0] + i;
                ST s0 = f * S[0] + _delta, s1 = f * S[1] + _delta,
                    s2 = f * S[2] + _delta, s3 = f * S[3] + _delta;

                for (k = 1; k < _ksize; k++)
                {
                    S = (const ST*)src[k] + i; f = ky[k];
                    s0 += f * S[0]; s1 += f * S[1];
                    s2 += f * S[2]; s3 += f * S[3];
                }

                D[i] = castOp(s0); D[i + 1] = castOp(s1);
                D[i + 2] = castOp(s2); D[i + 3] = castOp(s3);
            }
#endif
            for (; i < width; i++)
            {
                ST s0 = ky[0] * ((const ST*)src[0])[i] + _delta;
                for (k = 1; k < _ksize; k++)
                    s0 += ky[k] * ((const ST*)src[k])[i];
                D[i] = castOp(s0);
            }
        }
    }

    Mat kernel;
    CastOp castOp0;
    VecOp vecOp;
    ST delta;
};


template<class CastOp, class VecOp> struct SymmColumnFilter : public ColumnFilter<CastOp, VecOp>
{
    typedef typename CastOp::type1 ST;
    typedef typename CastOp::rtype DT;

    SymmColumnFilter(const Mat& _kernel, int _anchor,
        double _delta, int _symmetryType,
        const CastOp& _castOp = CastOp(),
        const VecOp& _vecOp = VecOp())
        : ColumnFilter<CastOp, VecOp>(_kernel, _anchor, _delta, _castOp, _vecOp)
    {
        symmetryType = _symmetryType;
        CV_Assert((symmetryType & (KERNEL_SYMMETRICAL | KERNEL_ASYMMETRICAL)) != 0);
    }

    void operator()(const uchar** src, uchar* dst, int dststep, int count, int width) CV_OVERRIDE
    {
        //CV_INSTRUMENT_REGION();

        int ksize2 = this->ksize / 2;
        const ST* ky = this->kernel.template ptr<ST>() + ksize2;
        int i, k;
        bool symmetrical = (symmetryType & KERNEL_SYMMETRICAL) != 0;
        ST _delta = this->delta;
        CastOp castOp = this->castOp0;
        src += ksize2;

        if (symmetrical)
        {
            for (; count--; dst += dststep, src++)
            {
                DT* D = (DT*)dst;
                i = (this->vecOp)(src, dst, width);
#if CV_ENABLE_UNROLLED
                for (; i <= width - 4; i += 4)
                {
                    ST f = ky[0];
                    const ST* S = (const ST*)src[0] + i, * S2;
                    ST s0 = f * S[0] + _delta, s1 = f * S[1] + _delta,
                        s2 = f * S[2] + _delta, s3 = f * S[3] + _delta;

                    for (k = 1; k <= ksize2; k++)
                    {
                        S = (const ST*)src[k] + i;
                        S2 = (const ST*)src[-k] + i;
                        f = ky[k];
                        s0 += f * (S[0] + S2[0]);
                        s1 += f * (S[1] + S2[1]);
                        s2 += f * (S[2] + S2[2]);
                        s3 += f * (S[3] + S2[3]);
                    }

                    D[i] = castOp(s0); D[i + 1] = castOp(s1);
                    D[i + 2] = castOp(s2); D[i + 3] = castOp(s3);
                }
#endif
                for (; i < width; i++)
                {
                    ST s0 = ky[0] * ((const ST*)src[0])[i] + _delta;
                    for (k = 1; k <= ksize2; k++)
                        s0 += ky[k] * (((const ST*)src[k])[i] + ((const ST*)src[-k])[i]);
                    D[i] = castOp(s0);
                }
            }
        }
        else
        {
            for (; count--; dst += dststep, src++)
            {
                DT* D = (DT*)dst;
                i = this->vecOp(src, dst, width);
#if CV_ENABLE_UNROLLED
                for (; i <= width - 4; i += 4)
                {
                    ST f = ky[0];
                    const ST* S, * S2;
                    ST s0 = _delta, s1 = _delta, s2 = _delta, s3 = _delta;

                    for (k = 1; k <= ksize2; k++)
                    {
                        S = (const ST*)src[k] + i;
                        S2 = (const ST*)src[-k] + i;
                        f = ky[k];
                        s0 += f * (S[0] - S2[0]);
                        s1 += f * (S[1] - S2[1]);
                        s2 += f * (S[2] - S2[2]);
                        s3 += f * (S[3] - S2[3]);
                    }

                    D[i] = castOp(s0); D[i + 1] = castOp(s1);
                    D[i + 2] = castOp(s2); D[i + 3] = castOp(s3);
                }
#endif
                for (; i < width; i++)
                {
                    ST s0 = _delta;
                    for (k = 1; k <= ksize2; k++)
                        s0 += ky[k] * (((const ST*)src[k])[i] - ((const ST*)src[-k])[i]);
                    D[i] = castOp(s0);
                }
            }
        }
    }

    int symmetryType;
};

template<typename ST, typename DT, class VecOp> struct SymmRowSmallFilter : public RowFilter<ST, DT, VecOp>
{
    SymmRowSmallFilter(const Mat& _kernel, int _anchor, int _symmetryType,
        const VecOp& _vecOp = VecOp())
        : RowFilter<ST, DT, VecOp>(_kernel, _anchor, _vecOp)
    {
        symmetryType = _symmetryType;
        CV_Assert((symmetryType & (KERNEL_SYMMETRICAL | KERNEL_ASYMMETRICAL)) != 0 && this->ksize <= 5);
    }

    void operator()(const uchar* src, uchar* dst, int width, int cn) CV_OVERRIDE
    {
        //CV_INSTRUMENT_REGION();

        int ksize2 = this->ksize / 2, ksize2n = ksize2 * cn;
        const DT* kx = this->kernel.template ptr<DT>() + ksize2;
        bool symmetrical = (this->symmetryType & KERNEL_SYMMETRICAL) != 0;
        DT* D = (DT*)dst;
        int i = this->vecOp(src, dst, width, cn), j, k;
        const ST* S = (const ST*)src + i + ksize2n;
        width *= cn;

        if (symmetrical)
        {
            if (this->ksize == 1 && kx[0] == 1)
            {
                for (; i <= width - 2; i += 2)
                {
                    DT s0 = S[i], s1 = S[i + 1];
                    D[i] = s0; D[i + 1] = s1;
                }
                S += i;
            }
            else if (this->ksize == 3)
            {
                if (kx[0] == 2 && kx[1] == 1)
                    for (; i <= width - 2; i += 2, S += 2)
                    {
                        DT s0 = S[-cn] + S[0] * 2 + S[cn], s1 = S[1 - cn] + S[1] * 2 + S[1 + cn];
                        D[i] = s0; D[i + 1] = s1;
                    }
                else if (kx[0] == -2 && kx[1] == 1)
                    for (; i <= width - 2; i += 2, S += 2)
                    {
                        DT s0 = S[-cn] - S[0] * 2 + S[cn], s1 = S[1 - cn] - S[1] * 2 + S[1 + cn];
                        D[i] = s0; D[i + 1] = s1;
                    }
                else
                {
                    DT k0 = kx[0], k1 = kx[1];
                    for (; i <= width - 2; i += 2, S += 2)
                    {
                        DT s0 = S[0] * k0 + (S[-cn] + S[cn]) * k1, s1 = S[1] * k0 + (S[1 - cn] + S[1 + cn]) * k1;
                        D[i] = s0; D[i + 1] = s1;
                    }
                }
            }
            else if (this->ksize == 5)
            {
                DT k0 = kx[0], k1 = kx[1], k2 = kx[2];
                if (k0 == -2 && k1 == 0 && k2 == 1)
                    for (; i <= width - 2; i += 2, S += 2)
                    {
                        DT s0 = -2 * S[0] + S[-cn * 2] + S[cn * 2];
                        DT s1 = -2 * S[1] + S[1 - cn * 2] + S[1 + cn * 2];
                        D[i] = s0; D[i + 1] = s1;
                    }
                else
                    for (; i <= width - 2; i += 2, S += 2)
                    {
                        DT s0 = S[0] * k0 + (S[-cn] + S[cn]) * k1 + (S[-cn * 2] + S[cn * 2]) * k2;
                        DT s1 = S[1] * k0 + (S[1 - cn] + S[1 + cn]) * k1 + (S[1 - cn * 2] + S[1 + cn * 2]) * k2;
                        D[i] = s0; D[i + 1] = s1;
                    }
            }

            for (; i < width; i++, S++)
            {
                DT s0 = kx[0] * S[0];
                for (k = 1, j = cn; k <= ksize2; k++, j += cn)
                    s0 += kx[k] * (S[j] + S[-j]);
                D[i] = s0;
            }
        }
        else
        {
            if (this->ksize == 3)
            {
                if (kx[0] == 0 && kx[1] == 1)
                    for (; i <= width - 2; i += 2, S += 2)
                    {
                        DT s0 = S[cn] - S[-cn], s1 = S[1 + cn] - S[1 - cn];
                        D[i] = s0; D[i + 1] = s1;
                    }
                else
                {
                    DT k1 = kx[1];
                    for (; i <= width - 2; i += 2, S += 2)
                    {
                        DT s0 = (S[cn] - S[-cn]) * k1, s1 = (S[1 + cn] - S[1 - cn]) * k1;
                        D[i] = s0; D[i + 1] = s1;
                    }
                }
            }
            else if (this->ksize == 5)
            {
                DT k1 = kx[1], k2 = kx[2];
                for (; i <= width - 2; i += 2, S += 2)
                {
                    DT s0 = (S[cn] - S[-cn]) * k1 + (S[cn * 2] - S[-cn * 2]) * k2;
                    DT s1 = (S[1 + cn] - S[1 - cn]) * k1 + (S[1 + cn * 2] - S[1 - cn * 2]) * k2;
                    D[i] = s0; D[i + 1] = s1;
                }
            }

            for (; i < width; i++, S++)
            {
                DT s0 = kx[0] * S[0];
                for (k = 1, j = cn; k <= ksize2; k++, j += cn)
                    s0 += kx[k] * (S[j] - S[-j]);
                D[i] = s0;
            }
        }
    }

    int symmetryType;
};


struct v_float32x4_
{
    typedef float lane_type;
    typedef __m128 vector_type;
    enum { nlanes = 4 };

    // coverity[uninit_ctor]: suppress warning 
    v_float32x4_() {}
    explicit v_float32x4_(__m128 v) : val(v) {}
    v_float32x4_(float v0, float v1, float v2, float v3)
    {
        val = _mm_setr_ps(v0, v1, v2, v3);
    }

    float get0() const
    {
        return _mm_cvtss_f32(val);
    }

    __m128 val;
};

template<typename _Tp, int n> struct v_reg
{
    //! @cond IGNORED
    typedef _Tp lane_type;
    enum { nlanes = n };
    // !@endcond

        /** @brief Constructor

        Initializes register with data from memory
        @param ptr pointer to memory block with data for register */
    explicit v_reg(const _Tp* ptr) { for (int i = 0; i < n; i++) s[i] = ptr[i]; }

    /** @brief Constructor

    Initializes register with two 64-bit values */
    v_reg(_Tp s0, _Tp s1) { s[0] = s0; s[1] = s1; }

    /** @brief Constructor

    Initializes register with four 32-bit values */
    v_reg(_Tp s0, _Tp s1, _Tp s2, _Tp s3) { s[0] = s0; s[1] = s1; s[2] = s2; s[3] = s3; }

    /** @brief Constructor

    Initializes register with eight 16-bit values */
    v_reg(_Tp s0, _Tp s1, _Tp s2, _Tp s3,
        _Tp s4, _Tp s5, _Tp s6, _Tp s7)
    {
        s[0] = s0; s[1] = s1; s[2] = s2; s[3] = s3;
        s[4] = s4; s[5] = s5; s[6] = s6; s[7] = s7;
    }

    /** @brief Constructor

    Initializes register with sixteen 8-bit values */
    v_reg(_Tp s0, _Tp s1, _Tp s2, _Tp s3,
        _Tp s4, _Tp s5, _Tp s6, _Tp s7,
        _Tp s8, _Tp s9, _Tp s10, _Tp s11,
        _Tp s12, _Tp s13, _Tp s14, _Tp s15)
    {
        s[0] = s0; s[1] = s1; s[2] = s2; s[3] = s3;
        s[4] = s4; s[5] = s5; s[6] = s6; s[7] = s7;
        s[8] = s8; s[9] = s9; s[10] = s10; s[11] = s11;
        s[12] = s12; s[13] = s13; s[14] = s14; s[15] = s15;
    }

    /** @brief Default constructor

    Does not initialize anything*/
    v_reg() {}

    /** @brief Copy constructor */
    v_reg(const v_reg<_Tp, n>& r)
    {
        for (int i = 0; i < n; i++)
            s[i] = r.s[i];
    }
    /** @brief Access first value

    Returns value of the first lane according to register type, for example:
    @code{.cpp}
    v_int32x4 r(1, 2, 3, 4);
    int v = r.get0(); // returns 1
    v_uint64x2 r(1, 2);
    uint64_t v = r.get0(); // returns 1
    @endcode
    */
    _Tp get0() const { return s[0]; }

    //! @cond IGNORED
    _Tp get(const int i) const { return s[i]; }
    v_reg<_Tp, n> high() const
    {
        v_reg<_Tp, n> c;
        int i;
        for (i = 0; i < n / 2; i++)
        {
            c.s[i] = s[i + (n / 2)];
            c.s[i + (n / 2)] = 0;
        }
        return c;
    }

    static v_reg<_Tp, n> zero()
    {
        v_reg<_Tp, n> c;
        for (int i = 0; i < n; i++)
            c.s[i] = (_Tp)0;
        return c;
    }

    static v_reg<_Tp, n> all(_Tp s)
    {
        v_reg<_Tp, n> c;
        for (int i = 0; i < n; i++)
            c.s[i] = s;
        return c;
    }

    template<typename _Tp2, int n2> v_reg<_Tp2, n2> reinterpret_as() const
    {
        size_t bytes = min(sizeof(_Tp2) * n2, sizeof(_Tp) * n);
        v_reg<_Tp2, n2> c;
        std::memcpy(&c.s[0], &s[0], bytes);
        return c;
    }

    v_reg& operator=(const v_reg<_Tp, n>& r)
    {
        for (int i = 0; i < n; i++)
            s[i] = r.s[i];
        return *this;
    }

    _Tp s[n];
    //! @endcond
};

/** @brief Sixteen 8-bit unsigned integer values */
//typedef v_reg<uchar, 16> v_uint8x16;
/** @brief Sixteen 8-bit signed integer values */
//typedef v_reg<schar, 16> v_int8x16;
/** @brief Eight 16-bit unsigned integer values */
//typedef v_reg<ushort, 8> v_uint16x8;
/** @brief Eight 16-bit signed integer values */
//typedef v_reg<short, 8> v_int16x8;
/** @brief Four 32-bit unsigned integer values */
/*typedef v_reg<unsigned, 4> v_uint32x4;
/** @brief Four 32-bit signed integer values */
//typedef v_reg<int, 4> v_int32x4;
/** @brief Four 32-bit floating point values (single precision) */
typedef v_reg<float, 4> v_float32x4;
/** @brief Two 64-bit floating point values (double precision) */
//typedef v_reg<double, 2> v_float64x2;
/** @brief Two 64-bit unsigned integer values */
//typedef v_reg<uint64, 2> v_uint64x2;
/** @brief Two 64-bit signed integer values */
//typedef v_reg<int64, 2> v_int64x2;

#define __CV_EXPAND(x) x

template<typename _Tp, int n> CV_INLINE v_reg<_Tp, n> operator+(const v_reg<_Tp, n>& a, const v_reg<_Tp, n>& b);
//template<typename _Tp, int n> CV_INLINE v_reg<_Tp, n>& operator+=(v_reg<_Tp, n>& a, const v_reg<_Tp, n>& b);

/** @brief Subtract values

For all types. */
template<typename _Tp, int n> CV_INLINE v_reg<_Tp, n> operator-(const v_reg<_Tp, n>& a, const v_reg<_Tp, n>& b);
//template<typename _Tp, int n> CV_INLINE v_reg<_Tp, n>& operator-=(v_reg<_Tp, n>& a, const v_reg<_Tp, n>& b);

/** @brief Multiply values

For 16- and 32-bit integer types and floating types. */
template<typename _Tp, int n> CV_INLINE v_reg<_Tp, n> operator*(const v_reg<_Tp, n>& a, const v_reg<_Tp, n>& b);
//template<typename _Tp, int n> CV_INLINE v_reg<_Tp, n>& operator*=(v_reg<_Tp, n>& a, const v_reg<_Tp, n>& b);





template<typename _Tp> struct V_TypeTraits
{
};

//#define CV_INTRIN_DEF_TYPE_TRAITS(type, int_type_, uint_type_, abs_type_, w_type_, q_type_, sum_type_, nlanes128_) \
    template<> struct V_TypeTraits<type> \
    { \
        typedef type value_type; \
        typedef int_type_ int_type; \
        typedef abs_type_ abs_type; \
        typedef uint_type_ uint_type; \
        typedef w_type_ w_type; \
        typedef q_type_ q_type; \
        typedef sum_type_ sum_type; \
        enum { nlanes128 = nlanes128_ }; \
    \
        static inline int_type reinterpret_int(type x) \
        { \
            union { type l; int_type i; } v; \
            v.l = x; \
            return v.i; \
        } \
    \
        static inline type reinterpret_from_int(int_type x) \
        { \
            union { type l; int_type i; } v; \
            v.i = x; \
            return v.l; \
        } \
    }

#define CV_INTRIN_DEF_TYPE_TRAITS_NO_Q_TYPE(type, int_type_, uint_type_, abs_type_, w_type_, sum_type_, nlanes128_) \
    template<> struct V_TypeTraits<type> \
    { \
        typedef type value_type; \
        typedef int_type_ int_type; \
        typedef abs_type_ abs_type; \
        typedef uint_type_ uint_type; \
        typedef w_type_ w_type; \
        typedef sum_type_ sum_type; \
        enum { nlanes128 = nlanes128_ }; \
    \
        static inline int_type reinterpret_int(type x) \
        { \
            union { type l; int_type i; } v; \
            v.l = x; \
            return v.i; \
        } \
    \
        static inline type reinterpret_from_int(int_type x) \
        { \
            union { type l; int_type i; } v; \
            v.i = x; \
            return v.l; \
        } \
    }

/*CV_INTRIN_DEF_TYPE_TRAITS(uchar, schar, uchar, uchar, ushort, unsigned, unsigned, 16);
CV_INTRIN_DEF_TYPE_TRAITS(schar, schar, uchar, uchar, short, int, int, 16);
CV_INTRIN_DEF_TYPE_TRAITS(ushort, short, ushort, ushort, unsigned, uint64, unsigned, 8);
//CV_INTRIN_DEF_TYPE_TRAITS(short, short, ushort, ushort, int, int64, int, 8);
CV_INTRIN_DEF_TYPE_TRAITS_NO_Q_TYPE(unsigned, int, unsigned, unsigned, uint64, unsigned, 4);*/
//CV_INTRIN_DEF_TYPE_TRAITS_NO_Q_TYPE(int, int, unsigned, unsigned, int64, int, 4);
CV_INTRIN_DEF_TYPE_TRAITS_NO_Q_TYPE(float, int, unsigned, float, double, float, 4);
//CV_INTRIN_DEF_TYPE_TRAITS_NO_Q_TYPE(uint64, int64, uint64, uint64, void, uint64, 2);
//CV_INTRIN_DEF_TYPE_TRAITS_NO_Q_TYPE(int64, int64, uint64, uint64, void, int64, 2);
//CV_INTRIN_DEF_TYPE_TRAITS_NO_Q_TYPE(double, int64, uint64, double, void, double, 2);


//__CV_EXPAND(macro_name(int64, __VA_ARGS__)) \
//__CV_EXPAND(macro_name(uint64, __VA_ARGS__)) \
//__CV_EXPAND(macro_name(uchar, __VA_ARGS__)) \
//__CV_EXPAND(macro_name(schar, __VA_ARGS__)) \
//__CV_EXPAND(macro_name(ushort, __VA_ARGS__)) \
//__CV_EXPAND(macro_name(short, __VA_ARGS__)) \

#define CV__HAL_INTRIN_EXPAND_WITH_INTEGER_TYPES(macro_name, ...) \
__CV_EXPAND(macro_name(unsigned, __VA_ARGS__)) \
__CV_EXPAND(macro_name(int, __VA_ARGS__)) \

#define CV__HAL_INTRIN_EXPAND_WITH_FP_TYPES(macro_name, ...) \
__CV_EXPAND(macro_name(float, __VA_ARGS__)) \
__CV_EXPAND(macro_name(double, __VA_ARGS__)) \

#define CV__HAL_INTRIN_EXPAND_WITH_ALL_TYPES(macro_name, ...) \
CV__HAL_INTRIN_EXPAND_WITH_INTEGER_TYPES(macro_name, __VA_ARGS__) \
CV__HAL_INTRIN_EXPAND_WITH_FP_TYPES(macro_name, __VA_ARGS__) \

#define CV__HAL_INTRIN_IMPL_BIN_OP_(_Tp, bin_op) \
template<int n> inline \
v_reg<_Tp, n> operator bin_op (const v_reg<_Tp, n>& a, const v_reg<_Tp, n>& b) \
{ \
    v_reg<_Tp, n> c; \
    for( int i = 0; i < n; i++ ) \
        c.s[i] = saturate_cast<_Tp>(a.s[i] bin_op b.s[i]); \
    return c; \
} //\
template<int n> inline \
v_reg<_Tp, n>& operator bin_op##= (v_reg<_Tp, n>& a, const v_reg<_Tp, n>& b) \
{ \
    for( int i = 0; i < n; i++ ) \
        a.s[i] = saturate_cast<_Tp>(a.s[i] bin_op b.s[i]); \
    return a; \
}

#define CV__HAL_INTRIN_IMPL_BIN_OP(bin_op) CV__HAL_INTRIN_EXPAND_WITH_ALL_TYPES(CV__HAL_INTRIN_IMPL_BIN_OP_, bin_op)

CV__HAL_INTRIN_IMPL_BIN_OP(+)
CV__HAL_INTRIN_IMPL_BIN_OP(-)
CV__HAL_INTRIN_IMPL_BIN_OP(*)
CV__HAL_INTRIN_EXPAND_WITH_FP_TYPES(CV__HAL_INTRIN_IMPL_BIN_OP_, / )

#define CV__HAL_INTRIN_IMPL_BIT_OP_(_Tp, bit_op) \
template<int n> CV_INLINE \
v_reg<_Tp, n> operator bit_op (const v_reg<_Tp, n>& a, const v_reg<_Tp, n>& b) \
{ \
    v_reg<_Tp, n> c; \
    typedef typename V_TypeTraits<_Tp>::int_type itype; \
    for( int i = 0; i < n; i++ ) \
        c.s[i] = V_TypeTraits<_Tp>::reinterpret_from_int((itype)(V_TypeTraits<_Tp>::reinterpret_int(a.s[i]) bit_op \
                                                        V_TypeTraits<_Tp>::reinterpret_int(b.s[i]))); \
    return c; \
} //\
template<int n> CV_INLINE \
v_reg<_Tp, n>& operator bit_op##= (v_reg<_Tp, n>& a, const v_reg<_Tp, n>& b) \
{ \
    typedef typename V_TypeTraits<_Tp>::int_type itype; \
    for( int i = 0; i < n; i++ ) \
        a.s[i] = V_TypeTraits<_Tp>::reinterpret_from_int((itype)(V_TypeTraits<_Tp>::reinterpret_int(a.s[i]) bit_op \
                                                        V_TypeTraits<_Tp>::reinterpret_int(b.s[i]))); \
    return a; \
}



#define OPENCV_HAL_IMPL_C_INIT_VAL(_Tpvec, _Tp, suffix) \
inline _Tpvec v_setall_##suffix(_Tp val) { return _Tpvec::all(val); }

//! @name Init with value
//! @{
//! @brief Create new vector with elements set to a specific value
/*OPENCV_HAL_IMPL_C_INIT_VAL(v_uint8x16, uchar, u8)
OPENCV_HAL_IMPL_C_INIT_VAL(v_int8x16, schar, s8)
OPENCV_HAL_IMPL_C_INIT_VAL(v_uint16x8, ushort, u16)
OPENCV_HAL_IMPL_C_INIT_VAL(v_int16x8, short, s16)*/
//OPENCV_HAL_IMPL_C_INIT_VAL(v_uint32x4, unsigned, u32)
//OPENCV_HAL_IMPL_C_INIT_VAL(v_int32x4, int, s32)
OPENCV_HAL_IMPL_C_INIT_VAL(v_float32x4, float, f32)
//OPENCV_HAL_IMPL_C_INIT_VAL(v_float64x2, double, f64)
//OPENCV_HAL_IMPL_C_INIT_VAL(v_uint64x2, uint64, u64)
//OPENCV_HAL_IMPL_C_INIT_VAL(v_int64x2, int64, s64)


/*#define OPENCV_HAL_IMPL_C_INIT_ZERO(_Tpvec, _Tp, suffix) \
inline _Tpvec v_setzero_##suffix() { return _Tpvec::zero(); }

//! @name Init with zero
//! @{
//! @brief Create new vector with zero elements
OPENCV_HAL_IMPL_C_INIT_ZERO(v_uint8x16, uchar, u8)
OPENCV_HAL_IMPL_C_INIT_ZERO(v_int8x16, schar, s8)
OPENCV_HAL_IMPL_C_INIT_ZERO(v_uint16x8, ushort, u16)
OPENCV_HAL_IMPL_C_INIT_ZERO(v_int16x8, short, s16)
OPENCV_HAL_IMPL_C_INIT_ZERO(v_uint32x4, unsigned, u32)
OPENCV_HAL_IMPL_C_INIT_ZERO(v_int32x4, int, s32)
OPENCV_HAL_IMPL_C_INIT_ZERO(v_float32x4, float, f32)
OPENCV_HAL_IMPL_C_INIT_ZERO(v_float64x2, double, f64)
OPENCV_HAL_IMPL_C_INIT_ZERO(v_uint64x2, uint64, u64)
OPENCV_HAL_IMPL_C_INIT_ZERO(v_int64x2, int64, s64)*/
//! @}

template<typename _Tp>
inline v_reg<_Tp, V_TypeTraits<_Tp>::nlanes128> v_load(const _Tp* ptr)
{
#if CV_STRONG_ALIGNMENT
    CV_Assert(isAligned<sizeof(_Tp)>(ptr));
#endif
    return v_reg<_Tp, V_TypeTraits<_Tp>::nlanes128>(ptr);
}

/** @brief Load register contents from memory (aligned)

similar to cv::v_load, but source memory block should be aligned (to 16-byte boundary in case of SIMD128, 32-byte - SIMD256, etc)
 */
/*template<typename _Tp>
inline v_reg<_Tp, V_TypeTraits<_Tp>::nlanes128> v_load_aligned(const _Tp* ptr)
{
    CV_Assert(isAligned<sizeof(v_reg<_Tp, V_TypeTraits<_Tp>::nlanes128>)>(ptr));
    return v_reg<_Tp, V_TypeTraits<_Tp>::nlanes128>(ptr);
}


template<typename _Tp>
inline v_reg<_Tp, V_TypeTraits<_Tp>::nlanes128> v_load_low(const _Tp* ptr)
{
#if CV_STRONG_ALIGNMENT
    CV_Assert(isAligned<sizeof(_Tp)>(ptr));
#endif
    v_reg<_Tp, V_TypeTraits<_Tp>::nlanes128> c;
    for (int i = 0; i < c.nlanes / 2; i++)
    {
        c.s[i] = ptr[i];
    }
    return c;
}


template<typename _Tp>
inline v_reg<_Tp, V_TypeTraits<_Tp>::nlanes128> v_load_halves(const _Tp* loptr, const _Tp* hiptr)
{
#if CV_STRONG_ALIGNMENT
    CV_Assert(isAligned<sizeof(_Tp)>(loptr));
    CV_Assert(isAligned<sizeof(_Tp)>(hiptr));
#endif
    v_reg<_Tp, V_TypeTraits<_Tp>::nlanes128> c;
    for (int i = 0; i < c.nlanes / 2; i++)
    {
        c.s[i] = loptr[i];
        c.s[i + c.nlanes / 2] = hiptr[i];
    }
    return c;
}


template<typename _Tp>
inline v_reg<typename V_TypeTraits<_Tp>::w_type, V_TypeTraits<_Tp>::nlanes128 / 2>
v_load_expand(const _Tp* ptr)
{
#if CV_STRONG_ALIGNMENT
    CV_Assert(isAligned<sizeof(_Tp)>(ptr));
#endif
    typedef typename V_TypeTraits<_Tp>::w_type w_type;
    v_reg<w_type, V_TypeTraits<w_type>::nlanes128> c;
    for (int i = 0; i < c.nlanes; i++)
    {
        c.s[i] = ptr[i];
    }
    return c;
}


template<typename _Tp>
inline v_reg<typename V_TypeTraits<_Tp>::q_type, V_TypeTraits<_Tp>::nlanes128 / 4>
v_load_expand_q(const _Tp* ptr)
{
#if CV_STRONG_ALIGNMENT
    CV_Assert(isAligned<sizeof(_Tp)>(ptr));
#endif
    typedef typename V_TypeTraits<_Tp>::q_type q_type;
    v_reg<q_type, V_TypeTraits<q_type>::nlanes128> c;
    for (int i = 0; i < c.nlanes; i++)
    {
        c.s[i] = ptr[i];
    }
    return c;
}

template<typename _Tp> inline v_reg<_Tp, V_TypeTraits<_Tp>::nlanes128> v_lut_quads(const _Tp* tab, const int* idx)
{
    v_reg<_Tp, V_TypeTraits<_Tp>::nlanes128> c;
    for (int i = 0; i < V_TypeTraits<_Tp>::nlanes128; i++)
        c.s[i] = tab[idx[i / 4] + i % 4];
    return c;
}

inline v_reg<float, V_TypeTraits<float>::nlanes128>
v_load_expand(const float16_t* ptr)
{
    v_reg<float, V_TypeTraits<float>::nlanes128> v;
    for (int i = 0; i < v.nlanes; i++)
    {
        v.s[i] = ptr[i];
    }
    return v;
}*/

template<typename _Tp, int n>
inline void v_store(_Tp* ptr, const v_reg<_Tp, n>& a)
{
#if CV_STRONG_ALIGNMENT
    CV_Assert(isAligned<sizeof(_Tp)>(ptr));
#endif
    for (int i = 0; i < n; i++)
        ptr[i] = a.s[i];
}


/*inline v_float32x4 v_fma(const v_float32x4& a, const v_float32x4& b, const v_float32x4& c)
{
#if CV_FMA3
    return v_float32x4(_mm_fmadd_ps(a.val, b.val, c.val));
#else
    return v_float32x4(_mm_add_ps(_mm_mul_ps(a.val, b.val), c.val));
#endif
}


#define OPENCV_HAL_IMPL_SSE_MISC_FLT_OP(_Tpvec, _Tp, _Tpreg, suffix, absmask_vec) \
inline _Tpvec v_absdiff(const _Tpvec& a, const _Tpvec& b) \
{ \
    _Tpreg absmask = _mm_castsi128_##suffix(absmask_vec); \
    return _Tpvec(_mm_and_##suffix(_mm_sub_##suffix(a.val, b.val), absmask)); \
} \
inline _Tpvec v_magnitude(const _Tpvec& a, const _Tpvec& b) \
{ \
    _Tpvec res = v_fma(a, a, b*b); \
    return _Tpvec(_mm_sqrt_##suffix(res.val)); \
} \
inline _Tpvec v_sqr_magnitude(const _Tpvec& a, const _Tpvec& b) \
{ \
    return v_fma(a, a, b*b); \
} \
inline _Tpvec v_muladd(const _Tpvec& a, const _Tpvec& b, const _Tpvec& c) \
{ \
    return v_fma(a, b, c); \
}

OPENCV_HAL_IMPL_SSE_MISC_FLT_OP(v_float32x4, float, __m128, ps, _mm_set1_epi32((int)0x7fffffff))*/


template<typename _Tp, int n>
inline v_reg<_Tp, n> v_fma(const v_reg<_Tp, n>& a, const v_reg<_Tp, n>& b,
    const v_reg<_Tp, n>& c)
{
    v_reg<_Tp, n> d;
    for (int i = 0; i < n; i++)
        d.s[i] = a.s[i] * b.s[i] + c.s[i];
    return d;
}

/** @brief A synonym for v_fma */
template<typename _Tp, int n>
inline v_reg<_Tp, n> v_muladd(const v_reg<_Tp, n>& a, const v_reg<_Tp, n>& b,
    const v_reg<_Tp, n>& c)
{
    return v_fma(a, b, c);
}


//typedef v_float32x4 v_float32;
/*#define OPENCV_HAL_IMPL_C_INIT_VAL(_Tpvec, _Tp, suffix) \
inline _Tpvec v_setall_##suffix(_Tp val) { return _Tpvec::all(val); }*/

//inline v_float32 vx_setall_f32(float v) { return  v_float32::all(v); }

#define CV_INTRIN_DEFINE_WIDE_INTRIN(typ, vtyp, short_typ, prefix, loadsfx) \
    inline vtyp vx_setall_##short_typ(typ v) { return prefix##_setall_##short_typ(v); } \
inline vtyp vx_##loadsfx(const typ* ptr) { return prefix##_##loadsfx(ptr); } \
inline void vx_store(typ* ptr, const vtyp& v) { return v_store(ptr, v); } 
    /*inline vtyp vx_##loadsfx##_aligned(const typ* ptr) { return prefix##_##loadsfx##_aligned(ptr); } \
    inline vtyp vx_##loadsfx##_low(const typ* ptr) { return prefix##_##loadsfx##_low(ptr); } \
    inline vtyp vx_##loadsfx##_halves(const typ* ptr0, const typ* ptr1) { return prefix##_##loadsfx##_halves(ptr0, ptr1); } \*/
   
    //inline vtyp vx_setzero_##short_typ() { return prefix##_setzero_##short_typ(); } \
    
   /* inline void vx_store_aligned(typ* ptr, const vtyp& v) { return v_store_aligned(ptr, v); } \
    inline vtyp vx_lut(const typ* ptr, const int* idx) { return prefix##_lut(ptr, idx); } \
    inline vtyp vx_lut_pairs(const typ* ptr, const int* idx) { return prefix##_lut_pairs(ptr, idx); }*/ 

/*#define CV_INTRIN_DEFINE_WIDE_LUT_QUAD(typ, vtyp, prefix) \
    inline vtyp vx_lut_quads(const typ* ptr, const int* idx) { return prefix##_lut_quads(ptr, idx); }

#define CV_INTRIN_DEFINE_WIDE_LOAD_EXPAND(typ, wtyp, prefix) \
    inline wtyp vx_load_expand(const typ* ptr) { return prefix##_load_expand(ptr); }*/

/*#define CV_INTRIN_DEFINE_WIDE_LOAD_EXPAND_Q(typ, qtyp, prefix) \
    inline qtyp vx_load_expand_q(const typ* ptr) { return prefix##_load_expand_q(ptr); }*/

/*#define CV_INTRIN_DEFINE_WIDE_INTRIN_WITH_EXPAND(typ, vtyp, short_typ, wtyp, qtyp, prefix, loadsfx) \
    CV_INTRIN_DEFINE_WIDE_INTRIN(typ, vtyp, short_typ, prefix, loadsfx) */
    /*CV_INTRIN_DEFINE_WIDE_LUT_QUAD(typ, vtyp, prefix) \
    CV_INTRIN_DEFINE_WIDE_LOAD_EXPAND(typ, wtyp, prefix) \
    CV_INTRIN_DEFINE_WIDE_LOAD_EXPAND_Q(typ, qtyp, prefix)*/

#define CV_INTRIN_DEFINE_WIDE_INTRIN_ALL_TYPES(prefix) \
 CV_INTRIN_DEFINE_WIDE_INTRIN(float, v_float32, f32, prefix, load) 
   /*CV_INTRIN_DEFINE_WIDE_INTRIN_WITH_EXPAND(uchar, v_uint8, u8, v_uint16, v_uint32, prefix, load) \
    CV_INTRIN_DEFINE_WIDE_INTRIN_WITH_EXPAND(schar, v_int8, s8, v_int16, v_int32, prefix, load) \*/
   
    /*CV_INTRIN_DEFINE_WIDE_INTRIN(ushort, v_uint16, u16, prefix, load) \
    CV_INTRIN_DEFINE_WIDE_LUT_QUAD(ushort, v_uint16, prefix) \
    CV_INTRIN_DEFINE_WIDE_LOAD_EXPAND(ushort, v_uint32, prefix) \
    CV_INTRIN_DEFINE_WIDE_INTRIN(short, v_int16, s16, prefix, load) \
    CV_INTRIN_DEFINE_WIDE_LUT_QUAD(short, v_int16, prefix) \
    CV_INTRIN_DEFINE_WIDE_LOAD_EXPAND(short, v_int32, prefix) \
    CV_INTRIN_DEFINE_WIDE_INTRIN(int, v_int32, s32, prefix, load) \
    CV_INTRIN_DEFINE_WIDE_LUT_QUAD(int, v_int32, prefix) \
    CV_INTRIN_DEFINE_WIDE_LOAD_EXPAND(int, v_int64, prefix) \*/
    
  // CV_INTRIN_DEFINE_WIDE_LUT_QUAD(float, v_float32, prefix) 
    /*CV_INTRIN_DEFINE_WIDE_INTRIN(unsigned, v_uint32, u32, prefix, load) \
    CV_INTRIN_DEFINE_WIDE_LUT_QUAD(unsigned, v_uint32, prefix) \
    CV_INTRIN_DEFINE_WIDE_LOAD_EXPAND(unsigned, v_uint64, prefix) \*/
    
    /*CV_INTRIN_DEFINE_WIDE_INTRIN(int64, v_int64, s64, prefix, load) \
    CV_INTRIN_DEFINE_WIDE_INTRIN(uint64, v_uint64, u64, prefix, load) \
    CV_INTRIN_DEFINE_WIDE_LOAD_EXPAND(float16_t, v_float32, prefix)*/


//typedef v_float32x4_ v_float32;

   /*typedef v_uint8x16  v_uint8;
typedef v_int8x16   v_int8;
typedef v_uint16x8  v_uint16;
typedef v_int16x8   v_int16;*/
//typedef v_uint32x4  v_uint32;
//typedef v_int32x4   v_int32;
//typedef v_uint64x2  v_uint64;
//typedef v_int64x2   v_int64;
typedef v_float32x4 v_float32;
    CV_INTRIN_DEFINE_WIDE_INTRIN_ALL_TYPES(v)
      




struct RowVec_32f
{
    RowVec_32f()
    {
#if defined USE_IPP_SEP_FILTERS
        bufsz = -1;
#endif
    }

    RowVec_32f(const Mat& _kernel)
    {
        kernel = _kernel;
#if defined USE_IPP_SEP_FILTERS
        bufsz = -1;
#endif
    }

    int operator()(const uchar* _src, uchar* _dst, int width, int cn) const
    {
        //CV_INSTRUMENT_REGION();

#if defined USE_IPP_SEP_FILTERS
        CV_IPP_CHECK()
        {
            int ret = ippiOperator(_src, _dst, width, cn);
            if (ret > 0)
                return ret;
        }
#endif
        int _ksize = kernel.rows + kernel.cols - 1;
        CV_DbgAssert(_ksize > 0);
        const float* src0 = (const float*)_src;
        float* dst = (float*)_dst;
        const float* _kx = kernel.ptr<float>();

        int i = 0, k;
        width *= cn;

#if CV_AVX
        for (; i <= width - 8; i += 8)
        {
            const float* src = src0 + i;
            __m256 f, x0;
            __m256 s0 = _mm256_set1_ps(0.0f);
            for (k = 0; k < _ksize; k++, src += cn)
            {
                f = _mm256_set1_ps(_kx[k]);
                x0 = _mm256_loadu_ps(src);
#if CV_FMA3
                s0 = _mm256_fmadd_ps(x0, f, s0);
#else
                s0 = _mm256_add_ps(s0, _mm256_mul_ps(x0, f));
#endif
            }
            _mm256_storeu_ps(dst + i, s0);
        }
#endif
        v_float32 k0 = vx_setall_f32(_kx[0]);
        for (; i <= width - 4 * v_float32::nlanes; i += 4 * v_float32::nlanes)
        {
            const float* src = src0 + i;
            v_float32 s0 = vx_load(src) * k0;
            v_float32 s1 = vx_load(src + v_float32::nlanes) * k0;
            v_float32 s2 = vx_load(src + 2 * v_float32::nlanes) * k0;
            v_float32 s3 = vx_load(src + 3 * v_float32::nlanes) * k0;
            src += cn;
            for (k = 1; k < _ksize; k++, src += cn)
            {
                v_float32 k1 = vx_setall_f32(_kx[k]);
                s0 = v_muladd(vx_load(src), k1, s0);
                s1 = v_muladd(vx_load(src + v_float32::nlanes), k1, s1);
                s2 = v_muladd(vx_load(src + 2 * v_float32::nlanes), k1, s2);
                s3 = v_muladd(vx_load(src + 3 * v_float32::nlanes), k1, s3);
            }
            v_store(dst + i, s0);
            v_store(dst + i + v_float32::nlanes, s1);
            v_store(dst + i + 2 * v_float32::nlanes, s2);
            v_store(dst + i + 3 * v_float32::nlanes, s3);
        }
        if (i <= width - 2 * v_float32::nlanes)
        {
            const float* src = src0 + i;
            v_float32 s0 = vx_load(src) * k0;
            v_float32 s1 = vx_load(src + v_float32::nlanes) * k0;
            src += cn;
            for (k = 1; k < _ksize; k++, src += cn)
            {
                v_float32 k1 = vx_setall_f32(_kx[k]);
                s0 = v_muladd(vx_load(src), k1, s0);
                s1 = v_muladd(vx_load(src + v_float32::nlanes), k1, s1);
            }
            v_store(dst + i, s0);
            v_store(dst + i + v_float32::nlanes, s1);
            i += 2 * v_float32::nlanes;
        }
        if (i <= width - v_float32::nlanes)
        {
            const float* src = src0 + i;
            v_float32 s0 = vx_load(src) * k0;
            src += cn;
            for (k = 1; k < _ksize; k++, src += cn)
                s0 = v_muladd(vx_load(src), vx_setall_f32(_kx[k]), s0);
            v_store(dst + i, s0);
            i += v_float32::nlanes;
        }
        return i;
    }

    Mat kernel;
#if defined USE_IPP_SEP_FILTERS
private:
    mutable int bufsz;
    int ippiOperator(const uchar* _src, uchar* _dst, int width, int cn) const
    {
        CV_INSTRUMENT_REGION_IPP();

        int _ksize = kernel.rows + kernel.cols - 1;
        if ((1 != cn && 3 != cn) || width < _ksize * 8)
            return 0;

        const float* src = (const float*)_src;
        float* dst = (float*)_dst;
        const float* _kx = (const float*)kernel.data;

        IppiSize roisz = { width, 1 };
        if (bufsz < 0)
        {
            if ((cn == 1 && ippiFilterRowBorderPipelineGetBufferSize_32f_C1R(roisz, _ksize, &bufsz) < 0) ||
                (cn == 3 && ippiFilterRowBorderPipelineGetBufferSize_32f_C3R(roisz, _ksize, &bufsz) < 0))
                return 0;
        }
        AutoBuffer<uchar> buf(bufsz + 64);
        uchar* bufptr = alignPtr(buf.data(), 32);
        int step = (int)(width * sizeof(dst[0]) * cn);
        float borderValue[] = { 0.f, 0.f, 0.f };
        // here is the trick. IPP needs border type and extrapolates the row. We did it already.
        // So we pass anchor=0 and ignore the right tail of results since they are incorrect there.
        if ((cn == 1 && CV_INSTRUMENT_FUN_IPP(ippiFilterRowBorderPipeline_32f_C1R, src, step, &dst, roisz, _kx, _ksize, 0,
            ippBorderRepl, borderValue[0], bufptr) < 0) ||
            (cn == 3 && CV_INSTRUMENT_FUN_IPP(ippiFilterRowBorderPipeline_32f_C3R, src, step, &dst, roisz, _kx, _ksize, 0,
                ippBorderRepl, borderValue, bufptr) < 0))
        {
            setIppErrorStatus();
            return 0;
        }
        CV_IMPL_ADD(CV_IMPL_IPP);
        return width - _ksize + 1;
    }
#endif
};

struct SymmColumnVec_32f
{
    SymmColumnVec_32f() {
        symmetryType = 0;
        delta = 0;
    }
    SymmColumnVec_32f(const Mat& _kernel, int _symmetryType, int, double _delta)
    {
        symmetryType = _symmetryType;
        kernel = _kernel;
        delta = (float)_delta;
        CV_Assert((symmetryType & (KERNEL_SYMMETRICAL | KERNEL_ASYMMETRICAL)) != 0);
    }

    int operator()(const uchar** _src, uchar* _dst, int width) const
    {
        //CV_INSTRUMENT_REGION();

        int ksize2 = (kernel.rows + kernel.cols - 1) / 2;
        const float* ky = kernel.ptr<float>() + ksize2;
        int i = 0, k;
        bool symmetrical = (symmetryType & KERNEL_SYMMETRICAL) != 0;
        const float** src = (const float**)_src;
        float* dst = (float*)_dst;

        if (symmetrical)
        {

#if CV_AVX
            {
                const float* S, * S2;
                const __m256 d8 = _mm256_set1_ps(delta);

                for (; i <= width - 16; i += 16)
                {
                    __m256 f = _mm256_set1_ps(ky[0]);
                    __m256 s0, s1;
                    __m256 x0;
                    S = src[0] + i;
                    s0 = _mm256_loadu_ps(S);
#if CV_FMA3
                    s0 = _mm256_fmadd_ps(s0, f, d8);
#else
                    s0 = _mm256_add_ps(_mm256_mul_ps(s0, f), d8);
#endif
                    s1 = _mm256_loadu_ps(S + 8);
#if CV_FMA3
                    s1 = _mm256_fmadd_ps(s1, f, d8);
#else
                    s1 = _mm256_add_ps(_mm256_mul_ps(s1, f), d8);
#endif

                    for (k = 1; k <= ksize2; k++)
                    {
                        S = src[k] + i;
                        S2 = src[-k] + i;
                        f = _mm256_set1_ps(ky[k]);
                        x0 = _mm256_add_ps(_mm256_loadu_ps(S), _mm256_loadu_ps(S2));
#if CV_FMA3
                        s0 = _mm256_fmadd_ps(x0, f, s0);
#else
                        s0 = _mm256_add_ps(s0, _mm256_mul_ps(x0, f));
#endif
                        x0 = _mm256_add_ps(_mm256_loadu_ps(S + 8), _mm256_loadu_ps(S2 + 8));
#if CV_FMA3
                        s1 = _mm256_fmadd_ps(x0, f, s1);
#else
                        s1 = _mm256_add_ps(s1, _mm256_mul_ps(x0, f));
#endif
                    }

                    _mm256_storeu_ps(dst + i, s0);
                    _mm256_storeu_ps(dst + i + 8, s1);
                }
            }
#endif
            const v_float32 d4 = vx_setall_f32(delta);
            const v_float32 k0 = vx_setall_f32(ky[0]);
            for (; i <= width - 4 * v_float32::nlanes; i += 4 * v_float32::nlanes)
            {
                v_float32 s0 = v_muladd(vx_load(src[0] + i), k0, d4);
                v_float32 s1 = v_muladd(vx_load(src[0] + i + v_float32::nlanes), k0, d4);
                v_float32 s2 = v_muladd(vx_load(src[0] + i + 2 * v_float32::nlanes), k0, d4);
                v_float32 s3 = v_muladd(vx_load(src[0] + i + 3 * v_float32::nlanes), k0, d4);
                for (k = 1; k <= ksize2; k++)
                {
                    v_float32 k1 = vx_setall_f32(ky[k]);
                    s0 = v_muladd(vx_load(src[k] + i) + vx_load(src[-k] + i), k1, s0);
                    s1 = v_muladd(vx_load(src[k] + i + v_float32::nlanes) + vx_load(src[-k] + i + v_float32::nlanes), k1, s1);
                    s2 = v_muladd(vx_load(src[k] + i + 2 * v_float32::nlanes) + vx_load(src[-k] + i + 2 * v_float32::nlanes), k1, s2);
                    s3 = v_muladd(vx_load(src[k] + i + 3 * v_float32::nlanes) + vx_load(src[-k] + i + 3 * v_float32::nlanes), k1, s3);
                }
                v_store(dst + i, s0);
                v_store(dst + i + v_float32::nlanes, s1);
                v_store(dst + i + 2 * v_float32::nlanes, s2);
                v_store(dst + i + 3 * v_float32::nlanes, s3);
            }
            if (i <= width - 2 * v_float32::nlanes)
            {
                v_float32 s0 = v_muladd(vx_load(src[0] + i), k0, d4);
                v_float32 s1 = v_muladd(vx_load(src[0] + i + v_float32::nlanes), k0, d4);
                for (k = 1; k <= ksize2; k++)
                {
                    v_float32 k1 = vx_setall_f32(ky[k]);
                    s0 = v_muladd(vx_load(src[k] + i) + vx_load(src[-k] + i), k1, s0);
                    s1 = v_muladd(vx_load(src[k] + i + v_float32::nlanes) + vx_load(src[-k] + i + v_float32::nlanes), k1, s1);
                }
                v_store(dst + i, s0);
                v_store(dst + i + v_float32::nlanes, s1);
                i += 2 * v_float32::nlanes;
            }
            if (i <= width - v_float32::nlanes)
            {
                v_float32 s0 = v_muladd(vx_load(src[0] + i), k0, d4);
                for (k = 1; k <= ksize2; k++)
                    s0 = v_muladd(vx_load(src[k] + i) + vx_load(src[-k] + i), vx_setall_f32(ky[k]), s0);
                v_store(dst + i, s0);
                i += v_float32::nlanes;
            }
        }
        else
        {
            CV_DbgAssert(ksize2 > 0);
#if CV_AVX
            {
                const float* S2;
                const __m256 d8 = _mm256_set1_ps(delta);

                for (; i <= width - 16; i += 16)
                {
                    __m256 f, s0 = d8, s1 = d8;
                    __m256 x0;

                    for (k = 1; k <= ksize2; k++)
                    {
                        const float* S = src[k] + i;
                        S2 = src[-k] + i;
                        f = _mm256_set1_ps(ky[k]);
                        x0 = _mm256_sub_ps(_mm256_loadu_ps(S), _mm256_loadu_ps(S2));
#if CV_FMA3
                        s0 = _mm256_fmadd_ps(x0, f, s0);
#else
                        s0 = _mm256_add_ps(s0, _mm256_mul_ps(x0, f));
#endif
                        x0 = _mm256_sub_ps(_mm256_loadu_ps(S + 8), _mm256_loadu_ps(S2 + 8));
#if CV_FMA3
                        s1 = _mm256_fmadd_ps(x0, f, s1);
#else
                        s1 = _mm256_add_ps(s1, _mm256_mul_ps(x0, f));
#endif
                    }

                    _mm256_storeu_ps(dst + i, s0);
                    _mm256_storeu_ps(dst + i + 8, s1);
                }
            }
#endif
            const v_float32 d4 = vx_setall_f32(delta);
            const v_float32 k1 = vx_setall_f32(ky[1]);
            for (; i <= width - 4 * v_float32::nlanes; i += 4 * v_float32::nlanes)
            {
                v_float32 s0 = v_muladd(vx_load(src[1] + i) - vx_load(src[-1] + i), k1, d4);
                v_float32 s1 = v_muladd(vx_load(src[1] + i + v_float32::nlanes) - vx_load(src[-1] + i + v_float32::nlanes), k1, d4);
                v_float32 s2 = v_muladd(vx_load(src[1] + i + 2 * v_float32::nlanes) - vx_load(src[-1] + i + 2 * v_float32::nlanes), k1, d4);
                v_float32 s3 = v_muladd(vx_load(src[1] + i + 3 * v_float32::nlanes) - vx_load(src[-1] + i + 3 * v_float32::nlanes), k1, d4);
                for (k = 2; k <= ksize2; k++)
                {
                    v_float32 k2 = vx_setall_f32(ky[k]);
                    s0 = v_muladd(vx_load(src[k] + i) - vx_load(src[-k] + i), k2, s0);
                    s1 = v_muladd(vx_load(src[k] + i + v_float32::nlanes) - vx_load(src[-k] + i + v_float32::nlanes), k2, s1);
                    s2 = v_muladd(vx_load(src[k] + i + 2 * v_float32::nlanes) - vx_load(src[-k] + i + 2 * v_float32::nlanes), k2, s2);
                    s3 = v_muladd(vx_load(src[k] + i + 3 * v_float32::nlanes) - vx_load(src[-k] + i + 3 * v_float32::nlanes), k2, s3);
                }
                v_store(dst + i, s0);
                v_store(dst + i + v_float32::nlanes, s1);
                v_store(dst + i + 2 * v_float32::nlanes, s2);
                v_store(dst + i + 3 * v_float32::nlanes, s3);
            }
            if (i <= width - 2 * v_float32::nlanes)
            {
                v_float32 s0 = v_muladd(vx_load(src[1] + i) - vx_load(src[-1] + i), k1, d4);
                v_float32 s1 = v_muladd(vx_load(src[1] + i + v_float32::nlanes) - vx_load(src[-1] + i + v_float32::nlanes), k1, d4);
                for (k = 2; k <= ksize2; k++)
                {
                    v_float32 k2 = vx_setall_f32(ky[k]);
                    s0 = v_muladd(vx_load(src[k] + i) - vx_load(src[-k] + i), k2, s0);
                    s1 = v_muladd(vx_load(src[k] + i + v_float32::nlanes) - vx_load(src[-k] + i + v_float32::nlanes), k2, s1);
                }
                v_store(dst + i, s0);
                v_store(dst + i + v_float32::nlanes, s1);
                i += 2 * v_float32::nlanes;
            }
            if (i <= width - v_float32::nlanes)
            {
                v_float32 s0 = v_muladd(vx_load(src[1] + i) - vx_load(src[-1] + i), k1, d4);
                for (k = 2; k <= ksize2; k++)
                    s0 = v_muladd(vx_load(src[k] + i) - vx_load(src[-k] + i), vx_setall_f32(ky[k]), s0);
                v_store(dst + i, s0);
                i += v_float32::nlanes;
            }
        }
        return i;
    }

    int symmetryType;
    float delta;
    Mat kernel;
};

struct RowNoVec
{
    RowNoVec() {}
    RowNoVec(const Mat&) {}
    int operator()(const uchar*, uchar*, int, int) const { return 0; }
};

struct ColumnNoVec
{
    ColumnNoVec() {}
    ColumnNoVec(const Mat&, int, int, double) {}
    int operator()(const uchar**, uchar*, int) const { return 0; }
};

Ptr<BaseRowFilter> getLinearRowFilter(
    int srcType, int bufType,
    const Mat& kernel, int anchor,
    int symmetryType)
{
    //CV_INSTRUMENT_REGION();

    int sdepth = CV_MAT_DEPTH(srcType), ddepth = CV_MAT_DEPTH(bufType);
    int cn = CV_MAT_CN(srcType);
    CV_Assert(cn == CV_MAT_CN(bufType) &&
        ddepth >= max(sdepth, CV_32S) &&
        kernel.type() == ddepth);
    int ksize = kernel.rows + kernel.cols - 1;

    /*if ((symmetryType & (KERNEL_SYMMETRICAL | KERNEL_ASYMMETRICAL)) != 0 && ksize <= 5)
    {
        if (sdepth == CV_8U && ddepth == CV_32S)
            return makePtr<SymmRowSmallFilter<uchar, int, SymmRowSmallVec_8u32s> >
            (kernel, anchor, symmetryType, SymmRowSmallVec_8u32s(kernel, symmetryType));
         if (sdepth == CV_32F && ddepth == CV_32F)
             return makePtr<SymmRowSmallFilter<float, float, SymmRowSmallVec_32f> >
             (kernel, anchor, symmetryType, SymmRowSmallVec_32f(kernel, symmetryType));
    }*/

    /*if (sdepth == CV_8U && ddepth == CV_32S)
        return makePtr<RowFilter<uchar, int, RowVec_8u32s> >
        (kernel, anchor, RowVec_8u32s(kernel));*/
    if (sdepth == CV_8U && ddepth == CV_32F)
        return makePtr<RowFilter<uchar, float, RowNoVec> >(kernel, anchor);
   /* if (sdepth == CV_8U && ddepth == CV_64F)
        return makePtr<RowFilter<uchar, double, RowNoVec> >(kernel, anchor);
    if (sdepth == CV_16U && ddepth == CV_32F)
        return makePtr<RowFilter<ushort, float, RowNoVec> >(kernel, anchor);
    if (sdepth == CV_16U && ddepth == CV_64F)
        return makePtr<RowFilter<ushort, double, RowNoVec> >(kernel, anchor);*/
    /*if (sdepth == CV_16S && ddepth == CV_32F)
        return makePtr<RowFilter<short, float, RowVec_16s32f> >
        (kernel, anchor, RowVec_16s32f(kernel));*/
   /*if (sdepth == CV_16S && ddepth == CV_64F)
        return makePtr<RowFilter<short, double, RowNoVec> >(kernel, anchor);*/
   /* if (sdepth == CV_32F && ddepth == CV_32F)
        return makePtr<RowFilter<float, float, RowVec_32f> >
        (kernel, anchor, RowVec_32f(kernel)); */
    /*if (sdepth == CV_32F && ddepth == CV_64F)
        return makePtr<RowFilter<float, double, RowNoVec> >(kernel, anchor);
    if (sdepth == CV_64F && ddepth == CV_64F)
        return makePtr<RowFilter<double, double, RowNoVec> >(kernel, anchor);

   // CV_Error_(CV_StsNotImplemented,
    //    ("Unsupported combination of source format (=%d), and buffer format (=%d)",
     //       srcType, bufType));*/
}


Ptr<BaseColumnFilter> getLinearColumnFilter(
    int bufType, int dstType,
    const Mat& kernel, int anchor,
    int symmetryType, double delta,
    int bits)
{
    //CV_INSTRUMENT_REGION();

    int sdepth = CV_MAT_DEPTH(bufType), ddepth = CV_MAT_DEPTH(dstType);
    int cn = CV_MAT_CN(dstType);
    CV_Assert(cn == CV_MAT_CN(bufType) &&
        sdepth >= max(ddepth, CV_32S) &&
        kernel.type() == sdepth);

    /*if (!(symmetryType & (KERNEL_SYMMETRICAL | KERNEL_ASYMMETRICAL)))
    {
        if (ddepth == CV_8U && sdepth == CV_32S)
            return makePtr<ColumnFilter<FixedPtCastEx<int, uchar>, ColumnNoVec> >
            (kernel, anchor, delta, FixedPtCastEx<int, uchar>(bits));
        if (ddepth == CV_8U && sdepth == CV_32F)
            return makePtr<ColumnFilter<Cast<float, uchar>, ColumnNoVec> >(kernel, anchor, delta);
        if (ddepth == CV_8U && sdepth == CV_64F)
            return makePtr<ColumnFilter<Cast<double, uchar>, ColumnNoVec> >(kernel, anchor, delta);
        if (ddepth == CV_16U && sdepth == CV_32F)
            return makePtr<ColumnFilter<Cast<float, ushort>, ColumnNoVec> >(kernel, anchor, delta);
        if (ddepth == CV_16U && sdepth == CV_64F)
            return makePtr<ColumnFilter<Cast<double, ushort>, ColumnNoVec> >(kernel, anchor, delta);
        if (ddepth == CV_16S && sdepth == CV_32F)
            return makePtr<ColumnFilter<Cast<float, short>, ColumnNoVec> >(kernel, anchor, delta);
        if (ddepth == CV_16S && sdepth == CV_64F)
            return makePtr<ColumnFilter<Cast<double, short>, ColumnNoVec> >(kernel, anchor, delta);
        if (ddepth == CV_32F && sdepth == CV_32F)
            return makePtr<ColumnFilter<Cast<float, float>, ColumnNoVec> >(kernel, anchor, delta);
        if (ddepth == CV_64F && sdepth == CV_64F)
            return makePtr<ColumnFilter<Cast<double, double>, ColumnNoVec> >(kernel, anchor, delta);
    }
    else*/
     //{
        int ksize = kernel.rows + kernel.cols - 1;
       /*if (ksize == 3)
        {
            if (ddepth == CV_8U && sdepth == CV_32S)
                return makePtr<SymmColumnSmallFilter<
                FixedPtCastEx<int, uchar>, SymmColumnVec_32s8u> >
                (kernel, anchor, delta, symmetryType, FixedPtCastEx<int, uchar>(bits),
                    SymmColumnVec_32s8u(kernel, symmetryType, bits, delta));
            if (ddepth == CV_16S && sdepth == CV_32S && bits == 0)
                return makePtr<SymmColumnSmallFilter<Cast<int, short>,
                SymmColumnSmallVec_32s16s> >(kernel, anchor, delta, symmetryType,
                    Cast<int, short>(), SymmColumnSmallVec_32s16s(kernel, symmetryType, bits, delta));
            if (ddepth == CV_32F && sdepth == CV_32F)
                return makePtr<SymmColumnSmallFilter<
                Cast<float, float>, SymmColumnSmallVec_32f> >
                (kernel, anchor, delta, symmetryType, Cast<float, float>(),
                    SymmColumnSmallVec_32f(kernel, symmetryType, 0, delta));
        }*/ 
        /*if (ddepth == CV_8U && sdepth == CV_32S)
            return makePtr<SymmColumnFilter<FixedPtCastEx<int, uchar>, SymmColumnVec_32s8u> >
            (kernel, anchor, delta, symmetryType, FixedPtCastEx<int, uchar>(bits),
                SymmColumnVec_32s8u(kernel, symmetryType, bits, delta));*/
        if (ddepth == CV_8U && sdepth == CV_32F)
            return makePtr<SymmColumnFilter<Cast<float, uchar>, ColumnNoVec> >
            (kernel, anchor, delta, symmetryType);
       /* if (ddepth == CV_8U && sdepth == CV_64F)
            return makePtr<SymmColumnFilter<Cast<double, uchar>, ColumnNoVec> >
            (kernel, anchor, delta, symmetryType);
        if (ddepth == CV_16U && sdepth == CV_32F)
            return makePtr<SymmColumnFilter<Cast<float, ushort>, ColumnNoVec> >
            (kernel, anchor, delta, symmetryType);
        if (ddepth == CV_16U && sdepth == CV_64F)
            return makePtr<SymmColumnFilter<Cast<double, ushort>, ColumnNoVec> >
            (kernel, anchor, delta, symmetryType);
        if (ddepth == CV_16S && sdepth == CV_32S)
            return makePtr<SymmColumnFilter<Cast<int, short>, ColumnNoVec> >
            (kernel, anchor, delta, symmetryType);
        if (ddepth == CV_16S && sdepth == CV_32F)
            return makePtr<SymmColumnFilter<Cast<float, short>, SymmColumnVec_32f16s> >
            (kernel, anchor, delta, symmetryType, Cast<float, short>(),
                SymmColumnVec_32f16s(kernel, symmetryType, 0, delta));
        if (ddepth == CV_16S && sdepth == CV_64F)
            return makePtr<SymmColumnFilter<Cast<double, short>, ColumnNoVec> >
            (kernel, anchor, delta, symmetryType);*/
       /* if (ddepth == CV_32F && sdepth == CV_32F)
            return makePtr<SymmColumnFilter<Cast<float, float>, SymmColumnVec_32f> >
            (kernel, anchor, delta, symmetryType, Cast<float, float>(),
                SymmColumnVec_32f(kernel, symmetryType, 0, delta));*/
       /*if (ddepth == CV_64F && sdepth == CV_64F)
            return makePtr<SymmColumnFilter<Cast<double, double>, ColumnNoVec> >
            (kernel, anchor, delta, symmetryType);
    }

   /* CV_Error_(CV_StsNotImplemented,
        ("Unsupported combination of buffer format (=%d), and destination format (=%d)",
            bufType, dstType));*/
}




int getKernelType(InputArray filter_kernel, Point anchor)
{
    Mat _kernel = filter_kernel.getMat();
    CV_Assert(_kernel.channels() == 1);
    int i, sz = _kernel.rows * _kernel.cols;

    Mat kernel;
    _kernel.convertTo(kernel, CV_64F);

    const double* coeffs = kernel.ptr<double>();
    double sum = 0;
    int type = KERNEL_SMOOTH + KERNEL_INTEGER;
    if ((_kernel.rows == 1 || _kernel.cols == 1) &&
        anchor.x * 2 + 1 == _kernel.cols &&
        anchor.y * 2 + 1 == _kernel.rows)
        type |= (KERNEL_SYMMETRICAL + KERNEL_ASYMMETRICAL);

    for (i = 0; i < sz; i++)
    {
        double a = coeffs[i], b = coeffs[sz - i - 1];
        if (a != b)
            type &= ~KERNEL_SYMMETRICAL;
        if (a != -b)
            type &= ~KERNEL_ASYMMETRICAL;
        if (a < 0)
            type &= ~KERNEL_SMOOTH;
        if (a != saturate_cast<int>(a))
            type &= ~KERNEL_INTEGER;
        sum += a;
    }

    if (fabs(sum - 1) > FLT_EPSILON * (fabs(sum) + 1))
        type &= ~KERNEL_SMOOTH;
    return type;
}


static bool createBitExactKernel_32S(const Mat& kernel, Mat& kernel_dst, int bits)
{
    kernel.convertTo(kernel_dst, CV_32S, (1 << bits));
    Mat_<double> kernel_64f;
    kernel.convertTo(kernel_64f, CV_64F, (1 << bits));
    int ksize = (int)kernel.total();
    const double eps = 10 * FLT_EPSILON * (1 << bits);
    for (int i = 0; i < ksize; i++)
    {
        int bitExactValue = kernel_dst.at<int>(i);
        double approxValue = kernel_64f.at<double>(i);
        if (fabs(approxValue - bitExactValue) > eps)
            return false;
    }
    return true;
}



#define CV_LOG_WITH_TAG(tag, msgLevel, extra_check0, extra_check1, ...) \
    for(;;) { extra_check0; \
        const auto cv_temp_msglevel = (LogLevel)(msgLevel); \
        if (cv_temp_msglevel >= (CV_LOG_STRIP_LEVEL)) break; \
        auto cv_temp_logtagptr = CV_LOGTAG_PTR_CAST(CV_LOGTAG_EXPAND_NAME(tag)); \
        if (!cv_temp_logtagptr) cv_temp_logtagptr = CV_LOGTAG_PTR_CAST(CV_LOGTAG_FALLBACK); \
        if (!cv_temp_logtagptr) cv_temp_logtagptr = CV_LOGTAG_PTR_CAST(CV_LOGTAG_GLOBAL); \
        if (cv_temp_logtagptr && (cv_temp_msglevel > cv_temp_logtagptr->level)) break; \
        extra_check1; \
        std::stringstream cv_temp_logstream; \
        cv_temp_logstream << __VA_ARGS__; \
        cv::utils::logging::internal::writeLogMessageEx( \
            cv_temp_msglevel, \
            (cv_temp_logtagptr ? cv_temp_logtagptr->name : nullptr), \
            __FILE__, \
            __LINE__, \
            CV_Func, \
            cv_temp_logstream.str().c_str()); \
        break; \
    }

#define CV_LOG_DEBUG(tag, ...) CV_LOG_WITH_TAG(tag, LOG_LEVEL_DEBUG, , , __VA_ARGS__)


Ptr<FilterEngine> createSeparableLinearFilter(int srcType, int dstType,
    InputArray rowKernel, InputArray columnKernel,
    Point anchor = Point(-1, -1), double delta = 0,
    int rowBorderType = BORDER_DEFAULT,
    int columnBorderType = -1,
    const Scalar& borderValue = Scalar());

Ptr<FilterEngine> createSeparableLinearFilter(
    int _srcType, int _dstType,
    InputArray __rowKernel, InputArray __columnKernel,
    Point _anchor, double _delta,
    int _rowBorderType, int _columnBorderType,
    const Scalar& _borderValue)
{
    Mat _rowKernel = __rowKernel.getMat(), _columnKernel = __columnKernel.getMat();
    _srcType = CV_MAT_TYPE(_srcType);
    _dstType = CV_MAT_TYPE(_dstType);
    int sdepth = CV_MAT_DEPTH(_srcType), ddepth = CV_MAT_DEPTH(_dstType);
    int cn = CV_MAT_CN(_srcType);
    CV_Assert(cn == CV_MAT_CN(_dstType));
    int rsize = _rowKernel.rows + _rowKernel.cols - 1;
    int csize = _columnKernel.rows + _columnKernel.cols - 1;
    if (_anchor.x < 0)
        _anchor.x = rsize / 2;
    if (_anchor.y < 0)
        _anchor.y = csize / 2;
    int rtype = getKernelType(_rowKernel,
        _rowKernel.rows == 1 ? Point(_anchor.x, 0) : Point(0, _anchor.x));
    int ctype = getKernelType(_columnKernel,
        _columnKernel.rows == 1 ? Point(_anchor.y, 0) : Point(0, _anchor.y));
    Mat rowKernel, columnKernel;

    bool isBitExactMode = false;
    int bdepth = max(CV_32F, max(sdepth, ddepth));
    int bits = 0;

    if (sdepth == CV_8U &&
        ((rtype == KERNEL_SMOOTH + KERNEL_SYMMETRICAL &&
            ctype == KERNEL_SMOOTH + KERNEL_SYMMETRICAL &&
            ddepth == CV_8U) ||
            ((rtype & (KERNEL_SYMMETRICAL + KERNEL_ASYMMETRICAL)) &&
                (ctype & (KERNEL_SYMMETRICAL + KERNEL_ASYMMETRICAL)) &&
                (rtype & ctype & KERNEL_INTEGER) &&
                ddepth == CV_16S)))
    {
        int bits_ = ddepth == CV_8U ? 8 : 0;
        bool isValidBitExactRowKernel = createBitExactKernel_32S(_rowKernel, rowKernel, bits_);
        bool isValidBitExactColumnKernel = createBitExactKernel_32S(_columnKernel, columnKernel, bits_);
        //cout << "isValidBitExactRowKernel : " << isValidBitExactRowKernel << endl;
        //cout << "isValidBitExactColumnKernel : " << isValidBitExactColumnKernel << endl;
        if (!isValidBitExactRowKernel)
        {
            //CV_LOG_DEBUG(NULL, "createSeparableLinearFilter: bit-exact row-kernel can't be applied: ksize=" << _rowKernel.total());
        }
        else if (!isValidBitExactColumnKernel)
        {
            //CV_LOG_DEBUG(NULL, "createSeparableLinearFilter: bit-exact column-kernel can't be applied: ksize=" << _columnKernel.total());
        }
        else
        {
            bdepth = CV_32S;
            bits = bits_;
            bits *= 2;
            _delta *= (1 << bits);
            isBitExactMode = true;
        }
    }
    if (!isBitExactMode)
    {
        if (_rowKernel.type() != bdepth)
            _rowKernel.convertTo(rowKernel, bdepth);
        else
            rowKernel = _rowKernel;
        if (_columnKernel.type() != bdepth)
            _columnKernel.convertTo(columnKernel, bdepth);
        else
            columnKernel = _columnKernel;
    }
    

    //int s = CV_MAT_DEPTH(CV_MAKETYPE(bdepth, cn)), d = CV_MAT_DEPTH(_dstType);
    //int k = rowKernel.rows + rowKernel.cols - 1;
    //int k = columnKernel.rows + columnKernel.cols - 1;
   // cout<<"??" << (ctype & (KERNEL_SYMMETRICAL | KERNEL_ASYMMETRICAL))  << endl;
    
    //cout << "sdepth : " << s << " ddepth : " << d  << " k : " << k << " ?? : " << (ctype & (KERNEL_SYMMETRICAL | KERNEL_ASYMMETRICAL)) << endl;


    int _bufType = CV_MAKETYPE(bdepth, cn);
    Ptr<BaseRowFilter> _rowFilter = getLinearRowFilter(
        _srcType, _bufType, rowKernel, _anchor.x, rtype);
    Ptr<BaseColumnFilter> _columnFilter = getLinearColumnFilter(
        _bufType, _dstType, columnKernel, _anchor.y, ctype, _delta, bits);

    return Ptr<FilterEngine>(new FilterEngine(Ptr<BaseFilter>(), _rowFilter, _columnFilter,
        _srcType, _dstType, _bufType, _rowBorderType, _columnBorderType, _borderValue));
}


static void ocvSepFilter(int stype, int dtype, int ktype,
    uchar* src_data, size_t src_step, uchar* dst_data, size_t dst_step,
    int width, int height, int full_width, int full_height,
    int offset_x, int offset_y,
    uchar* kernelx_data, int kernelx_len,
    uchar* kernely_data, int kernely_len,
    int anchor_x, int anchor_y, double delta, int borderType)
{
    Mat kernelX(Size(kernelx_len, 1), ktype, kernelx_data);
    Mat kernelY(Size(kernely_len, 1), ktype, kernely_data);
    Ptr<FilterEngine> f = createSeparableLinearFilter(stype, dtype, kernelX, kernelY,
        Point(anchor_x, anchor_y),
        delta, borderType & ~BORDER_ISOLATED);
    Mat src(Size(width, height), stype, src_data, src_step);
    Mat dst(Size(width, height), dtype, dst_data, dst_step);
    f->apply(src, dst, Size(full_width, full_height), Point(offset_x, offset_y));
};


void sepFilter2D_h(int stype, int dtype, int ktype,
    uchar* src_data, size_t src_step, uchar* dst_data, size_t dst_step,
    int width, int height, int full_width, int full_height,
    int offset_x, int offset_y,
    uchar* kernelx_data, int kernelx_len,
    uchar* kernely_data, int kernely_len,
    int anchor_x, int anchor_y, double delta, int borderType)
{

    bool res = replacementSepFilter(stype, dtype, ktype,
        src_data, src_step, dst_data, dst_step,
        width, height, full_width, full_height,
        offset_x, offset_y,
        kernelx_data, kernelx_len,
        kernely_data, kernely_len,
        anchor_x, anchor_y, delta, borderType);
    cout << res << endl;
    if (res)
        return;
    ocvSepFilter(stype, dtype, ktype,
        src_data, src_step, dst_data, dst_step,
        width, height, full_width, full_height,
        offset_x, offset_y,
        kernelx_data, kernelx_len,
        kernely_data, kernely_len,
        anchor_x, anchor_y, delta, borderType);
}



void sepFilter2D__(InputArray _src, OutputArray _dst, int ddepth,
    InputArray _kernelX, InputArray _kernelY, Point anchor,
    double delta, int borderType)
{
    //CV_INSTRUMENT_REGION();

    CV_Assert(!_src.empty());
    CV_Assert(!_kernelX.empty());
    CV_Assert(!_kernelY.empty());

    CV_OCL_RUN(_dst.isUMat() && _src.dims() <= 2 && (size_t)_src.rows() >= _kernelY.total() && (size_t)_src.cols() >= _kernelX.total(),
        ocl_sepFilter2D(_src, _dst, ddepth, _kernelX, _kernelY, anchor, delta, borderType))

        Mat src = _src.getMat(), kernelX = _kernelX.getMat(), kernelY = _kernelY.getMat();

    if (ddepth < 0)
        ddepth = src.depth();

    _dst.create(src.size(), CV_MAKETYPE(ddepth, src.channels()));
    Mat dst = _dst.getMat();

    Point ofs;
    Size wsz(src.cols, src.rows);
    if ((borderType & BORDER_ISOLATED) == 0)
        src.locateROI(wsz, ofs);

    CV_Assert(kernelX.type() == kernelY.type() &&
        (kernelX.cols == 1 || kernelX.rows == 1) &&
        (kernelY.cols == 1 || kernelY.rows == 1));

    Mat contKernelX = kernelX.isContinuous() ? kernelX : kernelX.clone();
    Mat contKernelY = kernelY.isContinuous() ? kernelY : kernelY.clone();

    sepFilter2D_h(src.type(), dst.type(), kernelX.type(),
        src.data, src.step, dst.data, dst.step,
        dst.cols, dst.rows, wsz.width, wsz.height, ofs.x, ofs.y,
        contKernelX.data, kernelX.cols + kernelX.rows - 1,
        contKernelY.data, kernelY.cols + kernelY.rows - 1,
        anchor.x, anchor.y, delta, borderType & ~BORDER_ISOLATED);
}
