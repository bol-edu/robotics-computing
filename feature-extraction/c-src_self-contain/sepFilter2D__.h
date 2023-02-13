#pragma once
//#include <opencv2/opencv.hpp>
#include "Point__.h"
#include "Size.h"
#include "define.h"
#include <algorithm>
//#include "v_float32x4.h"

using namespace std;

//#define FLT_EPSILON      1.192092896e-07F        // smallest such that 1.0+FLT_EPSILON != 1.0

enum
{
    KERNEL_GENERAL = 0, // the kernel is generic. No any type of symmetry or other properties.
    KERNEL_SYMMETRICAL = 1, // kernel[i] == kernel[ksize-i-1] , and the anchor is at the center
    KERNEL_ASYMMETRICAL = 2, // kernel[i] == -kernel[ksize-i-1] , and the anchor is at the center
    KERNEL_SMOOTH = 4, // all the kernel elements are non-negative and summed to 1
    KERNEL_INTEGER = 8  // all the kernel coefficients are integer numbers
};


/*
#define CV_HAL_ERROR_OK 0
#define CV_HAL_ERROR_NOT_IMPLEMENTED 1
#define CV_HAL_ERROR_UNKNOWN -1


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
}*/



#define CV_ELEM_SIZE1(type) ((0x28442211 >> CV_MAT_DEPTH(type)*4) & 15)

#define CV_ELEM_SIZE(type) (CV_MAT_CN(type)*CV_ELEM_SIZE1(type))

static inline size_t getElemSize_(int type) { return (size_t)CV_ELEM_SIZE(type); }



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
    Point__ anchor;
};

BaseFilter::BaseFilter() { ksize = Size(-1, -1); anchor = Point__(-1, -1); }
BaseFilter::~BaseFilter() {}
void BaseFilter::reset() {}


class BaseRowFilter
{
public:
    //! the default constructor
    BaseRowFilter();
    BaseRowFilter(const Mat& _kernel, int _anchor);
    //! the destructor
    virtual ~BaseRowFilter();
    //! the filtering operator. Must be overridden in the derived classes. The horizontal border interpolation is done outside of the class.
    void operator()(const uchar* src, uchar* dst, int width, int cn) ;

    int ksize;
    int anchor;
    Mat kernel;
};

BaseRowFilter::BaseRowFilter() { ksize = anchor = -1; }
BaseRowFilter::BaseRowFilter(const Mat& _kernel, int _anchor)
{
    if (_kernel.isContinuous())
        kernel = _kernel;
    else
        cout << "not Continuous" << endl;// _kernel.copyTo(kernel);
    anchor = _anchor;
    ksize = kernel.rows + kernel.cols - 1;
    
}
BaseRowFilter::~BaseRowFilter() {}

void BaseRowFilter:: operator()(const uchar* src, uchar* dst, int width, int cn)
{
    //CV_INSTRUMENT_REGION();

    int _ksize = ksize;
    const float* kx = kernel.ptr<float>();
    const uchar* S;
    float* D = (float*)dst;
    int i, k;

    i = 0;// vecOp(src, dst, width, cn);
    width *= cn;
//#if CV_ENABLE_UNROLLED
    for (; i <= width - 4; i += 4)
    {
        S = (const uchar*)src + i;
        float f = kx[0];
        float s0 = f * S[0], s1 = f * S[1], s2 = f * S[2], s3 = f * S[3];

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
//#endif
    for (; i < width; i++)
    {
        S = (const uchar*)src + i;
        float s0 = kx[0] * S[0];
        for (k = 1; k < _ksize; k++)
        {
            S += cn;
            s0 += kx[k] * S[0];
        }
        D[i] = s0;
    }
}

class BaseColumnFilter
{
public:
    //! the default constructor
    BaseColumnFilter();
    BaseColumnFilter(const Mat& _kernel, int _anchor, double _delta, int _symmetryType, const Cast<float, uchar>& _castOp );
    //! the destructor
    virtual ~BaseColumnFilter();
    //! the filtering operator. Must be overridden in the derived classes. The vertical border interpolation is done outside of the class.
     void operator()(const uchar** src, uchar* dst, int dststep, int dstcount, int width) ;
    //! resets the internal buffers, if any
    virtual void reset();

    int ksize;
    int anchor;
    Mat kernel;

    float delta;

    Cast<float, uchar> castOp0;

    int symmetryType;
};

BaseColumnFilter::BaseColumnFilter() { ksize = anchor = -1; }

BaseColumnFilter::BaseColumnFilter(const Mat& _kernel, int _anchor,
    double _delta, int _symmetryType,
    const Cast<float, uchar>& _castOp = Cast<float, uchar>() )
{
    symmetryType = _symmetryType;

    if (_kernel.isContinuous())
        kernel = _kernel;
    else
        cout << "not Continuous" << endl;//_kernel.copyTo(kernel);
    anchor = _anchor;
    ksize = kernel.rows + kernel.cols - 1;
    delta = saturate_cast<float>(_delta);
    castOp0 = _castOp;


}

void BaseColumnFilter::operator()(const uchar** src, uchar* dst, int dststep, int count, int width)
{

    //cout << "using ColumnFilter operator()" << endl;
    int ksize2 = this->ksize / 2;
    const float* ky = this->kernel.template ptr<float>() + ksize2;
    int i, k;
    bool symmetrical = (symmetryType & KERNEL_SYMMETRICAL) != 0;
    float _delta = this->delta;
    Cast<float, uchar> castOp = this->castOp0;
    src += ksize2;

    cout << "symmetrical: " << symmetrical << endl;

    if (symmetrical)
    {
        for (; count--; dst += dststep, src++)
        {
            uchar* D = (uchar*)dst;
            i = 0;// (this->vecOp)(src, dst, width);
#if CV_ENABLE_UNROLLED
            for (; i <= width - 4; i += 4)
            {
                float f = ky[0];
                const float* S = (const float*)src[0] + i, * S2;
                float s0 = f * S[0] + _delta, s1 = f * S[1] + _delta,
                    s2 = f * S[2] + _delta, s3 = f * S[3] + _delta;

                for (k = 1; k <= ksize2; k++)
                {
                    S = (const float*)src[k] + i;
                    S2 = (const float*)src[-k] + i;
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
                float s0 = ky[0] * ((const float*)src[0])[i] + _delta;
                for (k = 1; k <= ksize2; k++)
                    s0 += ky[k] * (((const float*)src[k])[i] + ((const float*)src[-k])[i]);
                D[i] = castOp(s0);
            }
        }
    }
    else
    {
        for (; count--; dst += dststep, src++)
        {
            uchar* D = (uchar*)dst;
            i = 0; // this->vecOp(src, dst, width);
#if CV_ENABLE_UNROLLED
            for (; i <= width - 4; i += 4)
            {
                float f = ky[0];
                const float* S, * S2;
                float s0 = _delta, s1 = _delta, s2 = _delta, s3 = _delta;

                for (k = 1; k <= ksize2; k++)
                {
                    S = (const float*)src[k] + i;
                    S2 = (const float*)src[-k] + i;
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
                float s0 = _delta;
                for (k = 1; k <= ksize2; k++)
                    s0 += ky[k] * (((const float*)src[k])[i] - ((const float*)src[-k])[i]);
                D[i] = castOp(s0);
            }
        }
    }
}


BaseColumnFilter::~BaseColumnFilter() {}
void BaseColumnFilter::reset() {}

class FilterEngine
{
public:
    //! the default constructor
    FilterEngine();
    //! the full constructor. Either _filter2D or both _rowFilter and _columnFilter must be non-empty.
    FilterEngine(//const BaseFilter& _filter2D,
        const BaseRowFilter& _rowFilter,
        const BaseColumnFilter& _columnFilter,
        int srcType, int dstType, int bufType,
        int _rowBorderType = BORDER_REPLICATE,
        int _columnBorderType = -1);
    //! the destructor
    //virtual ~FilterEngine();
    //! reinitializes the engine. The previously assigned filters are released.
    void init(//const Ptr<BaseFilter>& _filter2D,
        const BaseRowFilter& _rowFilter,
        const BaseColumnFilter& _columnFilter,
        int srcType, int dstType, int bufType,
        int _rowBorderType = BORDER_REPLICATE,
        int _columnBorderType = -1);

    //! starts filtering of the specified ROI of an image of size wholeSize.
    /*virtual int start(const cv::Size& wholeSize, const cv::Size& sz, const cv::Point& ofs);
    //! starts filtering of the specified ROI of the specified image.
    virtual int start(const Mat& src, const cv::Size& wsz, const cv::Point& ofs);
    //! processes the next srcCount rows of the image.
    virtual int proceed(const uchar* src, int srcStep, int srcCount,
        uchar* dst, int dstStep);*/
        //! applies filter to the specified ROI of the image. if srcRoi=(0,0,-1,-1), the whole image is filtered.
    virtual void apply(const Mat& src, Mat& dst, const Size& wsz, const Point__& ofs);

    //! returns true if the filter is separable
    bool isSeparable() const { return true; } // return !filter2D;
    //! returns the number
    int remainingInputRows() const;
    int remainingOutputRows() const;

    int srcType;
    int dstType;
    int bufType;
    Size ksize;
    Point__ anchor;
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

    //BaseFilter* filter2D;
    BaseRowFilter* rowFilter;
    BaseColumnFilter* columnFilter;
};

FilterEngine::FilterEngine()
    : srcType(-1), dstType(-1), bufType(-1), maxWidth(0), wholeSize(-1, -1), dx1(0), dx2(0),
    rowBorderType(BORDER_REPLICATE_), columnBorderType(BORDER_REPLICATE_),
    borderElemSize(0), bufStep(0), startY(0), startY0(0), endY(0), rowCount(0), dstY(0)
{
}


FilterEngine::FilterEngine(//const BaseFilter& _filter2D,
    const BaseRowFilter& _rowFilter,
    const BaseColumnFilter& _columnFilter,
    int _srcType, int _dstType, int _bufType,
    int _rowBorderType, int _columnBorderType)
    : srcType(-1), dstType(-1), bufType(-1), maxWidth(0), wholeSize(-1, -1), dx1(0), dx2(0),
    rowBorderType(BORDER_REPLICATE_), columnBorderType(BORDER_REPLICATE_),
    borderElemSize(0), bufStep(0), startY(0), startY0(0), endY(0), rowCount(0), dstY(0)
{
    init( _rowFilter, _columnFilter, _srcType, _dstType, _bufType,
        _rowBorderType, _columnBorderType);
}



void FilterEngine::init(//const Ptr<BaseFilter>& _filter2D,
    const BaseRowFilter& rowFilter,
    const BaseColumnFilter& columnFilter,
    int _srcType, int _dstType, int _bufType,
    int _rowBorderType, int _columnBorderType)
{
    _srcType = CV_MAT_TYPE(_srcType);
    _bufType = CV_MAT_TYPE(_bufType);
    _dstType = CV_MAT_TYPE(_dstType);

    srcType = _srcType;
    int srcElemSize = (int)getElemSize_(srcType);
    dstType = _dstType;
    bufType = _bufType;

    //filter2D = _filter2D;
    //rowFilter = _rowFilter;
    //columnFilter = _columnFilter;

    if (_columnBorderType < 0)
        _columnBorderType = _rowBorderType;

    rowBorderType = _rowBorderType;
    columnBorderType = _columnBorderType;

    //CV_Assert(columnBorderType != BORDER_WRAP);

    if (isSeparable())
    {
       // CV_Assert(rowFilter && columnFilter);
        ksize = Size(rowFilter.ksize, columnFilter.ksize);
        anchor = Point__(rowFilter.anchor, columnFilter.anchor);
    }
    else
    {
        //CV_Assert(bufType == srcType);
        //ksize = filter2D->ksize;
        //anchor = filter2D->anchor;
        cout << "In 2D.h need filter2D" << endl;

    }



    borderElemSize = srcElemSize / (CV_MAT_DEPTH(srcType) >= CV_32S ? sizeof(int) : 1);
    int borderLength = max(ksize.width - 1, 1);
    borderTab.resize(borderLength * borderElemSize);

    maxWidth = bufStep = 0;
    constBorderRow.clear();
    //cout << "here!!!!: " << columnBorderType << endl; 4
   

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




#define VEC_ALIGN 64

template<typename _Tp> static inline _Tp* alignPtr_(_Tp* ptr, int n = (int)sizeof(_Tp))
{
    //CV_DbgAssert((n & (n - 1)) == 0); // n is a power of 2
    return (_Tp*)(((size_t)ptr + n - 1) & -n);
}

static inline size_t alignSize_(size_t sz, int n)
{
    //CV_DbgAssert((n & (n - 1)) == 0); // n is a power of 2
    return (sz + n - 1) & -n;
}

int FilterEngine__start(FilterEngine& this_, const Size& _wholeSize, const Size& sz, const Point__& ofs)
{
    //CV_INSTRUMENT_REGION();

    int i, j;

    this_.wholeSize = _wholeSize;
    this_.roi = Rect(ofs, sz);


    int esz = (int)getElemSize_(this_.srcType);
    int bufElemSize = (int)getElemSize_(this_.bufType);
    const uchar* constVal = !this_.constBorderValue.empty() ? &this_.constBorderValue[0] : 0;

    int _maxBufRows = max(this_.ksize.height + 3,
        max(this_.anchor.y,
            this_.ksize.height - this_.anchor.y - 1) * 2 + 1);

    if (this_.maxWidth < this_.roi.width || _maxBufRows != (int)this_.rows.size())
    {
        //cout << " here!!!!!!!" << endl;
        this_.rows.resize(_maxBufRows);
        this_.maxWidth = max(this_.maxWidth, this_.roi.width);
        int cn = CV_MAT_CN(this_.srcType);
        this_.srcRow.resize(esz * (this_.maxWidth + this_.ksize.width - 1));
        

        int maxBufStep = bufElemSize * (int)alignSize_(this_.maxWidth +
            (!this_.isSeparable() ? this_.ksize.width - 1 : 0), VEC_ALIGN);
        this_.ringBuf.resize(maxBufStep * this_.rows.size() + VEC_ALIGN);
    }

    // adjust bufstep so that the used part of the ring buffer stays compact in memory
    this_.bufStep = bufElemSize * (int)alignSize_(this_.roi.width + (!this_.isSeparable() ? this_.ksize.width - 1 : 0), VEC_ALIGN);

    this_.dx1 = max(this_.anchor.x - this_.roi.x, 0);
    this_.dx2 = max(this_.ksize.width - this_.anchor.x - 1 + this_.roi.x + this_.roi.width - this_.wholeSize.width, 0);

    

    this_.rowCount = this_.dstY = 0;
    this_.startY = this_.startY0 = max(this_.roi.y - this_.anchor.y, 0);
    this_.endY = min(this_.roi.y + this_.roi.height + this_.ksize.height - this_.anchor.y - 1, this_.wholeSize.height);

    if (this_.columnFilter)
        this_.columnFilter->reset();
    //if (this_.filter2D)
     //   this_.filter2D->reset();

    return this_.startY;
}


int FilterEngine__proceed(FilterEngine& this_, const uchar* src, int srcstep, int count,
    uchar* dst, int dststep)
{
    //CV_INSTRUMENT_REGION();

   // CV_DbgAssert(this_.wholeSize.width > 0 && this_.wholeSize.height > 0);

    const int* btab = &this_.borderTab[0];
    int esz = (int)getElemSize_(this_.srcType), btab_esz = this_.borderElemSize;
    uchar** brows = &this_.rows[0];
    int bufRows = (int)this_.rows.size();
    int cn = CV_MAT_CN(this_.bufType);
    int width = this_.roi.width, kwidth = this_.ksize.width;
    int kheight = this_.ksize.height, ay = this_.anchor.y;
    int _dx1 = this_.dx1, _dx2 = this_.dx2;
    int width1 = this_.roi.width + kwidth - 1;
    int xofs1 = min(this_.roi.x, this_.anchor.x);
    bool isSep = this_.isSeparable();
    bool makeBorder = (_dx1 > 0 || _dx2 > 0) && this_.rowBorderType != BORDER_CONSTANT_;
    int dy = 0, i = 0;

    src -= xofs1 * esz;
    count = min(count, this_.remainingInputRows());

   // CV_Assert(src && dst && count > 0);

    for (;; dst += dststep * i, dy += i)
    {
        int dcount = bufRows - ay - this_.startY - this_.rowCount + this_.roi.y;
        dcount = dcount > 0 ? dcount : bufRows - kheight + 1;
        dcount = min(dcount, count);
        count -= dcount;
        for (; dcount-- > 0; src += srcstep)
        {
            int bi = (this_.startY - this_.startY0 + this_.rowCount) % bufRows;
            uchar* brow = alignPtr_(&this_.ringBuf[0], VEC_ALIGN) + bi * this_.bufStep;
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
            int srcY = borderInterpolate_(this_.dstY + dy + i + this_.roi.y - ay,
                this_.wholeSize.height, this_.columnBorderType);
            if (srcY < 0) // can happen only with constant border type
                brows[i] = alignPtr_(&this_.constBorderRow[0], VEC_ALIGN);
            else
            {
               // CV_Assert(srcY >= this_.startY);
                if (srcY >= this_.startY + this_.rowCount)
                    break;
                int bi = (srcY - this_.startY0) % bufRows;
                brows[i] = alignPtr_(&this_.ringBuf[0], VEC_ALIGN) + bi * this_.bufStep;
            }
        }
        if (i < kheight)
            break;
        i -= kheight - 1;
        if (isSep)
            (*this_.columnFilter)((const uchar**)brows, dst, dststep, i, this_.roi.width * cn);
        else
            cout << "need filter2D()" << endl;// (*this_.filter2D)((const uchar**)brows, dst, dststep, i, this_.roi.width, cn);
    }

    this_.dstY += dy;
    //CV_Assert(this_.dstY <= this_.roi.height);
    return dy;
}

void FilterEngine__apply(FilterEngine& this_, const Mat& src, Mat& dst, const Size& wsz, const Point__& ofs)
{

    Size src_size(src.size[1], src.size[0]);
    FilterEngine__start(this_, wsz, src_size, ofs);
    int y = this_.startY - ofs.y;
    FilterEngine__proceed(this_,
        src.ptr() + y * src.step[0],
        (int)src.step[0],
        this_.endY - this_.startY,
        dst.ptr(),
        (int)dst.step[0]);
}

void FilterEngine::apply(const Mat& src, Mat& dst, const Size& wsz, const Point__& ofs)
{

    FilterEngine__apply(*this, src, dst, wsz, ofs);

}

/*
template<typename ST, typename DT, class VecOp> struct RowFilter : public BaseRowFilter
{
    RowFilter(const Mat& _kernel, int _anchor, const VecOp& _vecOp = VecOp())
    {
        
        if (_kernel.isContinuous())
            kernel = _kernel;
        else
            cout << "not Continuous" << endl;// _kernel.copyTo(kernel);
        anchor = _anchor;
        ksize = kernel.rows + kernel.cols - 1;
       // CV_Assert(kernel.type() == DataType<DT>::type &&
       //     (kernel.rows == 1 || kernel.cols == 1));
        vecOp = _vecOp;
    }

    void operator()(const uchar* src, uchar* dst, int width, int cn) 
    {
        //CV_INSTRUMENT_REGION();
        cout << "RowFilter!!" << endl;
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
};*/
/*
template<class CastOp, class VecOp> struct ColumnFilter : public BaseColumnFilter
{
    typedef typename CastOp::type1 ST; // float
    typedef typename CastOp::rtype DT; // uchar

    ColumnFilter(const Mat& _kernel, int _anchor,
        double _delta, const CastOp& _castOp = CastOp(),
        const VecOp& _vecOp = VecOp())
    {
        if (_kernel.isContinuous())
            kernel = _kernel;
        else
            cout << "not Continuous" << endl;//_kernel.copyTo(kernel);
        anchor = _anchor;
        ksize = kernel.rows + kernel.cols - 1;
        delta = saturate_cast<ST>(_delta);
        castOp0 = _castOp;
        vecOp = _vecOp;
        //CV_Assert(kernel.type() == DataType<ST>::type &&
         //   (kernel.rows == 1 || kernel.cols == 1));
    }

    void operator()(const uchar** src, uchar* dst, int dststep, int count, int width) 
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
    typedef typename CastOp::type1 ST; //f
    typedef typename CastOp::rtype DT; //uc

    SymmColumnFilter(const Mat& _kernel, int _anchor,
        double _delta, int _symmetryType,
        const CastOp& _castOp = CastOp(),
        const VecOp& _vecOp = VecOp())
        : ColumnFilter<CastOp, VecOp>(_kernel, _anchor, _delta, _castOp, _vecOp)
    {
        symmetryType = _symmetryType;
        //CV_Assert((symmetryType & (KERNEL_SYMMETRICAL | KERNEL_ASYMMETRICAL)) != 0);
    }

    void operator()(const uchar** src, uchar* dst, int dststep, int count, int width) 
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


*/




/*
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

BaseRowFilter getLinearRowFilter(
    int srcType, int bufType,
    const Mat& kernel, int anchor,
    int symmetryType)
{


    //int sdepth = CV_MAT_DEPTH(srcType), ddepth = CV_MAT_DEPTH(bufType);
    //int cn = CV_MAT_CN(srcType);

   // int ksize = kernel.rows + kernel.cols - 1;
   
    //if (sdepth == CV_8U && ddepth == CV_32F)
        return RowFilter<uchar, float, RowNoVec> (kernel, anchor);
  
}


BaseColumnFilter getLinearColumnFilter(
    int bufType, int dstType,
    const Mat& kernel, int anchor,
    int symmetryType, double delta,
    int bits)
{
    //CV_INSTRUMENT_REGION();

    int sdepth = CV_MAT_DEPTH(bufType), ddepth = CV_MAT_DEPTH(dstType);
    //int cn = CV_MAT_CN(dstType);

   // cout << "s: " << sdepth << " d: " << ddepth << endl; d:0, s:5

    
     //{
        //int ksize = kernel.rows + kernel.cols - 1;
       
        
        //if (ddepth == CV_8U && sdepth == CV_32F)
            return SymmColumnFilter<Cast<float, uchar>, ColumnNoVec> 
            (kernel, anchor, delta, symmetryType);
       
}*/



/*
int getKernelType(InputArray filter_kernel, Point anchor)
{
    Mat _kernel = filter_kernel.getMat();
    //CV_Assert(_kernel.channels() == 1);
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
}*/



/*

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

#define CV_LOG_DEBUG(tag, ...) CV_LOG_WITH_TAG(tag, LOG_LEVEL_DEBUG, , , __VA_ARGS__)*/


FilterEngine createSeparableLinearFilter(int srcType, int dstType,
    Mat& rowKernel, Mat& columnKernel,
    Point__ anchor = Point__(-1, -1), double delta = 0,
    int rowBorderType = BORDER_DEFAULT_,
    int columnBorderType = -1);

FilterEngine createSeparableLinearFilter(
    int _srcType, int _dstType,
    Mat& _rowKernel, Mat& _columnKernel,
    Point__ _anchor, double _delta,
    int _rowBorderType, int _columnBorderType)
{
    //Mat _rowKernel = __rowKernel.getMat(), _columnKernel = __columnKernel.getMat();
    _srcType = CV_MAT_TYPE(_srcType);
    _dstType = CV_MAT_TYPE(_dstType);
    int sdepth = CV_MAT_DEPTH(_srcType), ddepth = CV_MAT_DEPTH(_dstType);
    int cn = CV_MAT_CN(_srcType);
   // CV_Assert(cn == CV_MAT_CN(_dstType));
    int rsize = _rowKernel.rows + _rowKernel.cols - 1;
    int csize = _columnKernel.rows + _columnKernel.cols - 1;
    if (_anchor.x < 0)
        _anchor.x = rsize / 2;
    if (_anchor.y < 0)
        _anchor.y = csize / 2;
    //int rtype = getKernelType(_rowKernel,
    //    _rowKernel.rows == 1 ? Point__(_anchor.x, 0) : Point(0, _anchor.x));
    //int ctype = getKernelType(_columnKernel,
    //    _columnKernel.rows == 1 ? Point(_anchor.y, 0) : Point(0, _anchor.y));

    int rtype = 5;
    int ctype = 5;

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
        //bool isValidBitExactRowKernel = createBitExactKernel_32S(_rowKernel, rowKernel, bits_);
        //bool isValidBitExactColumnKernel = createBitExactKernel_32S(_columnKernel, columnKernel, bits_);
        bool isValidBitExactRowKernel = 0;
        bool isValidBitExactColumnKernel = 0;

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
            cout << "In sepFilter2D__.h: need _rowKernel.convertTo" << endl;// _rowKernel.convertTo(rowKernel, bdepth);
        else
            rowKernel = _rowKernel;
        if (_columnKernel.type() != bdepth)
            cout << "In sepFilter2D__.h: need _columnKernel.convertTo" << endl;// _columnKernel.convertTo(columnKernel, bdepth);
        else
            columnKernel = _columnKernel;
    }
    

    //int s = CV_MAT_DEPTH(CV_MAKETYPE(bdepth, cn)), d = CV_MAT_DEPTH(_dstType);
    //int k = rowKernel.rows + rowKernel.cols - 1;
    //int k = columnKernel.rows + columnKernel.cols - 1;
   // cout<<"??" << (ctype & (KERNEL_SYMMETRICAL | KERNEL_ASYMMETRICAL))  << endl;
    
    //cout << "sdepth : " << s << " ddepth : " << d  << " k : " << k << " ?? : " << (ctype & (KERNEL_SYMMETRICAL | KERNEL_ASYMMETRICAL)) << endl;


    int _bufType = CV_MAKETYPE(bdepth, cn);
    //BaseRowFilter _rowF;
    BaseRowFilter _rowFilter(rowKernel, _anchor.x); // = getLinearRowFilter(_srcType, _bufType, rowKernel, _anchor.x, rtype);
    BaseColumnFilter _columnFilter(columnKernel, _anchor.y, _delta, ctype);// = getLinearColumnFilter(_bufType, _dstType, columnKernel, _anchor.y, ctype, _delta, bits);
   // cout << "borderType: " << _borderValue << endl; 0000
    //BaseFilter* _baseFilter ;
   // _baseFilter.ksize.height = -1;


    FilterEngine F( _rowFilter, _columnFilter,
        _srcType, _dstType, _bufType, _rowBorderType, _columnBorderType);

    return  F;
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
    FilterEngine f = createSeparableLinearFilter(stype, dtype, kernelX, kernelY,
        Point__(anchor_x, anchor_y),
        delta, borderType & ~BORDER_ISOLATED_);
   // cout << "borderType: " << borderType << endl;
    Mat src(Size(width, height), stype, src_data, src_step);
    Mat dst(Size(width, height), dtype, dst_data, dst_step);
    f.apply(src, dst, Size(full_width, full_height), Point__(offset_x, offset_y));
};


void sepFilter2D_h(int stype, int dtype, int ktype,
    uchar* src_data, size_t src_step, uchar* dst_data, size_t dst_step,
    int width, int height, int full_width, int full_height,
    int offset_x, int offset_y,
    uchar* kernelx_data, int kernelx_len,
    uchar* kernely_data, int kernely_len,
    int anchor_x, int anchor_y, double delta, int borderType)
{

    /*bool res = replacementSepFilter(stype, dtype, ktype,
        src_data, src_step, dst_data, dst_step,
        width, height, full_width, full_height,
        offset_x, offset_y,
        kernelx_data, kernelx_len,
        kernely_data, kernely_len,
        anchor_x, anchor_y, delta, borderType);
    cout << res << endl;
    if (res)
        return;*/
    ocvSepFilter(stype, dtype, ktype,
        src_data, src_step, dst_data, dst_step,
        width, height, full_width, full_height,
        offset_x, offset_y,
        kernelx_data, kernelx_len,
        kernely_data, kernely_len,
        anchor_x, anchor_y, delta, borderType);
}



void sepFilter2D__(Mat& src, Mat& dst, int ddepth,
    Mat& kernelX, Mat& kernelY, Point__ anchor,
    double delta, int borderType)
{
    

        //Mat src = _src.getMat(), kernelX = _kernelX.getMat(), kernelY = _kernelY.getMat();

    if (ddepth < 0)
        ddepth = src.depth();

    //_dst.create(src.size(), CV_MAKETYPE(ddepth, src.channels()));
    //Mat dst = _dst.getMat();

    Point__ ofs;
    Size wsz(src.cols, src.rows);
    if ((borderType & BORDER_ISOLATED_) == 0)
        src.locateROI(wsz, ofs);   //wsz: 1312 1859

   // CV_Assert(kernelX.type() == kernelY.type() &&
   //     (kernelX.cols == 1 || kernelX.rows == 1) &&
   //     (kernelY.cols == 1 || kernelY.rows == 1));

    //Mat contKernelX = kernelX.isContinuous() ? kernelX : kernelX.clone();
    //Mat contKernelY = kernelY.isContinuous() ? kernelY : kernelY.clone();
    Mat contKernelX = kernelX;
    Mat contKernelY = kernelY;


    sepFilter2D_h(src.type(), dst.type(), kernelX.type(),
        src.data, src.step[0], dst.data, dst.step[0],
        dst.cols, dst.rows, wsz.width, wsz.height, ofs.x, ofs.y,
        contKernelX.data, kernelX.cols + kernelX.rows - 1,
        contKernelY.data, kernelY.cols + kernelY.rows - 1,
        anchor.x, anchor.y, delta, borderType & ~BORDER_ISOLATED_);
}
