

#include <iostream>
#include "Methods.h"
#include "KeyPoint.h"
//#include "Mat.h"
#include "resize1.h"
#include "threshold__.h"
#include "copyMakeBorder__.h"
//#include "GaussianBlur__.h"
//#include "KeyPointsFilter__.h"
//#include "AutoBuffer__.h"
#include "RNG__.h"
//#include "BufferArea__.h"
#include "FAST__.h"




using namespace std;
//using namespace cv;


/*
void outmat(Mat extImg)
{
    FILE* fpt_level;
    fopen_s(&fpt_level, "../output_own/matdata_img.txt", "w");

    fprintf_s(fpt_level, "des :\n");
    fprintf_s(fpt_level, "size: %d %d\tflags: %d\tdims: %d\tstep: %lld\t",
        extImg.size[0],
        extImg.size[1],
        extImg.flags,
        extImg.dims,
        extImg.step[0]);
    fprintf_s(fpt_level, "%lld\trows: %d\tcols: %d \n ", extImg.step[1], extImg.rows, extImg.cols);


    int imcol = extImg.cols;
    for (int j = 0; j < extImg.rows; j++)
    {
        for (int k = 0; k < extImg.cols; k++)
        {
            fprintf_s(fpt_level, " ( ");
            fprintf_s(fpt_level, "%d, ", j);
            fprintf_s(fpt_level, "%d ): ", k);
            fprintf_s(fpt_level, " data: %d ", extImg.at<unsigned char>(j, k));
            fprintf_s(fpt_level, " %d ", extImg.data[k + (j * imcol)]);
            //cout << image_left.at<unsigned char>(j, k) << endl;
            /*fprintf_s(fpm, " flags: %f ", kp0.at(v).angle);
            fprintf_s(fpm, " dims: %f ", kp0.at(v).response);
            fprintf_s(fpm, " octave: %d ", kp0.at(v).octave);
            fprintf_s(fpm, " class_id: %d ", kp0.at(v).class_id);
            fprintf_s(fpt_level, "\n");
        }
    }

    fclose(fpt_level);
}

*/

enum
{
    KERNEL_GENERAL = 0, // the kernel is generic. No any type of symmetry or other properties.
    KERNEL_SYMMETRICAL = 1, // kernel[i] == kernel[ksize-i-1] , and the anchor is at the center
    KERNEL_ASYMMETRICAL = 2, // kernel[i] == -kernel[ksize-i-1] , and the anchor is at the center
    KERNEL_SMOOTH = 4, // all the kernel elements are non-negative and summed to 1
    KERNEL_INTEGER = 8  // all the kernel coefficients are integer numbers
};





#define CV_ELEM_SIZE1(type) ((0x28442211 >> CV_MAT_DEPTH(type)*4) & 15)

#define CV_ELEM_SIZE(type) (CV_MAT_CN(type)*CV_ELEM_SIZE1(type))

static inline size_t getElemSize_(int type) { return (size_t)CV_ELEM_SIZE(type); }





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

    //cout << "symmetrical: " << symmetrical << endl;

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
    BaseRowFilter rowFilter;
    BaseColumnFilter columnFilter;
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
    const BaseRowFilter& _rowFilter,
    const BaseColumnFilter& _columnFilter,
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

    rowFilter.kernel = _rowFilter.kernel;
        rowFilter.ksize = _rowFilter.ksize;
        rowFilter.anchor = _rowFilter.anchor;

        columnFilter.kernel = _columnFilter.kernel;
            columnFilter.ksize = _columnFilter.ksize;
            columnFilter.anchor = _columnFilter.anchor;

            columnFilter.delta = _columnFilter.delta;

            columnFilter.castOp0 = _columnFilter.castOp0;

            columnFilter.symmetryType = _columnFilter.symmetryType;



    if (_columnBorderType < 0)
        _columnBorderType = _rowBorderType;

    rowBorderType = _rowBorderType;
    columnBorderType = _columnBorderType;



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


    //cout << "Y: " << this_.startY << endl;
    this_.rowCount = this_.dstY = 0;
    this_.startY = this_.startY0 = max(this_.roi.y - this_.anchor.y, 0);
    this_.endY = min(this_.roi.y + this_.roi.height + this_.ksize.height - this_.anchor.y - 1, this_.wholeSize.height);
    //cout << "Y: " << this_.startY << endl;
    //if (this_.columnFilter)
    //    this_.columnFilter->reset();
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
        //cout << "d: " << dcount << endl;
        dcount = min(dcount, count);
        count -= dcount;
        //cout<<"dcount:"<< dcount<<endl;
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
                (this_.rowFilter)(row, brow, width, CV_MAT_CN(this_.srcType));
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
            (this_.columnFilter)((const uchar**)brows, dst, dststep, i, this_.roi.width * cn);
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


    /*sepFilter2D_h(src.type(), dst.type(), kernelX.type(),
        src.data, src.step[0], dst.data, dst.step[0],
        dst.cols, dst.rows, wsz.width, wsz.height, ofs.x, ofs.y,
        contKernelX.data, kernelX.cols + kernelX.rows - 1,
        contKernelY.data, kernelY.cols + kernelY.rows - 1,
        anchor.x, anchor.y, delta, borderType & ~BORDER_ISOLATED_);
    */

    Mat kernelX1(Size(kernelX.cols + kernelX.rows - 1, 1), kernelX.type(), kernelX.data);
        Mat kernelY1(Size(kernelY.cols + kernelY.rows - 1, 1), kernelX.type(), kernelX.data);

        int _srcType = CV_MAT_TYPE(src.type());
            int _dstType = CV_MAT_TYPE(dst.type());

            int _rowBorderType = (borderType & ~BORDER_ISOLATED_) & ~BORDER_ISOLATED_;
            int _columnBorderType = -1;


            int cn = 1;
            int rsize = kernelX1.rows + kernelX1.cols - 1;
            int csize = kernelY1.rows + kernelY1.cols - 1;
            int _anchorx, _anchory;
            _anchorx = anchor.x;
            _anchory = anchor.y;

            if (anchor.x < 0)
                _anchorx = rsize / 2;
            if (anchor.y < 0)
                _anchory = csize / 2;


            int rtype = 5;
            int ctype = 5;

            Mat rowKernel, columnKernel;

            bool isBitExactMode = false;
            int bdepth = 5; // max(CV_32F, max(sdepth, ddepth));
            int bits = 0;

            int _bufType = CV_MAKETYPE(bdepth, cn);

            BaseRowFilter _rowFilter(kernelX1, _anchorx);
            BaseColumnFilter _columnFilter(kernelY1, _anchory, delta, ctype);
            FilterEngine f(_rowFilter, _columnFilter,
                   _srcType, _dstType, _bufType, _rowBorderType, _columnBorderType);

            f.apply(src, dst, Size(wsz.width, wsz.height), Point__(ofs.x, ofs.y));



}





#define EXPTAB_SCALE 6
#define EXPTAB_MASK  ((1 << EXPTAB_SCALE) - 1)

 void GaussianBlur__(Mat& src, Mat& dst, Size ksize,
    double sigmaX, double sigmaY = 0,
    int borderType = BORDER_DEFAULT_);

static const softdouble EXPPOLY_32F_A0 = float64_t::fromRaw(0x3f83ce0f3e46f431);
static const float64_t exp_max_val(3000 * (1 << EXPTAB_SCALE)); // log10(DBL_MAX) < 3000
static const float64_t exp_prescale = float64_t::fromRaw(0x3ff71547652b82fe) * float64_t(1 << EXPTAB_SCALE);
static const float64_t exp_postscale = float64_t::one() / float64_t(1 << EXPTAB_SCALE);

static const uint64_t expTab[] = {
    0x3ff0000000000000, // 1.000000
    0x3ff02c9a3e778061, // 1.010889
    0x3ff059b0d3158574, // 1.021897
    0x3ff0874518759bc8, // 1.033025
    0x3ff0b5586cf9890f, // 1.044274
    0x3ff0e3ec32d3d1a2, // 1.055645
    0x3ff11301d0125b51, // 1.067140
    0x3ff1429aaea92de0, // 1.078761
    0x3ff172b83c7d517b, // 1.090508
    0x3ff1a35beb6fcb75, // 1.102383
    0x3ff1d4873168b9aa, // 1.114387
    0x3ff2063b88628cd6, // 1.126522
    0x3ff2387a6e756238, // 1.138789
    0x3ff26b4565e27cdd, // 1.151189
    0x3ff29e9df51fdee1, // 1.163725
    0x3ff2d285a6e4030b, // 1.176397
    0x3ff306fe0a31b715, // 1.189207
    0x3ff33c08b26416ff, // 1.202157
    0x3ff371a7373aa9cb, // 1.215247
    0x3ff3a7db34e59ff7, // 1.228481
    0x3ff3dea64c123422, // 1.241858
    0x3ff4160a21f72e2a, // 1.255381
    0x3ff44e086061892d, // 1.269051
    0x3ff486a2b5c13cd0, // 1.282870
    0x3ff4bfdad5362a27, // 1.296840
    0x3ff4f9b2769d2ca7, // 1.310961
    0x3ff5342b569d4f82, // 1.325237
    0x3ff56f4736b527da, // 1.339668
    0x3ff5ab07dd485429, // 1.354256
    0x3ff5e76f15ad2148, // 1.369002
    0x3ff6247eb03a5585, // 1.383910
    0x3ff6623882552225, // 1.398980
    0x3ff6a09e667f3bcd, // 1.414214
    0x3ff6dfb23c651a2f, // 1.429613
    0x3ff71f75e8ec5f74, // 1.445181
    0x3ff75feb564267c9, // 1.460918
    0x3ff7a11473eb0187, // 1.476826
    0x3ff7e2f336cf4e62, // 1.492908
    0x3ff82589994cce13, // 1.509164
    0x3ff868d99b4492ed, // 1.525598
    0x3ff8ace5422aa0db, // 1.542211
    0x3ff8f1ae99157736, // 1.559004
    0x3ff93737b0cdc5e5, // 1.575981
    0x3ff97d829fde4e50, // 1.593142
    0x3ff9c49182a3f090, // 1.610490
    0x3ffa0c667b5de565, // 1.628027
    0x3ffa5503b23e255d, // 1.645755
    0x3ffa9e6b5579fdbf, // 1.663677
    0x3ffae89f995ad3ad, // 1.681793
    0x3ffb33a2b84f15fb, // 1.700106
    0x3ffb7f76f2fb5e47, // 1.718619
    0x3ffbcc1e904bc1d2, // 1.737334
    0x3ffc199bdd85529c, // 1.756252
    0x3ffc67f12e57d14b, // 1.775376
    0x3ffcb720dcef9069, // 1.794709
    0x3ffd072d4a07897c, // 1.814252
    0x3ffd5818dcfba487, // 1.834008
    0x3ffda9e603db3285, // 1.853979
    0x3ffdfc97337b9b5f, // 1.874168
    0x3ffe502ee78b3ff6, // 1.894576
    0x3ffea4afa2a490da, // 1.915207
    0x3ffefa1bee615a27, // 1.936062
    0x3fff50765b6e4540, // 1.957144
    0x3fffa7c1819e90d8, // 1.978456
};


static float64_t f64_roundToInt(float64_t a, uint_fast8_t roundingMode, bool exact)
{
    uint_fast64_t uiA;
    int_fast16_t exp;
    uint_fast64_t uiZ, lastBitMask, roundBitsMask;

    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uiA = a.v;
    exp = expF64UI(uiA);
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    if (exp <= 0x3FE) {
        if (!(uiA & UINT64_C(0x7FFFFFFFFFFFFFFF))) return a;
        if (exact)// raiseFlags(flag_inexact);
        uiZ = uiA & packToF64UI(1, 0, 0);
        switch (roundingMode) {
        case round_near_even:
            if (!fracF64UI(uiA)) break;
            /* fallthrough */
        case round_near_maxMag:
            if (exp == 0x3FE) uiZ |= packToF64UI(0, 0x3FF, 0);
            break;
        case round_min:
            if (uiZ) uiZ = packToF64UI(1, 0x3FF, 0);
            break;
        case round_max:
            if (!uiZ) uiZ = packToF64UI(0, 0x3FF, 0);
            break;
        }
        goto uiZ;
    }
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    if (0x433 <= exp) {
        if ((exp == 0x7FF) && fracF64UI(uiA)) {
            uiZ = softfloat_propagateNaNF64UI(uiA, 0);
            goto uiZ;
        }
        return a;
    }
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uiZ = uiA;
    lastBitMask = (uint_fast64_t)1 << (0x433 - exp);
    roundBitsMask = lastBitMask - 1;
    if (roundingMode == round_near_maxMag) {
        uiZ += lastBitMask >> 1;
    }
    else if (roundingMode == round_near_even) {
        uiZ += lastBitMask >> 1;
        if (!(uiZ & roundBitsMask)) uiZ &= ~lastBitMask;
    }
    else if (
        roundingMode
        == (signF64UI(uiZ) ? round_min : round_max)
        ) {
        uiZ += roundBitsMask;
    }
    uiZ &= ~roundBitsMask;
    if (exact && (uiZ != uiA)) {
        //raiseFlags(flag_inexact);
    }
uiZ:
    return float64_t::fromRaw(uiZ);
}


static float64_t f64_exp(float64_t x)
{
    //special cases
    if (x.isNaN()) return float64_t::nan();
    if (x.isInf()) return (x == float64_t::inf()) ? x : float64_t::zero();

    static const float64_t
        A5 = float64_t::one() / EXPPOLY_32F_A0,
        A4 = float64_t::fromRaw(0x3fe62e42fefa39f1) / EXPPOLY_32F_A0, // .69314718055994546743029643825322 / EXPPOLY_32F_A0
        A3 = float64_t::fromRaw(0x3fcebfbdff82a45a) / EXPPOLY_32F_A0, // .24022650695886477918181338054308 / EXPPOLY_32F_A0
        A2 = float64_t::fromRaw(0x3fac6b08d81fec75) / EXPPOLY_32F_A0, // .55504108793649567998466049042729e-1 / EXPPOLY_32F_A0
        A1 = float64_t::fromRaw(0x3f83b2a72b4f3cd3) / EXPPOLY_32F_A0, // .96180973140732918010002372686186e-2 / EXPPOLY_32F_A0
        A0 = float64_t::fromRaw(0x3f55e7aa1566c2a4) / EXPPOLY_32F_A0; // .13369713757180123244806654839424e-2 / EXPPOLY_32F_A0

    float64_t x0;
    if (expF64UI(x.v) > 1023 + 10)
        x0 = signF64UI(x.v) ? -exp_max_val : exp_max_val;
    else
        x0 = x * exp_prescale;

    int val0 = cvRound(x0);
    int t = (val0 >> EXPTAB_SCALE) + 1023;
    t = t < 0 ? 0 : (t > 2047 ? 2047 : t);
    float64_t buf; buf.v = packToF64UI(0, t, 0);

    x0 = (x0 - f64_roundToInt(x0, round_near_even, false)) * exp_postscale;

    return buf * EXPPOLY_32F_A0 * float64_t::fromRaw(expTab[val0 & EXPTAB_MASK]) * (((((A0 * x0 + A1) * x0 + A2) * x0 + A3) * x0 + A4) * x0 + A5);
}

softdouble exp(const softdouble& a) { return f64_exp(a); }



static
softdouble getGaussianKernelBitExact(std::vector<softdouble>& result, int n, double sigma)
{
    //CV_Assert(n > 0);
    //TODO: incorrect SURF implementation requests kernel with n = 20 (PATCH_SZ): https://github.com/opencv/opencv/issues/15856
    //CV_Assert((n & 1) == 1);  // odd

    //cout << "sigma: " << sigma << endl; 2


    softdouble sd_0_15 = softdouble::fromRaw(0x3fc3333333333333);  // 0.15
    softdouble sd_0_35 = softdouble::fromRaw(0x3fd6666666666666);  // 0.35
    softdouble sd_minus_0_125 = softdouble::fromRaw(0xbfc0000000000000);  // -0.5*0.25

    //softdouble sigmaX = sigma > 0 ? softdouble(sigma) : mulAdd(softdouble(n), sd_0_15, sd_0_35);// softdouble(((n-1)*0.5 - 1)*0.3 + 0.8)
    softdouble sigmaX = softdouble(sigma);
    softdouble scale2X = sd_minus_0_125 / (sigmaX * sigmaX);

    int n2_ = (n - 1) / 2;
    AutoBuffer__<softdouble> values(n2_ + 1);
    softdouble sum = softdouble::zero();
    for (int i = 0, x = 1 - n; i < n2_; i++, x += 2)
    {
        // x = i - (n - 1)*0.5
        // t = std::exp(scale2X*x*x)
        softdouble t = exp(softdouble(x * x) * scale2X);
        values[i] = t;
        sum += t;
    }
    sum *= softdouble(2);
    //values[n2_] = softdouble::one(); // x=0 in exp(softdouble(x*x)*scale2X);
    sum += softdouble::one();
    if ((n & 1) == 0)
    {
        //values[n2_ + 1] = softdouble::one();
        sum += softdouble::one();
    }

    // normalize: sum(k[i]) = 1
    softdouble mul1 = softdouble::one() / sum;

    result.resize(n);

    softdouble sum2 = softdouble::zero();
    for (int i = 0; i < n2_; i++)
    {
        softdouble t = values[i] * mul1;
        result[i] = t;
        result[n - 1 - i] = t;
        sum2 += t;
    }
    sum2 *= softdouble(2);
    result[n2_] = /*values[n2_]*/ softdouble::one() * mul1;
    sum2 += result[n2_];
    if ((n & 1) == 0)
    {
        result[n2_ + 1] = result[n2_];
        sum2 += result[n2_];
    }

    return sum2;
}

Mat getGaussianKernel__(int n, double sigma, int ktype)
{
   // CV_CheckDepth(ktype, ktype == CV_32F || ktype == CV_64F, "");
    Mat kernel;// (n, 1, ktype); 7 1 5
    kernel.create_ker();

    std::vector<softdouble> kernel_bitexact;
    getGaussianKernelBitExact(kernel_bitexact, n, sigma);

    if (ktype == CV_32F)
    {
        for (int i = 0; i < n; i++)
            kernel.at<float>(i) = (float)kernel_bitexact[i];
    }
    else
    {
        //CV_DbgAssert(ktype == CV_64F);
        for (int i = 0; i < n; i++)
            kernel.at<double>(i) = kernel_bitexact[i];
    }

    return kernel;
}

static void getGaussianKernel_(int n, double sigma, int ktype, Mat& res) { res = getGaussianKernel__(n, sigma, ktype); }


//template <typename T>
static void createGaussianKernels(Mat& kx, Mat& ky, int type, Size& ksize,
    double sigma1, double sigma2)
{
    int depth = CV_MAT_DEPTH(type);
    if (sigma2 <= 0)
        sigma2 = sigma1;

    // automatic detection of kernel size from sigma
    if (ksize.width <= 0 && sigma1 > 0)
        ksize.width = cvRound(sigma1 * (depth == CV_8U ? 3 : 4) * 2 + 1) | 1;
    if (ksize.height <= 0 && sigma2 > 0)
        ksize.height = cvRound(sigma2 * (depth == CV_8U ? 3 : 4) * 2 + 1) | 1;

    //CV_Assert(ksize.width > 0 && ksize.width % 2 == 1 &&
    //    ksize.height > 0 && ksize.height % 2 == 1);

    sigma1 = max(sigma1, 0.);
    sigma2 = max(sigma2, 0.);

    getGaussianKernel_(ksize.width, sigma1, std::max(depth, CV_32F), kx);
    if (ksize.height == ksize.width && std::abs(sigma1 - sigma2) < DBL_EPSILON)
        ky = kx;
    else
         getGaussianKernel_(ksize.height, sigma2, std::max(depth, CV_32F), ky);
}




void GaussianBlur__(Mat& src, Mat& dst, Size ksize,
    double sigma1, double sigma2,
    int borderType)
{


    int type = src.type();

    Size size(src.size[1], src.size[0]);
   // Size size = _src.size();

   // _dst.create(size, type);



    int sdepth = CV_MAT_DEPTH(type), cn = CV_MAT_CN(type);

    Mat kx, ky;
    createGaussianKernels(kx, ky, type, ksize, sigma1, sigma2);



   // Mat src = _src.getMat();
   // Mat dst = _dst.getMat();

   // Point__ ofs;
   // Size wsz(src.cols, src.rows);
   // if (!(borderType & BORDER_ISOLATED_))
   //    src.locateROI(wsz, ofs);






    sepFilter2D__(src, dst, sdepth, kx, ky, Point__(-1, -1), 0, borderType);

}


static void
computeOrbDescriptors(const Mat& imagePyramid, const std::vector<Rect>& layerInfo,
    const std::vector<float>& layerScale, std::vector<KeyPoint>& keypoints,
    Mat& descriptors, const std::vector<Point__>& _pattern, int dsize, int wta_k)
{
    int step = (int)imagePyramid.step[0];
    int j, i, nkeypoints = (int)keypoints.size();

    for (j = 0; j < nkeypoints; j++)
    {
        const KeyPoint& kpt = keypoints[j];
        const Rect& layer = layerInfo[kpt.octave];
        float scale = 1.f / layerScale[kpt.octave];
        float angle = kpt.angle; //angle = cvFloor(angle/12)*12.f; 

        angle *= (float)(CV_PI / 180.f);
        float a = (float)cos(angle), b = (float)sin(angle);

        const uchar* center = &imagePyramid.at<uchar>(cvRound__(kpt.pt.y * scale) + layer.y,
            cvRound__(kpt.pt.x * scale) + layer.x);
        float x, y;
        int ix, iy;
        const Point__* pattern = &_pattern[0];
        uchar* desc = descriptors.ptr<uchar>(j);

#if 1                         //取旋轉後一個像素點的值  
#define GET_VALUE(idx) (x = pattern[idx].x * a - pattern[idx].y * b, \
            y = pattern[idx].x * b + pattern[idx].y * a, \
            ix = cvRound__(x), \
            iy = cvRound__(y), \
            * (center + iy * step + ix))
#else
        //取旋轉後一個像素點，插值法  
#define GET_VALUE(idx) (x = pattern[idx].x * a - pattern[idx].y * b, \
            y = pattern[idx].x * b + pattern[idx].y * a, \
            ix = cvFloor__(x), iy = cvFloor__(y), \
            x -= ix, y -= iy, \
            cvRound__(center[iy * step + ix] * (1 - x) * (1 - y) + center[(iy + 1) * step + ix] * (1 - x) * y + \
                center[iy * step + ix + 1] * x * (1 - y) + center[(iy + 1) * step + ix + 1] * x * y))
#endif

        if (wta_k == 2)
        {
            for (i = 0; i < dsize; ++i, pattern += 16) //每個特徵描述子長度爲32個字節  
            {
                int t0, t1, val;
                t0 = GET_VALUE(0); t1 = GET_VALUE(1);
                val = t0 < t1;
                t0 = GET_VALUE(2); t1 = GET_VALUE(3);
                val |= (t0 < t1) << 1;
                t0 = GET_VALUE(4); t1 = GET_VALUE(5);
                val |= (t0 < t1) << 2;
                t0 = GET_VALUE(6); t1 = GET_VALUE(7);
                val |= (t0 < t1) << 3;
                t0 = GET_VALUE(8); t1 = GET_VALUE(9);
                val |= (t0 < t1) << 4;
                t0 = GET_VALUE(10); t1 = GET_VALUE(11);
                val |= (t0 < t1) << 5;
                t0 = GET_VALUE(12); t1 = GET_VALUE(13);
                val |= (t0 < t1) << 6;
                t0 = GET_VALUE(14); t1 = GET_VALUE(15);
                val |= (t0 < t1) << 7;

                desc[i] = (uchar)val;
            }
        }

        else
            ;// CV_Error(Error::StsBadSize, "Wrong wta_k. It can be only 2, 3 or 4.");
#undef GET_VALUE
    }
}



static void makeRandomPattern(int patchSize, Point__* pattern, int npoints)
{
    RNG__ rng(0x34985739); // we always start with a fixed seed,
    // to make patterns the same on each run
    for (int i = 0; i < npoints; i++)
    {
        pattern[i].x = rng.uniform(-patchSize / 2, patchSize / 2 + 1);
        pattern[i].y = rng.uniform(-patchSize / 2, patchSize / 2 + 1);
    }
}



static void ICAngles(const Mat& img, const std::vector<Rect>& layerinfo,
    std::vector<KeyPoint>& pts, const std::vector<int>& u_max, int half_k)
{
    int step = (int)img.step1();
    size_t ptidx, ptsize = pts.size();

    for (ptidx = 0; ptidx < ptsize; ptidx++)
    {
        const Rect& layer = layerinfo[pts[ptidx].octave];
        const uchar* center = &img.at<uchar>(cvRound__(pts[ptidx].pt.y) + layer.y, cvRound__(pts[ptidx].pt.x) + layer.x);

        int m_01 = 0, m_10 = 0;

        // Treat the center line differently, v=0
        for (int u = -half_k; u <= half_k; ++u)
            m_10 += u * center[u];

        // Go line by line in the circular patch
        for (int v = 1; v <= half_k; ++v)  //每次處理對稱的兩行v  
        {
            // Proceed over the two lines
            int v_sum = 0;
            int d = u_max[v];
            for (int u = -d; u <= d; ++u)
            {
                int val_plus = center[u + v * step], val_minus = center[u - v * step];
                v_sum += (val_plus - val_minus);  //計算m_01時,位置上差一個符號  
                m_10 += u * (val_plus + val_minus);
            }
            m_01 += v * v_sum;  //計算上下兩行的m_01  
        }

        pts[ptidx].angle = fastAtan2__((float)m_01, (float)m_10);  //計算角度  
    }
}




const float HARRIS_K = 0.04f;

static void
HarrisResponses(const Mat& img, const std::vector<Rect>& layerinfo,
    std::vector<KeyPoint>& pts, int blockSize, float harris_k)
{
    //CV_CheckTypeEQ(img.type(), CV_8UC1, "");
    //CV_CheckGT(blockSize, 0, "");
    //CV_CheckLE(blockSize * blockSize, 2048, "");

    size_t ptidx, ptsize = pts.size();

    const uchar* ptr00 = img.ptr<uchar>();
    size_t size_t_step = img.step[0];
    //CV_CheckLE(size_t_step * blockSize + blockSize + 1, (size_t)INT_MAX, "");  // ofs computation, step+1
    int step = static_cast<int>(size_t_step);

    int r = blockSize / 2;

    float scale = 1.f / ((1 << 2) * blockSize * 255.f);
    float scale_sq_sq = scale * scale * scale * scale;

    AutoBuffer__<int> ofsbuf(blockSize * blockSize);
    int* ofs = ofsbuf.data();
    for (int i = 0; i < blockSize; i++)
        for (int j = 0; j < blockSize; j++)
            ofs[i * blockSize + j] = (int)(i * step + j);

    for (ptidx = 0; ptidx < ptsize; ptidx++)
    {
        int x0 = cvRound__(pts[ptidx].pt.x);
        int y0 = cvRound__(pts[ptidx].pt.y);
        int z = pts[ptidx].octave;

        const uchar* ptr0 = ptr00 + (y0 - r + layerinfo[z].y) * size_t_step + (x0 - r + layerinfo[z].x);
        int a = 0, b = 0, c = 0;

        for (int k = 0; k < blockSize * blockSize; k++)
        {
            const uchar* ptr = ptr0 + ofs[k];
            int Ix = (ptr[1] - ptr[-1]) * 2 + (ptr[-step + 1] - ptr[-step - 1]) + (ptr[step + 1] - ptr[step - 1]);
            int Iy = (ptr[step] - ptr[-step]) * 2 + (ptr[step - 1] - ptr[-step - 1]) + (ptr[step + 1] - ptr[-step + 1]);
            a += Ix * Ix;
            b += Iy * Iy;
            c += Ix * Iy;
        }
        pts[ptidx].response = ((float)a * b - (float)c * c -
            harris_k * ((float)a + b) * ((float)a + b)) * scale_sq_sq;
    }
}

struct KeypointResponseGreaterThanThreshold
{
    KeypointResponseGreaterThanThreshold(float _value) :
        value(_value)
    {
    }
    inline bool operator()(const KeyPoint& kpt) const
    {
        return kpt.response >= value;
    }
    float value;
};

struct KeypointResponseGreater
{
    inline bool operator()(const KeyPoint& kp1, const KeyPoint& kp2) const
    {
        return kp1.response > kp2.response;
    }
};


void retainBest(std::vector<KeyPoint>& keypoints, int n_points)
{
    //this is only necessary if the keypoints size is greater than the number of desired points.
    if (n_points >= 0 && keypoints.size() > (size_t)n_points)
    {
        if (n_points == 0)
        {
            keypoints.clear();
            return;
        }
        //first use nth element to partition the keypoints into the best and worst.
        std::nth_element(keypoints.begin(), keypoints.begin() + n_points - 1, keypoints.end(), KeypointResponseGreater());
        //this is the boundary response, and in the case of FAST may be ambiguous
        float ambiguous_response = keypoints[n_points - 1].response;
        //use std::partition to grab all of the keypoints with the boundary response.
        std::vector<KeyPoint>::const_iterator new_end =
            std::partition(keypoints.begin() + n_points, keypoints.end(),
                KeypointResponseGreaterThanThreshold(ambiguous_response));
        //resize the keypoints, given this new end point. nth_element and partition reordered the points inplace
        keypoints.resize(new_end - keypoints.begin());
    }
}

struct RoiPredicate
{
    RoiPredicate(const Rect& _r) : r(_r)
    {}

    bool operator()(const KeyPoint& keyPt) const
    {
        //return !r.contains(keyPt.pt);
        return!(r.x <= keyPt.pt.x && keyPt.pt.x < r.x + r.width && r.y <= keyPt.pt.y && keyPt.pt.y < r.y + r.height);
    }

    Rect r;
};

void runByImageBorder(std::vector<KeyPoint>& keypoints, Size imageSize, int borderSize)
{
    if (borderSize > 0)
    {
        if (imageSize.height <= borderSize * 2 || imageSize.width <= borderSize * 2)
            keypoints.clear();
        else
            keypoints.erase(std::remove_if(keypoints.begin(), keypoints.end(),
                RoiPredicate(Rect(Point__(borderSize, borderSize),
                    Point__(imageSize.width - borderSize, imageSize.height - borderSize)))),
                keypoints.end());
    }
}


class MaskPredicate
{
public:
    MaskPredicate(const Mat& _mask) : mask(_mask) {}
    bool operator() (const KeyPoint& key_pt) const
    {
        return mask.at<uchar>((int)(key_pt.pt.y + 0.5f), (int)(key_pt.pt.x + 0.5f)) == 0;
    }

private:
    const Mat mask;
    MaskPredicate& operator=(const MaskPredicate&);
};

void detect_fast(Mat& _image, std::vector<KeyPoint>& keypoints, Mat mask)
{


    int threshold = 20;
    bool nonmaxSuppression = true;
    //DetectorType type = TYPE_9_16;

    //cout << "detect" << endl;

    if (_image.empty())
    {
        keypoints.clear();
        return;
    }

    //Mat mask = _mask.getMat(), grayImage;

    Mat gray = _image;

    //outmat(gray);
    FAST__(gray, keypoints, threshold, nonmaxSuppression);
    //KeyPointsFilter__::runByPixelsMask(keypoints, mask);
    if (!mask.empty())
            keypoints.erase(std::remove_if(keypoints.begin(), keypoints.end(), MaskPredicate(mask)), keypoints.end());
}




static void computeKeyPoints(const Mat& imagePyramid,

    const Mat& maskPyramid,
    const std::vector<Rect>& layerInfo,

    const std::vector<float>& layerScale,
    std::vector<KeyPoint>& allKeypoints,
    int nfeatures, double scaleFactor,
    int edgeThreshold, int patchSize, int scoreType,
    int fastThreshold)
{


    int i, nkeypoints, level, nlevels = (int)layerInfo.size(); //金字塔層數  
    std::vector<int> nfeaturesPerLevel(nlevels);

    // fill the extractors and descriptors for the corresponding scales
    float factor = (float)(1.0 / scaleFactor);
    float ndesiredFeaturesPerScale = nfeatures * (1 - factor) / (1 - (float)std::pow((double)factor, (double)nlevels));

    int sumFeatures = 0;
    for (level = 0; level < nlevels - 1; level++) //對每層圖像上分配相應角點數
    {
        nfeaturesPerLevel[level] = cvRound__(ndesiredFeaturesPerScale);
        sumFeatures += nfeaturesPerLevel[level];
        ndesiredFeaturesPerScale *= factor;
    }
    nfeaturesPerLevel[nlevels - 1] = max(nfeatures - sumFeatures, 0); //剩下角點數，由最上層圖像提取  

    // Make sure we forget about what is too close to the boundary
    //edge_threshold_ = std::max(edge_threshold_, patch_size_/2 + kKernelWidth / 2 + 2);

    // pre-compute the end of a row in a circular patch
    int halfPatchSize = patchSize / 2;  //計算每個特徵點圓鄰域的位置信息
    std::vector<int> umax(halfPatchSize + 2);

    int v, v0, vmax = cvFloor__(halfPatchSize * std::sqrt(2.f) / 2 + 1);
    int vmin = cvCeil__(halfPatchSize * std::sqrt(2.f) / 2);
    for (v = 0; v <= vmax; ++v)
        umax[v] = cvRound(std::sqrt((double)halfPatchSize * halfPatchSize - v * v));

    // Make sure we are symmetric
    for (v = halfPatchSize, v0 = 0; v >= vmin; --v)
    {
        while (umax[v0] == umax[v0 + 1])
            ++v0;
        umax[v] = v0;
        ++v0;
    }

    allKeypoints.clear();
    std::vector<KeyPoint> keypoints;
    std::vector<int> counters(nlevels);
    keypoints.reserve(nfeaturesPerLevel[0] * 2);
    //cout << "C: " << nfeaturesPerLevel[0] * 2 << endl;

    for (level = 0; level < nlevels; level++)
    {
        int featuresNum = nfeaturesPerLevel[level];
        Mat img = imagePyramid(layerInfo[level]);
        Mat mask = maskPyramid.empty() ? Mat() : maskPyramid(layerInfo[level]);


        // Detect FAST features, 20 is a good threshold
        {
            //Ptr<FastFeatureDetector__> fd = FastFeatureDetector__::create(fastThreshold, true);
            //fd->detect(img, keypoints, mask); //Fast角點檢測  



            detect_fast(img, keypoints, mask); //Fast角點檢測



        }

        // Remove keypoints very close to the border

        Size img_size(img.size[1], img.size[0]);
        runByImageBorder(keypoints, img_size, edgeThreshold); //去除鄰近邊界的點

        // Keep more points than necessary as FAST does not give amazing corners
        retainBest(keypoints, scoreType == 0 ? 2 * featuresNum : featuresNum);
        //按Fast強度排序,保留前2*featuresNum個特徵點

        nkeypoints = (int)keypoints.size();
        counters[level] = nkeypoints;

        float sf = layerScale[level];
        for (i = 0; i < nkeypoints; i++) // Set the level of the coordinates  
        {
            keypoints[i].octave = level;  //層信息  
            keypoints[i].size = patchSize * sf;
        }

        std::copy(keypoints.begin(), keypoints.end(), std::back_inserter(allKeypoints));

    }

    // std::vector<Vec3i> ukeypoints_buf;

    nkeypoints = (int)allKeypoints.size();
    if (nkeypoints == 0)
    {
        return;
    }
    Mat responses;


    // Select best features using the Harris cornerness (better scoring than FAST)
    //if (scoreType == ORB_Impl__::HARRIS_SCORE)
    //{

    HarrisResponses(imagePyramid, layerInfo, allKeypoints, 7, HARRIS_K);
    //計算每個角點的Harris強度響應  

    std::vector<KeyPoint> newAllKeypoints;
    newAllKeypoints.reserve(nfeaturesPerLevel[0] * nlevels);

    int offset = 0;
    for (level = 0; level < nlevels; level++)
    {
        int featuresNum = nfeaturesPerLevel[level];
        nkeypoints = counters[level];
        keypoints.resize(nkeypoints);
        std::copy(allKeypoints.begin() + offset,
            allKeypoints.begin() + offset + nkeypoints,
            keypoints.begin());
        offset += nkeypoints;

        //cull to the final desired level, using the new Harris scores.
        retainBest(keypoints, featuresNum); //按Harris強度排序，保留前featuresNum個

        std::copy(keypoints.begin(), keypoints.end(), std::back_inserter(newAllKeypoints));
    }
    std::swap(allKeypoints, newAllKeypoints);
    //}

    nkeypoints = (int)allKeypoints.size();


    {   // Process each keypoint  爲每個角點計算主方向，質心法
        ICAngles(imagePyramid, layerInfo, allKeypoints, umax, halfPatchSize);
    }

    for (i = 0; i < nkeypoints; i++)
    {
        float scale = layerScale[allKeypoints[i].octave];
        allKeypoints[i].pt *= scale;
    }
}




static inline size_t alignSize__(size_t sz, int n)
{
    return (sz + n - 1) & -n;
}

static inline float getScale(int level, int firstLevel, double scaleFactor)
{
    return (float)std::pow(scaleFactor, (double)(level - firstLevel));
}



void detectAndCompute(Mat image, Mat mask,
    std::vector<KeyPoint>& keypoints,
     Mat& descriptors, bool useProvidedKeypoints)
{
    enum ScoreType { HARRIS_SCORE = 0 };
    int nfeatures = 500;
    float scaleFactor = 1.2f;
    int nlevels = 8;
    int edgeThreshold = 31;
    int firstLevel = 0;
    int wta_k = 2;
    int scoreType = HARRIS_SCORE;
    int patchSize = 31;
    int fastThreshold = 20;


    bool do_keypoints = !useProvidedKeypoints;
    bool do_descriptors = descriptors.dims == 0 ? false : true;


    if ((!do_keypoints && !do_descriptors) || image.empty())
        return;


    const int HARRIS_BLOCK_SIZE = 9; //Harris角點響應需要的邊界大小  
    int halfPatchSize = patchSize / 2; //鄰域半徑  
    // sqrt(2.0) is for handling patch rotation
    int descPatchSize = cvCeil__(halfPatchSize * sqrt(2.0));
    int border = std::max(edgeThreshold, std::max(descPatchSize, HARRIS_BLOCK_SIZE / 2)) + 1; //採用最大的邊界

    //cout << "border: " << border << endl; 32


    bool useOCL = false;
    //#endif

   // Mat image = _image.getMat(), mask = _mask.getMat();


    int i, nLevels = nlevels, level, nkeypoints = (int)keypoints.size(); //金字塔層數  
    bool sortedByLevel = true;

    if (!do_keypoints) //不做特徵點檢測  
    {

        nLevels = 0;
        for (i = 0; i < nkeypoints; i++)
        {
            level = keypoints[i].octave;

            if (i > 0 && level < keypoints[i - 1].octave)
                sortedByLevel = false;
            nLevels = std::max(nLevels, level);
        }
        nLevels++;
    }

    std::vector<Rect> layerInfo(nLevels);
    std::vector<int> layerOfs(nLevels);
    std::vector<float> layerScale(nLevels);
    Mat imagePyramid, maskPyramid;  //創建尺度金字塔圖像  

    float level0_inv_scale = 1.0f / getScale(0, firstLevel, scaleFactor);       //cout << "level0_inv_scale: " << level0_inv_scale << endl; 1
    size_t level0_width = (size_t)cvRound__(image.cols * level0_inv_scale);     //cout << "level0_width: " << level0_width << endl; 1241
    size_t level0_height = (size_t)cvRound__(image.rows * level0_inv_scale);    //cout << "level0_height: " << level0_height << endl; 376
    Size bufSize((int)alignSize__(level0_width + border * 2, 16), 0);  // TODO change alignment to 64
    //cout << "bufSize: " << bufSize.height << " " << bufSize.width << endl; 0 1312

    int level_dy = (int)level0_height + border * 2;     //cout << "level_dy: " << level_dy << endl; 440
    Point__ level_ofs(0, 0);

    for (level = 0; level < nLevels; level++)
    {
        float scale = getScale(level, firstLevel, scaleFactor); //每層對應的尺度 
        layerScale[level] = scale;
        float inv_scale = 1.0f / scale;
        Size sz(cvRound__(image.cols * inv_scale), cvRound__(image.rows * inv_scale));//每層對應的圖像大小
        Size wholeSize(sz.width + border * 2, sz.height + border * 2);
        if (level_ofs.x + wholeSize.width > bufSize.width)
        {
            level_ofs = Point__(0, level_ofs.y + level_dy);
            level_dy = wholeSize.height;
        }
        //cout << "L: " << level << endl;
        Rect linfo(level_ofs.x + border, level_ofs.y + border, sz.width, sz.height);    // cout << "Rect linfo: " << linfo.x << " " << linfo.y << " " << linfo.width << " " << linfo.height << endl;
        layerInfo[level] = linfo;
        layerOfs[level] = linfo.y * bufSize.width + linfo.x;
        level_ofs.x += wholeSize.width;

    }
    bufSize.height = level_ofs.y + level_dy;

    imagePyramid.create();



    if (!mask.empty())
        maskPyramid.create1();






    Mat prevImg = image, prevMask = mask;
    //outmat(prevImg);


    // Pre-compute the scale pyramids
    for (level = 0; level < nLevels; ++level)
    {



        Rect linfo = layerInfo[level];
        Size sz(linfo.width, linfo.height);
        Size wholeSize(sz.width + border * 2, sz.height + border * 2);
        Rect wholeLinfo = Rect(linfo.x - border, linfo.y - border, wholeSize.width, wholeSize.height);
        Mat extImg = imagePyramid(wholeLinfo), extMask;
        Mat currImg = extImg(Rect(border, border, sz.width, sz.height)), currMask;

        //cout << "L: " << level << endl;
        //cout << "extImg: " << extImg.flags << endl;//112404704
        //cout << "currImg: " << currImg.cols << " " << currImg.rows << " " << currImg.size[0] << endl;
        //Mat prevImg1 = currImg, prevMask1 = currMask;






        if (!mask.empty())
        {
            extMask = maskPyramid(wholeLinfo);
            currMask = extMask(Rect(border, border, sz.width, sz.height));
        }

        // Compute the resized image
        if (level != firstLevel)  //得到金字塔每層的圖像
        {

            resize__(prevImg, currImg, sz, 0, 0, INTER_LINEAR_EXACT);




             //cout << "currImgR : " << currImg.rows << " currImgC : " << currImg.cols << "currImgD : " << currImg.dims << endl;
             //cout << "size : " << *prevImg.size.p << endl;
             //cout << "data : " << *prevImg.data << endl;


            if (!mask.empty())
            {

                resize__(prevMask, currMask, sz, 0, 0, INTER_LINEAR_EXACT);

                if (level > firstLevel)
                    threshold__(currMask, currMask, 254, 0, THRESH_TOZERO);
            }

            copyMakeBorder__(currImg, extImg, border, border, border, border,
                BORDER_REFLECT_101 + BORDER_ISOLATED);
            if (!mask.empty())
                copyMakeBorder__(currMask, extMask, border, border, border, border,
                    BORDER_CONSTANT + BORDER_ISOLATED);




        }
        else
        {

            copyMakeBorder__(image, extImg, border, border, border, border, BORDER_REFLECT_101); //擴大圖像的邊界
            //outmat(extImg);
           // outmat(imagePyramid);


            if (!mask.empty())
                copyMakeBorder__(mask, extMask, border, border, border, border,
                    BORDER_CONSTANT + BORDER_ISOLATED);


        }
        if (level > firstLevel)
        {
            prevImg = currImg;
            prevMask = currMask;

            //prevImg = currImg;
           // prevMask = currMask;



        }


        //Mat prevImg_save = currImg, prevMask_save = currMask;



    }


    //cout << "use: " << useOCL << endl;
    //cout << "layerInfo: " << layerInfo[level].x << " " << layerInfo[level].y << " " << layerInfo[level].height << " " << layerInfo[level].width << endl;



    if (do_keypoints) //提取角點  
    {

        //outmat(imagePyramid);

            // Get keypoints, those will be far enough from the border that no check will be required for the descriptor
        computeKeyPoints(imagePyramid, maskPyramid,
            layerInfo, layerScale, keypoints,
            nfeatures, scaleFactor, edgeThreshold, patchSize, scoreType, fastThreshold);
        //cout << "imagePyramid : " << imagePyramid.rows << " imagePyramidC : " << imagePyramid.cols << "imagePyramidD : " << imagePyramid.dims << endl;
        //cout << "_descriptors : " << *_descriptors.data << endl;
    }
    else
    {
        Size image_size(image.size[1], image.size[0]);
        runByImageBorder(keypoints, image_size, edgeThreshold);

        if (!sortedByLevel)
        {
            std::vector<std::vector<KeyPoint> > allKeypoints(nLevels);
            nkeypoints = (int)keypoints.size();
            for (i = 0; i < nkeypoints; i++)
            {
                level = keypoints[i].octave;
                //CV_Assert(0 <= level);
                allKeypoints[level].push_back(keypoints[i]);
            }
            keypoints.clear();
            for (level = 0; level < nLevels; level++)
                std::copy(allKeypoints[level].begin(), allKeypoints[level].end(), std::back_inserter(keypoints));
        }
    }

    if (do_descriptors)
    {
        //cout << "doing" << endl;
        int dsize = 32; //int dsize = descriptorSize();

        nkeypoints = (int)keypoints.size();
        if (nkeypoints == 0)
        {
            descriptors.release();
            return;
        }

        descriptors.create_des();
        std::vector<Point__> pattern;

        const int npoints = 512;
        Point__ patternbuf[npoints];
        const Point__* pattern0 = (const Point__*)bit_pattern_31_;

        if (patchSize != 31)
        {
            pattern0 = patternbuf;
            makeRandomPattern(patchSize, patternbuf, npoints);
        }


        //cout << "wta_k: " << wta_k << endl; 2
        if (wta_k == 2)
            std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));


        for (level = 0; level < nLevels; level++)
        {
            // preprocess the resized image
            Mat workingMat = imagePyramid(layerInfo[level]);

            //boxFilter(working_mat, working_mat, working_mat.depth(), Size(5,5), Point(-1,-1), true, BORDER_REFLECT_101);
            GaussianBlur__(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);


        }


        //Mat descriptors = _descriptors.getMat();
        computeOrbDescriptors(imagePyramid, layerInfo, layerScale,
            keypoints, descriptors, pattern, dsize, wta_k);


    }




}



void detect(Mat image,
    std::vector<KeyPoint> &keypoints,
    Mat mask)
{

    if (image.empty())
    {
        keypoints.clear();
        return;
    }
    Mat noArray;
    detectAndCompute(image, mask, keypoints, noArray, false);
}

void compute(Mat image,
    std::vector<KeyPoint> &keypoints,
     Mat& descriptors)
{


    if (image.empty())
    {
        descriptors.release();
        return;
    }
    Mat noArray;
    descriptors.dims = 2;

    detectAndCompute(image, noArray, keypoints, descriptors, true);
}



void extract_features(unsigned char* image_data, unsigned char* mask_data, float* kp_xy, unsigned char* des_data)
{


	//Mat des;
	//des.dims = 1;
    Mat image, mask, des;
    vector<KeyPoint> kp;

    image.flags = 1124024320;
    image.dims = 2;
    image.step[0] = 1241;
    image.step[1] = 1;
    image.rows = 376;
    image.cols = 1241;
    //unsigned char ch_image[1241 * 376];
    //for(int i = 0; i < 376*1241; i++)
    	image.data = image_data;
    	//for(int i = 0; i < 10; i++)
    //cout<<"data: "<<static_cast<int>(image_data[i])<<endl;

    mask.flags = 1124024320;
    mask.dims = 2;
    mask.step[0] = 1241;
    mask.step[1] = 1;
    mask.rows = 376;
    mask.cols = 1241;
    //for(int i = 0; i < 376*1241; i++)
    	mask.data = mask_data;



	detect(image, kp, mask);
	compute(image, kp, des);

    for(int o = 0; o < 500 * 32; o++)
            des_data[o] = des.data[o];

    for (int v = 0; v < 500; v++)
    {
        kp_xy[v * 2] = kp.at(v).pt.x;
        kp_xy[v * 2 + 1] = kp.at(v).pt.y;
    }

	//cout << "extract_features" << endl;

    //outmat(des);









}
