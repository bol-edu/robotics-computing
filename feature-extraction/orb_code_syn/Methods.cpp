

#include <iostream>
#include "Methods.h"
//#include "KeyPoint.h"
//#include "Mat.h" //0
#include "resize1.h"
#include "threshold__.h"
#include "copyMakeBorder__.h"
//#include "GaussianBlur__.h" 0
//#include "KeyPointsFilter__.h" 0
//#include "AutoBuffer__.h" 0
//#include "RNG__.h"
//#include "BufferArea__.h" 0
#include "FAST__.h"
#include "KeyPoint.h"




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
            fprintf_s(fpm, " flags: %f ", kp0.at(v).angle);
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
     ~BaseRowFilter();
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
     ~BaseColumnFilter();
    //! the filtering operator. Must be overridden in the derived classes. The vertical border interpolation is done outside of the class.
     void operator()(const uchar** src, uchar* dst, int dststep, int dstcount, int width) ;
    //! resets the internal buffers, if any
     void reset();

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
        const BaseRowFilter _rowFilter,
        const BaseColumnFilter _columnFilter,
        int srcType, int dstType, int bufType,
        int _rowBorderType = BORDER_REPLICATE,
        int _columnBorderType = -1);
    //! the destructor
    //virtual ~FilterEngine();
    //! reinitializes the engine. The previously assigned filters are released.
    void init(//const Ptr<BaseFilter>& _filter2D,
        const BaseRowFilter _rowFilter,
        const BaseColumnFilter _columnFilter,
        int srcType, int dstType, int bufType,
        int _rowBorderType = BORDER_REPLICATE,
        int _columnBorderType = -1);

    //! starts filtering of the specified ROI of an image of size wholeSize.

        //! applies filter to the specified ROI of the image. if srcRoi=(0,0,-1,-1), the whole image is filtered.
     void apply(const Mat& src, Mat& dst, const Size& wsz, const Point__& ofs);

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
    //std::vector<int> borderTab;
    int borderTab[6];
    int borderElemSize;
    //std::vector<uchar> ringBuf;
    uchar ringBuf[51264];
    //std::vector<uchar> srcRow;
    uchar srcRow[1247];
    //std::vector<uchar> constBorderValue;
    //std::vector<uchar> constBorderRow;
    int bufStep;
    int startY;
    int startY0;
    int endY;
    int rowCount;
    int dstY;
    //std::vector<uchar*> rows;
    uchar* rows[10];

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
    const BaseRowFilter _rowFilter,
    const BaseColumnFilter _columnFilter,
    int _srcType, int _dstType, int _bufType,
    int _rowBorderType, int _columnBorderType)
{
	srcType = -1;
	dstType = -1;
	bufType = -1;
	maxWidth = 0;
	wholeSize = Size(-1, -1);
	dx1 = 0;
	dx2 = 0;
	rowBorderType = BORDER_REPLICATE_;
	columnBorderType = BORDER_REPLICATE_;
	borderElemSize = 0;
	bufStep = 0;
	startY = 0;
	startY0 = 0;
	endY = 0;
	rowCount = 0;
	dstY = 0;

    //init( _rowFilter, _columnFilter, _srcType, _dstType, _bufType,
    //    _rowBorderType, _columnBorderType);

	_srcType = CV_MAT_TYPE(_srcType);
	_bufType = CV_MAT_TYPE(_bufType);
	_dstType = CV_MAT_TYPE(_dstType);

	srcType = _srcType;
	int srcElemSize = (int)getElemSize_(srcType);
	dstType = _dstType;
	bufType = _bufType;


	//rowFilter.kernel = _rowFilter.kernel;

	rowFilter.kernel.flags = _rowFilter.kernel.flags;
	rowFilter.kernel.dims = _rowFilter.kernel.dims;
	rowFilter.kernel.rows = _rowFilter.kernel.rows;
	rowFilter.kernel.cols = _rowFilter.kernel.cols;
	rowFilter.kernel.step[0] = _rowFilter.kernel.step[0];
	rowFilter.kernel.step[1] = _rowFilter.kernel.step[1];

	rowFilter.kernel.data = _rowFilter.kernel.data;


	rowFilter.ksize = _rowFilter.ksize;
	rowFilter.anchor = _rowFilter.anchor;



	//columnFilter.kernel = _columnFilter.kernel;

	columnFilter.kernel.flags = _columnFilter.kernel.flags;
	columnFilter.kernel.dims = _columnFilter.kernel.dims;
	columnFilter.kernel.rows = _columnFilter.kernel.rows;
	columnFilter.kernel.cols = _columnFilter.kernel.cols;
	columnFilter.kernel.step[0] = _columnFilter.kernel.step[0];
	columnFilter.kernel.step[1] = _columnFilter.kernel.step[1];

	columnFilter.kernel.data = _columnFilter.kernel.data;



	columnFilter.ksize = _columnFilter.ksize;
	columnFilter.anchor = _columnFilter.anchor;

	columnFilter.delta = _columnFilter.delta;

	columnFilter.castOp0 = _columnFilter.castOp0;

	columnFilter.symmetryType = _columnFilter.symmetryType;



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
	//borderTab.resize(borderLength * borderElemSize);
   // cout << "borderTab: " << borderTab.size() << " " << borderLength * borderElemSize << endl;

	maxWidth = bufStep = 0;
	//constBorderRow.clear();
	//cout << "here!!!!: " << columnBorderType << endl; 4


	wholeSize = Size(-1, -1);


}



void FilterEngine::init(//const Ptr<BaseFilter>& _filter2D,
    const BaseRowFilter _rowFilter,
    const BaseColumnFilter _columnFilter,
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

    //rowFilter.kernel = _rowFilter.kernel;

    rowFilter.kernel.flags = _rowFilter.kernel.flags;
	rowFilter.kernel.dims = _rowFilter.kernel.dims;
	rowFilter.kernel.rows = _rowFilter.kernel.rows;
	rowFilter.kernel.cols = _rowFilter.kernel.cols;
	rowFilter.kernel.step[0] = _rowFilter.kernel.step[0];
	rowFilter.kernel.step[1] = _rowFilter.kernel.step[1];

	rowFilter.kernel.data = _rowFilter.kernel.data;

	rowFilter.ksize = _rowFilter.ksize;
	rowFilter.anchor = _rowFilter.anchor;

	//columnFilter.kernel = _columnFilter.kernel;

	columnFilter.kernel.flags = _columnFilter.kernel.flags;
	columnFilter.kernel.dims = _columnFilter.kernel.dims;
	columnFilter.kernel.rows = _columnFilter.kernel.rows;
	columnFilter.kernel.cols = _columnFilter.kernel.cols;
	columnFilter.kernel.step[0] = _columnFilter.kernel.step[0];
	columnFilter.kernel.step[1] = _columnFilter.kernel.step[1];

	columnFilter.kernel.data = _columnFilter.kernel.data;

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
    //borderTab.resize(borderLength * borderElemSize);

    maxWidth = bufStep = 0;
    //constBorderRow.clear();
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
    //const uchar* constVal = !this_.constBorderValue.empty() ? &this_.constBorderValue[0] : 0;

    int _maxBufRows = max(this_.ksize.height + 3,
        max(this_.anchor.y,
            this_.ksize.height - this_.anchor.y - 1) * 2 + 1);

    if (this_.maxWidth < this_.roi.width /*|| _maxBufRows != (int)this_.rows.size()*/)
    {
        //cout << " here!!!!!!!" << endl;
        //this_.rows.resize(_maxBufRows);
        this_.maxWidth = max(this_.maxWidth, this_.roi.width);
        int cn = CV_MAT_CN(this_.srcType);
        //this_.srcRow.resize(esz * (this_.maxWidth + this_.ksize.width - 1));


       // int maxBufStep = bufElemSize * (int)alignSize_(this_.maxWidth +
        //    (!this_.isSeparable() ? this_.ksize.width - 1 : 0), VEC_ALIGN);
       // this_.ringBuf.resize(maxBufStep * this_.rows.size() + VEC_ALIGN);
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
    int bufRows = 10;//(int)this_.rows.size();
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
            {
				cout << "Error: in sepFilter2D" << endl;
				cout << "srcY < 0" << endl;
				//brows[i] = alignPtr_(&this_.constBorderRow[0], VEC_ALIGN);
			}
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






void sepFilter2D__(Mat src, Mat dst, int ddepth,
    Mat kernelX, Mat kernelY, Point__ anchor,
    double delta, int borderType)
{

	//dst.data[0] = 0;
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

 //   Mat contKernelX = kernelX;
  //  Mat contKernelY = kernelY;


    //sepFilter2D_h(src.type(), dst.type(), kernelX.type(),
    //    src.data, src.step[0], dst.data, dst.step[0],
    //    dst.cols, dst.rows, wsz.width, wsz.height, ofs.x, ofs.y,
    //    contKernelX.data, kernelX.cols + kernelX.rows - 1,
     //   contKernelY.data, kernelY.cols + kernelY.rows - 1,
    //    anchor.x, anchor.y, delta, borderType & ~BORDER_ISOLATED_);


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

           // Mat rowKernel, columnKernel;

            bool isBitExactMode = false;
            int bdepth = 5; // max(CV_32F, max(sdepth, ddepth));
            int bits = 0;

            int _bufType = CV_MAKETYPE(bdepth, cn);

            BaseRowFilter _rowFilter(kernelX1, _anchorx);
            BaseColumnFilter _columnFilter(kernelY1, _anchory, delta, ctype);
            FilterEngine this_(_rowFilter, _columnFilter,
                   _srcType, _dstType, _bufType, _rowBorderType, _columnBorderType);

           //f.apply(src, dst, Size(wsz.width, wsz.height), Point__(ofs.x, ofs.y));

           Size src_size(src.size[1], src.size[0]);
		   //FilterEngine__start(this_, wsz, src_size, ofs);



		   this_.wholeSize = wsz;
		   this_.roi = Rect(ofs, src_size);
		   //cout << "ofs: " << ofs.x << " " << ofs.y << endl;
		  // cout << "sz: " << sz.width << " " << sz.height << endl;


		   int bufElemSize = (int)getElemSize_(this_.bufType);
		   //const uchar* constVal = !this_.constBorderValue.empty() ? &this_.constBorderValue[0] : 0;

		   int _maxBufRows = max(this_.ksize.height + 3,
			   max(this_.anchor.y,
				   this_.ksize.height - this_.anchor.y - 1) * 2 + 1);
		   //_maxBufRows: 10
		  //cout << "_maxBufRows: " << _maxBufRows << " this_.roi.width: " << this_.roi.width << endl;

		   if (this_.maxWidth < this_.roi.width )
		   {

			   this_.maxWidth = max(this_.maxWidth, this_.roi.width);
			   int cn = CV_MAT_CN(this_.srcType);

		   }

		   // adjust bufstep so that the used part of the ring buffer stays compact in memory
		   this_.bufStep = bufElemSize * (int)alignSize_(this_.roi.width + (!this_.isSeparable() ? this_.ksize.width - 1 : 0), VEC_ALIGN);

		   this_.dx1 = max(this_.anchor.x - this_.roi.x, 0);
		   this_.dx2 = max(this_.ksize.width - this_.anchor.x - 1 + this_.roi.x + this_.roi.width - this_.wholeSize.width, 0);



		   this_.rowCount = this_.dstY = 0;
		   this_.startY = this_.startY0 = max(this_.roi.y - this_.anchor.y, 0);
		   this_.endY = min(this_.roi.y + this_.roi.height + this_.ksize.height - this_.anchor.y - 1, this_.wholeSize.height);



		   ////////  FilterEngine__start finish  ///////

		    int y = this_.startY - ofs.y;

		    //outmat(dst);
		    //outdata(dst.data);
		  /*  FilterEngine__proceed(this_,
		        src.ptr() + y * src.step[0],
		        (int)src.step[0],
		        this_.endY - this_.startY,
		        dst.ptr(),
		        (int)dst.step[0]);*/

		    const uchar* srcF = src.ptr() + y * src.step[0];
		    int srcstep = (int)src.step[0];
		    int count = this_.endY - this_.startY;
		    static uchar dstbuf[1859 * 1312];

			uchar* dstF =  dst.ptr();

			/*for (int indx = 0; indx < 1859 * 1312; indx++)
			{
				#pragma HLS PIPELINE
				dstF[indx] = dst.data[indx];

			}*/
			//dst.data[0] = 0;
		    int dststep = (int)dst.step[0];
		    //dstF[0] = 0;

		    //int FilterEngine__proceed(FilterEngine & this_, const uchar * src, int srcstep, int count,
		    //      uchar * dst, int dststep)



		    const int* btab = &this_.borderTab[0];
		    int esz = (int)getElemSize_(this_.srcType), btab_esz = this_.borderElemSize;
		    //uchar** brows = &this_.rows[0];
		    uchar* brows0, *brows1, *brows2, *brows3, *brows4, *brows5, *brows6, *brows7, *brows8, *brows9;
		    int bufRows = 10;//(int)this_.rows.size();
		    int cnF = CV_MAT_CN(this_.bufType);
		    int width = this_.roi.width, kwidth = this_.ksize.width;
		    int kheight = this_.ksize.height, ay = this_.anchor.y;
		    int _dx1 = this_.dx1, _dx2 = this_.dx2;
		    int width1 = this_.roi.width + kwidth - 1;
		    int xofs1 = min(this_.roi.x, this_.anchor.x);
		    bool isSep = this_.isSeparable();
		    bool makeBorder = (_dx1 > 0 || _dx2 > 0) && this_.rowBorderType != BORDER_CONSTANT_;
		    int dy = 0, i = 0;

		    srcF -= xofs1 * esz;
		    count = min(count, this_.remainingInputRows());

		    int bi_[10];
		    float kernel_bitexact[7];
		        kernel_bitexact[0] = 0.0701593;
		        kernel_bitexact[1] = 0.131075;
		        kernel_bitexact[2] = 0.190713;
		        kernel_bitexact[3] = 0.216106;
		        kernel_bitexact[4] = 0.190713;
		        kernel_bitexact[5] = 0.131075;
		        kernel_bitexact[6] = 0.0701593;

		        float kernel_bitexact_col[7];
		        kernel_bitexact_col[0] = 0.216106;
		        kernel_bitexact_col[1] = 0.190713;
		        kernel_bitexact_col[2] = 0.131075;
		        kernel_bitexact_col[3] = 0.0701593;
		        kernel_bitexact_col[4] = 0;
		        kernel_bitexact_col[5] = 0;
		        kernel_bitexact_col[6] = 0;

		    for (;; dstF += dststep * i, dy += i)
		    {
				//#pragma HLS PIPELINE OFF
		        //  cout << "src_data: " << static_cast<int>(src[0]) << " , " << static_cast<int>(src[1]) << endl;
		        //  cout << "data: " << static_cast<int>(dst[0]) << " , " << static_cast<int>(dst[1]) << endl;
		          //cout << "dst: " << static_cast<int>(dst[0]) << " " << static_cast<int>(dst[1]) << endl;
		        int dcount = bufRows - ay - this_.startY - this_.rowCount + this_.roi.y;

		        dcount = dcount > 0 ? dcount : bufRows - kheight + 1;

		        dcount = min(dcount, count);

		        count -= dcount;




		        for (; dcount-- > 0; srcF += srcstep)
		        {
					//#pragma HLS PIPELINE OFF
		            //cout << "here!!!" << endl;
		            int bi = (this_.startY - this_.startY0 + this_.rowCount) % bufRows;

		            //uchar* brow = alignPtr_(&this_.ringBuf[0], VEC_ALIGN) + bi * this_.bufStep;
		            uchar* ptr = &this_.ringBuf[0];
		            uchar* brow = (uchar*)(((size_t)ptr + 64 - 1)  & -64  ) + bi * this_.bufStep;
		            //brow[1] = 0;
		            uchar* row = isSep ? &this_.srcRow[0] : brow;

		            if (++this_.rowCount > bufRows)
		            {
		                --this_.rowCount;
		                ++this_.startY;
		            }

		            memcpy(row + _dx1 * esz, srcF, (width1 - _dx2 - _dx1) * esz);

		           /* if (makeBorder)
		            {
		                if (btab_esz * (int)sizeof(int) == esz)
		                {
		                    const int* isrc = (const int*)srcF;
		                    int* irow = (int*)row;

		                    for (i = 0; i < _dx1 * btab_esz; i++)
		                        irow[i] = isrc[btab[i]];
		                    for (i = 0; i < _dx2 * btab_esz; i++)
		                        irow[i + (width1 - _dx2) * btab_esz] = isrc[btab[i + _dx1 * btab_esz]];
		                }
		                else
		                {
		                    for (i = 0; i < _dx1 * esz; i++)
		                        row[i] = srcF[btab[i]];
		                    for (i = 0; i < _dx2 * esz; i++)
		                        row[i + (width1 - _dx2) * esz] = srcF[btab[i + _dx1 * esz]];
		                }
		            }*/
		            // cout << "is:" << isSep << endl;
		             //cout << "kernel: " << (*this_.rowFilter).kernel.step[0] << endl;

		            if (isSep) //(this_.rowFilter)(row, brow, width, CV_MAT_CN(this_.srcType));
		            {

						int widthR = width;
						int cnR = CV_MAT_CN(this_.srcType);
						//CV_INSTRUMENT_REGION();

						int _ksize = this_.rowFilter.ksize;
						//cout << "kernel.data: " << (float*)(kernel.data )[0] << endl;
						const float* kx = kernel_bitexact;//this_.rowFilter.kernel.ptr<float>();
						//cout << "kernel: " << kernel.step[0] << endl;
						const uchar* S;
						float* D = (float*)brow;
						int i, k;



						i = 0;// vecOp(src, dst, width, cn);
						widthR *= cnR;


						for (; i <= widthR - 4; i += 4)
						{
							#pragma HLS PIPELINE
							S = (const uchar*)row + i;
							float f = kx[0];
							float s0 = f * S[0], s1 = f * S[1], s2 = f * S[2], s3 = f * S[3];

							for (k = 1; k < _ksize; k++)
							{
								S += cnR;
								f = kx[k];
								s0 += f * S[0]; s1 += f * S[1];
								s2 += f * S[2]; s3 += f * S[3];
							}

							D[i] = s0; D[i + 1] = s1;
							D[i + 2] = s2; D[i + 3] = s3;
							//cout << "dst_in: " << static_cast<int>(dst[0]) << endl;
						}


						for (; i < widthR; i++)
						{
							S = (const uchar*)row + i;
							float s0 = kx[0] * S[0];
							for (k = 1; k < _ksize; k++)
							{
								S += cnR;
								s0 += kx[k] * S[0];
							}
							D[i] = s0;
						}

					}

		        }



		        int max_i = min(bufRows, this_.roi.height - (this_.dstY + dy) + (kheight - 1));
		        //cout << "max_i: " << max_i << endl;

		        for (i = 0; i < max_i; i++)
		        {
		            int srcY = borderInterpolate_(this_.dstY + dy + i + this_.roi.y - ay,
		                this_.wholeSize.height, this_.columnBorderType);
		            //cout << "srcY: " << srcY << endl;
		            if (srcY < 0) // can happen only with constant border type
		            {
		                cout << "Error: in sepFilter2D" << endl;
		                cout << "srcY < 0" << endl;
		                //brows[i] = alignPtr_(&this_.constBorderRow[0], VEC_ALIGN);
		            }
		            else
		            {
		                // CV_Assert(srcY >= this_.startY);
		                 //cout << "else: " << endl;
		                if (srcY >= this_.startY + this_.rowCount)
		                    break;
		                int bi = (srcY - this_.startY0) % bufRows;
		                //brows[i] = alignPtr_(&this_.ringBuf[0], VEC_ALIGN) + bi * this_.bufStep;
		                //uchar* ptr = &this_.ringBuf[0];
		                //this_.rows[i] = (uchar*)(ptr)+12 + bi * this_.bufStep;
		                bi_[i] = (srcY - this_.startY0) % bufRows;

		            }



		        }

		        uchar* ptr = &this_.ringBuf[0];

				brows0 = (uchar*)(((size_t)ptr + 64 - 1)  & -64 ) + bi_[0] * this_.bufStep;

				brows1 = (uchar*)(((size_t)ptr + 64 - 1)  & -64 )  + bi_[1] * this_.bufStep;

				brows2 = (uchar*)(((size_t)ptr + 64 - 1)  & -64 ) + bi_[2] * this_.bufStep;

				brows3 = (uchar*)(((size_t)ptr + 64 - 1)  & -64 ) + bi_[3] * this_.bufStep;

				brows4 = (uchar*)(((size_t)ptr + 64 - 1)  & -64 )  + bi_[4] * this_.bufStep;

				brows5 = (uchar*)(((size_t)ptr + 64 - 1)  & -64 ) + bi_[5] * this_.bufStep;

				brows6 = (uchar*)(((size_t)ptr + 64 - 1)  & -64 ) + bi_[6] * this_.bufStep;

				brows7 = (uchar*)(((size_t)ptr + 64 - 1)  & -64 ) + bi_[7] * this_.bufStep;

				brows8 = (uchar*)(((size_t)ptr + 64 - 1)  & -64 ) + bi_[8] * this_.bufStep;

				brows9 = (uchar*)(((size_t)ptr + 64 - 1)  & -64 ) + bi_[9] * this_.bufStep;

		        if (i < kheight)
		            break;
		        i -= kheight - 1;

		        int countC = i;
				int widthC = this_.roi.width * cnF;
				int dststepC = dststep;
				uchar* dstC = dstF;

		        //if (isSep) //(this_.columnFilter)((const uchar**)brows, dstF, dststep, i, this_.roi.width * cnF);
		       // {

					//cout << "using ColumnFilter operator()" << endl;
					int ksize2 = 3; // this_.columnFilter.ksize / 2;
					//cout << "ksize2: " << ksize2 << endl; 3
					const float* ky = kernel_bitexact_col;//this_.columnFilter.kernel.ptr<float>() + ksize2;
					int iC = 0, kC;
					bool symmetrical = (this_.columnFilter.symmetryType & KERNEL_SYMMETRICAL) != 0;
					float _delta = this_.columnFilter.delta;
					Cast<float, uchar> castOp = this_.columnFilter.castOp0;

					int browsNum = 0;
					browsNum += ksize2;

					// cout << "symmetrical: " << symmetrical << endl;
					// cout << "ky: " << static_cast<float>(ky[0]) << endl;
					//float s0 = /*ky[0]*/ 1* ((const float*)brows0)[iC] + _delta; //
					//dstF[iC] = saturate_cast<uchar>(s0);  //

					//if (symmetrical)
					{
						// cout << "dst_first" << static_cast<int>(dst[0]) << endl;

						for (; countC--; dstC += dststepC, browsNum++)
						{
							uchar* D = (uchar*)dstC;
							iC = 0;
	//	#if CV_ENABLE_UNROLLED
							for (; iC <= widthC - 4; iC += 4)
							{
								//#pragma HLS UNROLL factor=16
								float f = ky[0];
								const float* S = (const float*)brows0 + iC, * S2;
								switch (browsNum)
								{
								case 0:
									S = (const float*)brows0 + iC;
									break;
								case 1:
									S = (const float*)brows1 + iC;
									break;
								case 2:
									S = (const float*)brows2 + iC;
									break;
								case 3:
									S = (const float*)brows3 + iC;
									break;
								case 4:
									S = (const float*)brows4 + iC;
									break;
								case 5:
									S = (const float*)brows5 + iC;
									break;
								case 6:
									S = (const float*)brows6 + iC;
									break;
								case 7:
									S = (const float*)brows7 + iC;
									break;
								case 8:
									S = (const float*)brows8 + iC;
									break;
								case 9:
									S = (const float*)brows9 + iC;
									break;
								default:
									cout << "Error: browsNum should <= 9" << endl;
									break;
								}
								float s0 = f * S[0] + _delta, s1 = f * S[1] + _delta,
									s2 = f * S[2] + _delta, s3 = f * S[3] + _delta;


								for (kC = 1; kC <= ksize2; kC++)
								{
									S = (const float*)brows0+ iC;
									S2 = (const float*)brows0 + iC;
									switch (browsNum + kC)
									{
									case 0:
										S = (const float*)brows0 + iC;
										break;
									case 1:
										S = (const float*)brows1 + iC;
										break;
									case 2:
										S = (const float*)brows2 + iC;
										break;
									case 3:
										S = (const float*)brows3 + iC;
										break;
									case 4:
										S = (const float*)brows4 + iC;
										break;
									case 5:
										S = (const float*)brows5 + iC;
										break;
									case 6:
										S = (const float*)brows6 + iC;
										break;
									case 7:
										S = (const float*)brows7 + iC;
										break;
									case 8:
										S = (const float*)brows8 + iC;
										break;
									case 9:
										S = (const float*)brows9 + iC;
										break;
									default:
										cout << "Error: browsNum + KC should <= 9" << endl;
										break;
									}

									switch (browsNum - kC)
									{
									case 0:
										S2 = (const float*)brows0 + iC;
										break;
									case 1:
										S2 = (const float*)brows1 + iC;
										break;
									case 2:
										S2 = (const float*)brows2 + iC;
										break;
									case 3:
										S2 = (const float*)brows3 + iC;
										break;
									case 4:
										S2 = (const float*)brows4 + iC;
										break;
									case 5:
										S2 = (const float*)brows5 + iC;
										break;
									case 6:
										S2 = (const float*)brows6 + iC;
										break;
									case 7:
										S2 = (const float*)brows7 + iC;
										break;
									case 8:
										S2 = (const float*)brows8 + iC;
										break;
									case 9:
										S2 = (const float*)brows9 + iC;
										break;
									default:
										cout << "Error: browsNum - KC should <= 9" << endl;
										break;
									}


									f = ky[kC];
									s0 += f * (S[0] + S2[0]);
									s1 += f * (S[1] + S2[1]);
									s2 += f * (S[2] + S2[2]);
									s3 += f * (S[3] + S2[3]);
								}
								//D[iC] = 0;
								D[iC] = saturate_cast<uchar>(s0); D[iC + 1] = saturate_cast<uchar>(s1);
								D[iC + 2] = saturate_cast<uchar>(s2); D[iC + 3] = saturate_cast<uchar>(s3);
							}
		//#endif

							for (; iC < widthC; iC++)
							{
								float s0 = ky[0] * ((const float*)brows0)[iC] + _delta;
								switch (browsNum)
								{
								case 0:
									s0 = ky[0] * ((const float*)brows0)[iC] + _delta;
									break;
								case 1:
									s0 = ky[0] * ((const float*)brows1)[iC] + _delta;
									break;
								case 2:
									s0 = ky[0] * ((const float*)brows2)[iC] + _delta;
									break;
								case 3:
									s0 = ky[0] * ((const float*)brows3)[iC] + _delta;
									break;
								case 4:
									s0 = ky[0] * ((const float*)brows4)[iC] + _delta;
									break;
								case 5:
									s0 = ky[0] * ((const float*)brows5)[iC] + _delta;
									break;
								case 6:
									s0 = ky[0] * ((const float*)brows6)[iC] + _delta;
									break;
								case 7:
									s0 = ky[0] * ((const float*)brows7)[iC] + _delta;
									break;
								case 8:
									s0 = ky[0] * ((const float*)brows8)[iC] + _delta;
									break;
								case 9:
									s0 = ky[0] * ((const float*)brows9)[iC] + _delta;
									break;
								default:
									cout << "Error: browsNum should <= 9" << endl;
									break;
								}
								for (kC = 1; kC <= ksize2; kC++)
								{
									//#pragma HLS unroll factor = 3
									float tempk, tempk_;
									tempk = ((const float*)brows0)[iC];
									tempk_ = ((const float*)brows0)[iC];
									switch (browsNum + kC)
									{
									case 0:
										tempk = ((const float*)brows0)[iC];
										break;
									case 1:
										tempk = ((const float*)brows1)[iC];
										break;
									case 2:
										tempk = ((const float*)brows2)[iC];
										break;
									case 3:
										tempk = ((const float*)brows3)[iC];
										break;
									case 4:
										tempk = ((const float*)brows4)[iC];
										break;
									case 5:
										tempk = ((const float*)brows5)[iC];
										break;
									case 6:
										tempk = ((const float*)brows6)[iC];
										break;
									case 7:
										tempk = ((const float*)brows7)[iC];
										break;
									case 8:
										tempk = ((const float*)brows8)[iC];
										break;
									case 9:
										tempk = ((const float*)brows9)[iC];
										break;
									default:
										cout << "Error: browsNum should <= 9" << endl;
										break;
									}

									switch (browsNum - kC)
									{
									case 0:
										tempk_ = ((const float*)brows0)[iC];
										break;
									case 1:
										tempk_ = ((const float*)brows1)[iC];
										break;
									case 2:
										tempk_ = ((const float*)brows2)[iC];
										break;
									case 3:
										tempk_ = ((const float*)brows3)[iC];
										break;
									case 4:
										tempk_ = ((const float*)brows4)[iC];
										break;
									case 5:
										tempk_ = ((const float*)brows5)[iC];
										break;
									case 6:
										tempk_ = ((const float*)brows6)[iC];
										break;
									case 7:
										tempk_ = ((const float*)brows7)[iC];
										break;
									case 8:
										tempk_ = ((const float*)brows8)[iC];
										break;
									case 9:
										tempk_ = ((const float*)brows9)[iC];
										break;
									default:
										cout << "Error: browsNum should <= 9" << endl;
										break;
									}

									s0 += ky[kC] * (tempk + tempk_); // (((const float*)browsC[kC])[iC] + ((const float*)browsC[-kC])[iC]);
								}
								D[iC] = saturate_cast<uchar>(s0);
							}


						}



					}
					/*else
					{
						cout << "NOT SYM!!" << endl;


					}*/


				//}
		       // else
		       //     cout << "need filter2D()" << endl;// (*this_.filter2D)((const uchar**)brows, dst, dststep, i, this_.roi.width, cn);




		    }


		    this_.dstY += dy;


		    ////////  FilterEngine__proceed finish  ///////

		    /*for (int indx = 0; indx < 1859 * 1312; indx++)
			{
				#pragma HLS PIPELINE
				dst.data[indx] = 0;

			}*/
		    //dst.data[0] = 0;



}





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

    //getGaussianKernel_(ksize.width, sigma1, std::max(depth, CV_32F), kx);

    float kernel_bitexact[7];
	kernel_bitexact[0] = 0.0701593;
	kernel_bitexact[1] = 0.131075;
	kernel_bitexact[2] = 0.190713;
	kernel_bitexact[3] = 0.216106;
	kernel_bitexact[4] = 0.190713;
	kernel_bitexact[5] = 0.131075;
	kernel_bitexact[6] = 0.0701593;

	for (int i = 0; i < 7; i++)
		kx.at<float>(i) = (float)kernel_bitexact[i];

	for (int i = 0; i < 7; i++)
		ky.at<float>(i) = (float)kernel_bitexact[i];

    //if (ksize.height == ksize.width && std::abs(sigma1 - sigma2) < DBL_EPSILON)
       // ky = kx;
   // else
      //   getGaussianKernel_(ksize.height, sigma2, std::max(depth, CV_32F), ky);
}




void GaussianBlur__(Mat src, Mat dst, Size ksize,
    double sigma1, double sigma2,
    int borderType)
{


    int type = src.type();

    Size size(src.cols, src.rows);
   // Size size = _src.size();

   // _dst.create(size, type);



    int sdepth = CV_MAT_DEPTH(type), cn = CV_MAT_CN(type);

    Mat kx, ky;
    kx.create_ker();
    ky.create_ker();
    //createGaussianKernels(kx, ky, type, ksize, sigma1, sigma2);
    float kernel_bitexact[7];
        kernel_bitexact[0] = 0.0701593;
        kernel_bitexact[1] = 0.131075;
        kernel_bitexact[2] = 0.190713;
        kernel_bitexact[3] = 0.216106;
        kernel_bitexact[4] = 0.190713;
        kernel_bitexact[5] = 0.131075;
        kernel_bitexact[6] = 0.0701593;

        for (int i = 0; i < 7; i++)
            kx.at<float>(i) = (float)kernel_bitexact[i];

        for (int i = 0; i < 7; i++)
            ky.at<float>(i) = (float)kernel_bitexact[i];



   // Mat src = _src.getMat();
   // Mat dst = _dst.getMat();

   // Point__ ofs;
   // Size wsz(src.cols, src.rows);
   // if (!(borderType & BORDER_ISOLATED_))
   //    src.locateROI(wsz, ofs);






    sepFilter2D__(src, dst, sdepth, kx, ky, Point__(-1, -1), 0, borderType);

}



static void
computeOrbDescriptors(const Mat& imagePyramid, const Rect* layerInfo,
    const float* layerScale, KeyPoint* keypoints,
    Mat& descriptors, const Point__* _pattern, int dsize, int wta_k, int KPsize)
{
    int step = (int)imagePyramid.step[0];
    int j, i, nkeypoints = KPsize;

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
#endif

        if (wta_k == 2)
        {
            for (i = 0; i < dsize; ++i, pattern += 16) //每個特徵描述子長度爲32個字節  
            {
            	#pragma HLS PIPELINE OFF
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


/*
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

*/

static void ICAngles(const Mat& img, const Rect* layerinfo,
		KeyPoint* pts, const int* u_max, int half_k, int allkpsize)
{
    int step = (int)img.step1();
    size_t ptidx, ptsize = allkpsize;

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
HarrisResponses(const Mat& img, const Rect* layerinfo,
		KeyPoint* pts, int blockSize, float harris_k, int allkpsize)
{
    //CV_CheckTypeEQ(img.type(), CV_8UC1, "");
    //CV_CheckGT(blockSize, 0, "");
    //CV_CheckLE(blockSize * blockSize, 2048, "");

    size_t ptidx, ptsize = allkpsize;

    const uchar* ptr00 = img.ptr<uchar>();
    size_t size_t_step = img.step[0];
    //CV_CheckLE(size_t_step * blockSize + blockSize + 1, (size_t)INT_MAX, "");  // ofs computation, step+1
    int step = static_cast<int>(size_t_step);

    int r = blockSize / 2;

    float scale = 1.f / ((1 << 2) * blockSize * 255.f);
    float scale_sq_sq = scale * scale * scale * scale;

    //AutoBuffer__<int> ofsbuf(blockSize * blockSize);
    int ofsbuf[7 * 7];

    int* ofs = ofsbuf;
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


void detect_fast(Mat& _image, KeyPoint* keypoints, Mat mask, int& kpSize)
{


    int threshold = 20;
    bool nonmaxSuppression = true;


    if (_image.empty())
    {
        cout << "in detect_fast: _image is empty " << endl;
        //keypoints.clear();
        //kpSize = 0;
        return;
    }


    Mat gray = _image;


    FAST__(gray, keypoints, threshold, nonmaxSuppression, kpSize);
    //KeyPointsFilter__::runByPixelsMask(keypoints, mask);
    //if (!mask.empty())
     //       keypoints.erase(std::remove_if(keypoints.begin(), keypoints.end(), MaskPredicate(mask)), keypoints.end());



    if (mask.empty())
            return;

        // keypoints.erase(std::remove_if(keypoints.begin(), keypoints.end(), MaskPredicate(mask)), keypoints.end());

    //cout << "i: " << keypoints[0].pt.x << endl;
   // cout << "i: " << keypoints[1].pt.x << endl;

        for (int i = 0; i < kpSize; i++)
        {
            if (mask.at<uchar>((int)(keypoints[i].pt.y + 0.5f), (int)(keypoints[i].pt.x + 0.5f)) == 0)
            {
                   keypoints[i].clear();
                   for (int j = i; j < kpSize - 1; j++)
                       keypoints[j] = keypoints[j + 1];

                   kpSize--;
                   i--;

            }


        }

       // cout << "in kpSize: " << kpSize << endl;
}

/*
void Insert(const KeyPoint& e, KeyPoint* a,  int i)
{ // insert e into ordered a[1:i]
    //a[0].response = e.response;
    while (e.response > a[i].response && i >= 0)
    {
        a[i + 1].response = a[i].response;
        a[i + 1].pt.x = a[i].pt.x;
        a[i + 1].pt.y = a[i].pt.y;
        a[i + 1].angle = a[i].angle;
        a[i + 1].size = a[i].size;
        a[i + 1].octave = a[i].octave;
        a[i + 1].class_id = a[i].class_id;

        i--;
    }

    a[i + 1].response = e.response;
    a[i + 1].pt.x = e.pt.x;
    a[i + 1].pt.y = e.pt.y;
    a[i + 1].angle = e.angle;
    a[i + 1].size = e.size;
    a[i + 1].octave = e.octave;
    a[i + 1].class_id = e.class_id;

}


void InsertionSort(KeyPoint* a, const int n)
{ // Sort a[1:n] into nondecreasing order
    for (int j = 1; j <= n; j++)
    {
        KeyPoint temp = a[j];
        Insert(temp, a, j - 1);
    }
}*/

void Merge(KeyPoint* a, KeyPoint* b, const int k, const int m, const int n)
{ // a[k:m], a[m+1:n] are sorted, merged to b[k:n]
    int i1 = k, i2 = m + 1, i3 = k;
    for (; i1 <= m && i2 <= n; i3++) {
#pragma HLS PIPELINE
        if (a[i1].response >= a[i2].response) {
            b[i3] = a[i1];
            i1++;
        }
        else {
            b[i3] = a[i2];
            i2++;
        }
    }
    // copy remaining records, if any, of 1st sublist
    if (i2 > n) //copy (a+i1, a+m+1, b+i3);
    {
        int i1_ = i1;
        for (; i1_ <= m; i1_++, i3++)
        {
#pragma HLS PIPELINE
            b[i3] = a[i1_];
        }
    }
    // copy remaining records, if any, of 2nd sublist
    if (i1 > m) //copy (a+i2, a+n+1, b+i3);
    {
        int i2_ = i2;
        for (; i2_ <= n; i2_++, i3++)
        {
#pragma HLS PIPELINE
            b[i3] = a[i2_];
        }
    }





}



void MergePass(KeyPoint* a, KeyPoint* b, const int n, const int s)
{//adjacent pairs of sublist of size s are merged from a to b. n = # records in a
    int i = 0;
    for (; //i the first position in first of the sublists merged
        i <= n - (2 * s); //enough elements for 2 sublists of length s?
        i += 2 * s) {

        Merge(a, b, i, i + s - 1, i + (2 * s) - 1);
    }
    // merge remaining lists of size < 2*s
    if ((i + s - 1) < n - 1) //one full and one partial lists
        Merge(a, b, i, i + s - 1, n - 1);
    else               //only one partial lists remained
    {
        for (; i < n; i++)
        {
#pragma HLS PIPELINE
            b[i] = a[i];
        }

    }
    //copy(a+i, b+n+1, b+i);
}


void MergeSort(KeyPoint* a, const int n)
{ // sort a[0:n] into non-decreasing order
    KeyPoint T[10600];
    KeyPoint* tempList = T;

    // s is the length of the currently merged sublist
    for (int s = 1; s < n; s *= 2)
    {
#pragma HLS PIPELINE
        MergePass(a, tempList, n, s);

        s *= 2;
        MergePass(tempList, a, n, s);
    }

}

void retainBestHarris(KeyPoint* keypoints, int n_points, int& kpSize)
{
    //this is only necessary if the keypoints size is greater than the number of desired points.
    if (n_points >= 0 && kpSize > (size_t)n_points)
    {
        if (n_points == 0)
        {
            for (int i = 0; i < kpSize; i++)
            {
                keypoints[i].clear();
            }
            kpSize = 0;
            return;
        }


        //InsertionSort(keypoints, kpSize);
        MergeSort(keypoints, kpSize);



        for (int i = n_points; i < kpSize; i++)
        {
            keypoints[i].clear();
        }
        kpSize = n_points;


    }
}

void retainBest(KeyPoint* keypoints, int n_points, int& kpSize)
{
    //this is only necessary if the keypoints size is greater than the number of desired points.
    if (n_points >= 0 && kpSize > (size_t)n_points)
    {
        if (n_points == 0)
        {
            for (int i = 0; i < kpSize; i++)
            {
                keypoints[i].clear();
            }
            kpSize = 0;
            return;
        }


        //InsertionSort(keypoints, kpSize);
        MergeSort(keypoints, kpSize);
        int buf = n_points;

        while (keypoints[buf - 1].response == keypoints[buf].response)
            buf++;


        for (int i = buf; i < kpSize; i++)
        {
            keypoints[i].clear();
        }
        kpSize = buf;



        /*
        //first use nth element to partition the keypoints into the best and worst.
        std::nth_element(keypoints.begin(), keypoints.begin() + n_points - 1, keypoints.end(), KeypointResponseGreater());
        //this is the boundary response, and in the case of FAST may be ambiguous
        float ambiguous_response = keypoints[n_points - 1].response;
        //use std::partition to grab all of the keypoints with the boundary response.
        std::vector<KeyPoint>::const_iterator new_end =
            std::partition(keypoints.begin() + n_points, keypoints.end(),
                KeypointResponseGreaterThanThreshold(ambiguous_response));
        //resize the keypoints, given this new end point. nth_element and partition reordered the points inplace
        keypoints.resize(new_end - keypoints.begin());*/
    }
}


void runByImageBorder(KeyPoint* keypoints, Size imageSize, int borderSize, int& kpSize)
{
    if (borderSize > 0)
    {
        if (imageSize.height <= borderSize * 2 || imageSize.width <= borderSize * 2)
        {
                //keypoints.clear();
            for (int i = 0; i < kpSize; i++)
            {
                keypoints[i].clear();
            }
            kpSize = 0;
        }
        else
        {
            Rect r = Rect(Point__(borderSize, borderSize), Point__(imageSize.width - borderSize, imageSize.height - borderSize));
            for (int i = 0; i < kpSize; i++)
            {
                if (!(r.x <= keypoints[i].pt.x && keypoints[i].pt.x < r.x + r.width && r.y <= keypoints[i].pt.y && keypoints[i].pt.y < r.y + r.height))
                {
                    keypoints[i].clear();
                    for (int j = i; j < kpSize - 1; j++)
                        keypoints[j] = keypoints[j + 1];

                    kpSize--;
                    i--;
                }


            }
        }

           // keypoints.erase(std::remove_if(keypoints.begin(), keypoints.end(),
           //     RoiPredicate(Rect(Point__(borderSize, borderSize),
           //         Point__(imageSize.width - borderSize, imageSize.height - borderSize)))),
           //     keypoints.end());
    }
}

static void computeKeyPoints(const Mat& imagePyramid,

    const Mat& maskPyramid,
    const Rect* layerInfo,

    const float* layerScale,
    KeyPoint* allKeypoints,
    int nfeatures, double scaleFactor,
    int edgeThreshold, int patchSize, int scoreType,
    int fastThreshold, int& outkpSize)
{


    int i, nkeypoints, level, nlevels = 8; //金字塔層數
    int nfeaturesPerLevel[8];

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
    int umax[15 + 2];

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

   // allKeypoints.clear();
   /* std::vector<KeyPoint> keypoints;
    std::vector<int> counters(nlevels);
    keypoints.reserve(nfeaturesPerLevel[0] * 2);
    //cout << "C: " << nfeaturesPerLevel[0] * 2 << endl;*/

	for (int i = 0; i < 1200; i++)
		allKeypoints[i].clear();

	int counters[8];
	static KeyPoint keypoints[10600];
	int kpSize;

	int useKPnum = 0;

    for (level = 0; level < nlevels; level++)
    {
        int featuresNum = nfeaturesPerLevel[level];
        Mat img;// = imagePyramid(layerInfo[level]);
        Mat mask;// = maskPyramid.empty() ? Mat() : maskPyramid(layerInfo[level]);
        Rect wholeLinfo = layerInfo[level];

        ///////    img      ///////

              img.flags = imagePyramid.flags;
              img.dims = 2;
              img.rows = wholeLinfo.height;
              img.cols = wholeLinfo.width;

              img.data = imagePyramid.data + wholeLinfo.y * imagePyramid.step[0];

              img.size[0] = img.rows;
              img.size[1] = img.cols;

              size_t eszi = CV_ELEM_SIZE(img.flags);
              img.data += wholeLinfo.x * eszi;

              if (wholeLinfo.width < imagePyramid.cols || wholeLinfo.height < imagePyramid.rows)
                  img.flags |= CV_SUBMAT_FLAG;

              img.step[0] = imagePyramid.step[0]; img.step[1] = eszi;
              img.updateContinuityFlag();

       ///////    mask      ///////

              mask.flags = maskPyramid.flags;
              mask.dims = 2;
              mask.rows = wholeLinfo.height;
              mask.cols = wholeLinfo.width;

              mask.data = maskPyramid.data + wholeLinfo.y * maskPyramid.step[0];

              mask.size[0] = mask.rows;
              mask.size[1] = mask.cols;

              size_t esz = CV_ELEM_SIZE(mask.flags);
              mask.data += wholeLinfo.x * esz;

              if (wholeLinfo.width < maskPyramid.cols || wholeLinfo.height < maskPyramid.rows)
                  mask.flags |= CV_SUBMAT_FLAG;

              mask.step[0] = maskPyramid.step[0]; mask.step[1] = esz;
              mask.updateContinuityFlag();







        // Detect FAST features, 20 is a good threshold
        {
            //Ptr<FastFeatureDetector__> fd = FastFeatureDetector__::create(fastThreshold, true);
            //fd->detect(img, keypoints, mask); //Fast角點檢測  



            detect_fast(img, keypoints, mask, kpSize); //Fast角點檢測

            //cout << "kpSize: " << kpSize << endl;

        }

        // Remove keypoints very close to the border

        Size img_size(img.cols, img.rows);
        runByImageBorder(keypoints, img_size, edgeThreshold, kpSize); //去除鄰近邊界的點

        // Keep more points than necessary as FAST does not give amazing corners
        retainBest(keypoints, scoreType == 0 ? 2 * featuresNum : featuresNum, kpSize);
        //按Fast強度排序,保留前2*featuresNum個特徵點

        //nkeypoints = (int)keypoints.size();
        nkeypoints = kpSize;
        counters[level] = nkeypoints;

        float sf = layerScale[level];
        for (i = 0; i < nkeypoints; i++) // Set the level of the coordinates  
        {
            keypoints[i].octave = level;  //層信息  
            keypoints[i].size = patchSize * sf;
        }

        //std::copy(keypoints.begin(), keypoints.end(), std::back_inserter(allKeypoints));

        for (int c = useKPnum, i = 0; i < kpSize; c++ , i++)
        {
        	allKeypoints[c].response = keypoints[i].response;
        	allKeypoints[c].pt.x = keypoints[i].pt.x;
        	allKeypoints[c].pt.y = keypoints[i].pt.y;
        	allKeypoints[c].angle = keypoints[i].angle;
        	allKeypoints[c].size = keypoints[i].size;
        	allKeypoints[c].octave = keypoints[i].octave;
        	allKeypoints[c].class_id = keypoints[i].class_id;

         }

                useKPnum += kpSize;



    }

   // cout << "keypoints: " << useKPnum << endl;
    // std::vector<Vec3i> ukeypoints_buf;

    nkeypoints = useKPnum;
    if (nkeypoints == 0)
    {
        return;
    }
    //Mat responses;


    // Select best features using the Harris cornerness (better scoring than FAST)


    HarrisResponses(imagePyramid, layerInfo, allKeypoints, 7, HARRIS_K, useKPnum);
    //計算每個角點的Harris強度響應  

   // std::vector<KeyPoint> newAllKeypoints;
   // newAllKeypoints.reserve(nfeaturesPerLevel[0] * nlevels);

    int offset = 0;
    useKPnum = 0;



    for (level = 0; level < nlevels; level++)    //
           {
               int featuresNum = nfeaturesPerLevel[level];
               kpSize = counters[level];




               for (int c = offset, i = 0; i < kpSize; c++, i++)
               {
            	   keypoints[i].response = allKeypoints[c].response;
            	   keypoints[i].pt.x = allKeypoints[c].pt.x;
            	   keypoints[i].pt.y = allKeypoints[c].pt.y;
                   keypoints[i].angle = allKeypoints[c].angle;
                   keypoints[i].size = allKeypoints[c].size;
                   keypoints[i].octave = allKeypoints[c].octave;
                   keypoints[i].class_id = allKeypoints[c].class_id;

               }


               offset += kpSize;

               //cull to the final desired level, using the new Harris scores.

               retainBestHarris(keypoints, featuresNum, kpSize); //按Harris強度排序，保留前featuresNum個

              // std::copy(keypoints.begin(), keypoints.end(), std::back_inserter(newAllKeypoints));



               for (int c = useKPnum, i = 0; i < kpSize; c++, i++)
               {
            	   allKeypoints[c].response = keypoints[i].response;
            	   allKeypoints[c].pt.x = keypoints[i].pt.x;
            	   allKeypoints[c].pt.y = keypoints[i].pt.y;
            	   allKeypoints[c].angle = keypoints[i].angle;
            	   allKeypoints[c].size = keypoints[i].size;
            	   allKeypoints[c].octave = keypoints[i].octave;
            	   allKeypoints[c].class_id = keypoints[i].class_id;

               }

               useKPnum += kpSize;

           }


    nkeypoints = useKPnum;


    {   // Process each keypoint  爲每個角點計算主方向，質心法
        ICAngles(imagePyramid, layerInfo, allKeypoints, umax, halfPatchSize, useKPnum);
    }

    for (i = 0; i < nkeypoints; i++)
    {
        float scale = layerScale[allKeypoints[i].octave];
        allKeypoints[i].pt *= scale;
    }

    outkpSize = useKPnum;

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
    KeyPoint* keypoints,
	uchar* des_data, bool useProvidedKeypoints, int& kpSize, uchar* imgPyramid_data)
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
   // mask.data[0] = 0;

    bool do_keypoints = !useProvidedKeypoints;
    bool do_descriptors = !do_keypoints;//descriptors.dims == 0 ? false : true;

    cout << "do_keypoints: " << do_keypoints << endl;
    cout << "do_descriptors: " << do_descriptors << endl;

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


    int i, nLevels = nlevels, level, nkeypoints = kpSize; //金字塔層數
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

    Rect layerInfo[8];
    int layerOfs[8];
    float layerScale[8];
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
		#pragma HLS PIPELINE
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
    cout << "bufSize: " << bufSize.height << " " << bufSize.width << endl;

    imagePyramid.create();
    imagePyramid.data = imgPyramid_data;



    //if (!mask.empty())
        maskPyramid.create1();






    Mat prevImg = image, prevMask = mask;
    //outmat(prevImg);


    // Pre-compute the scale pyramids

//#pragma HLS UNROLL factor=8
    for (level = 0; level < nLevels; ++level)
    {
		#pragma HLS PIPELINE

        Rect linfo = layerInfo[level];
        Size sz(linfo.width, linfo.height);
        Size wholeSize(sz.width + border * 2, sz.height + border * 2);
        Rect wholeLinfo = Rect(linfo.x - border, linfo.y - border, wholeSize.width, wholeSize.height);
        //Mat extImg = imagePyramid(wholeLinfo), extMask;
        //Mat extImg(imagePyramid, wholeLinfo);
        Mat extImg;

		extImg.flags = imagePyramid.flags;
		        extImg.dims = 2;
		        extImg.rows = wholeLinfo.height;
		        extImg.cols = wholeLinfo.width;

		        extImg.data = imagePyramid.data + wholeLinfo.y * imagePyramid.step[0];

		        extImg.size[0] = extImg.rows;
		        extImg.size[1] = extImg.cols;

		        size_t eszi = CV_ELEM_SIZE(extImg.flags);
		        extImg.data += wholeLinfo.x  * eszi;

		        if (wholeLinfo.width < imagePyramid.cols || wholeLinfo.height < imagePyramid.rows)
		            extImg.flags |= CV_SUBMAT_FLAG;

		        extImg.step[0] = imagePyramid.step[0]; extImg.step[1] = eszi;
		        extImg.updateContinuityFlag();





        Mat extMask;
        //Mat currImg = extImg(Rect(border, border, sz.width, sz.height)), currMask;
        Rect extImginfo = Rect(border, border, sz.width, sz.height);
        Mat currImg;//( extImg, extImginfo );
        Mat currMask;



        		currImg.flags = imagePyramid.flags;
                currImg.dims = 2;
                currImg.rows = sz.height;
                currImg.cols = sz.width;

                currImg.data = imagePyramid.data + (wholeLinfo.y + 32) * imagePyramid.step[0];

                currImg.size[0] = currImg.rows;
                currImg.size[1] = currImg.cols;

                size_t eszc = CV_ELEM_SIZE(currImg.flags);
                currImg.data += (wholeLinfo.x + 32) * eszc;

                if (extImginfo.width < extImg.cols || extImginfo.height < extImg.rows)
                    currImg.flags |= CV_SUBMAT_FLAG;

                currImg.step[0] = imagePyramid.step[0]; currImg.step[1] = eszc;
                currImg.updateContinuityFlag();

        //currImg.data[0] = 0;
        //cout << "L: " << level << endl;
        //cout << "extImg: " << extImg.flags << endl;//112404704
        //cout << "currImg: " << currImg.cols << " " << currImg.rows << " " << currImg.size[0] << endl;
        //Mat prevImg1 = currImg, prevMask1 = currMask;






       //if (!mask.empty())
        {

    	   //extMask.data = maskPyramid.data + wholeLinfo.y * maskPyramid.step[0];
    	   //extMask = maskPyramid(wholeLinfo);
    	   //maskPyramid.data[0] = 0;
    	   extMask.flags = maskPyramid.flags;
    	   extMask.dims = 2;
    	   extMask.rows = wholeLinfo.height;
    	   extMask.cols = wholeLinfo.width;

    	   extMask.data = maskPyramid.data + wholeLinfo.y * maskPyramid.step[0];

    	   extMask.size[0] = extMask.rows;
    	   extMask.size[1] = extMask.cols;

    	   size_t esz = CV_ELEM_SIZE(extMask.flags);
    	   extMask.data += wholeLinfo.x * esz;

    	   if (wholeLinfo.width < maskPyramid.cols || wholeLinfo.height < maskPyramid.rows)
    	         extMask.flags |= CV_SUBMAT_FLAG;

    	   extMask.step[0] = maskPyramid.step[0]; extMask.step[1] = esz;
    	   extMask.updateContinuityFlag();


           // currMask = extMask(Rect(border, border, sz.width, sz.height));
    	   Rect MaskLinfo = Rect(border, border, sz.width, sz.height);

    	   currMask.flags = maskPyramid.flags;
    	   currMask.dims = 2;
    	   currMask.rows = MaskLinfo.height;
    	   currMask.cols = MaskLinfo.width;

    	   currMask.data = maskPyramid.data + ( wholeLinfo.y + 32) * maskPyramid.step[0];

    	   currMask.size[0] = currMask.rows;
    	   currMask.size[1] = currMask.cols;

    	   size_t esz1 = CV_ELEM_SIZE(currMask.flags);
    	   currMask.data += ( wholeLinfo.x + 32) * esz1;

    	   if (MaskLinfo.width < extMask.cols || MaskLinfo.height < extMask.rows)
    	          currMask.flags |= CV_SUBMAT_FLAG;

    	   currMask.step[0] = maskPyramid.step[0]; currMask.step[1] = esz1;
    	   currMask.updateContinuityFlag();




        }

       // Compute the resized image
        if (level != firstLevel)  //得到金字塔每層的圖像
        {

        	resize__(prevImg, currImg, sz, 0, 0, INTER_LINEAR_EXACT);

             	 	 	 //if (!mask.empty())
                resize__(prevMask, currMask, sz, 0, 0, INTER_LINEAR_EXACT);

                			// if (level > firstLevel)
                threshold__(currMask, currMask, 254, 0, THRESH_TOZERO);

            copyMakeBorder__(currImg, extImg, border, border, border, border,
                BORDER_REFLECT_101 + BORDER_ISOLATED);
            			//if (!mask.empty())
                copyMakeBorder__(currMask, extMask, border, border, border, border,
                    BORDER_CONSTANT + BORDER_ISOLATED);

        }
        else
        {

            copyMakeBorder__(image, extImg, border, border, border, border, BORDER_REFLECT_101); //擴大圖像的邊界


           // if (!mask.empty())
                copyMakeBorder__(mask, extMask, border, border, border, border,
                    BORDER_CONSTANT + BORDER_ISOLATED);

        }
        if (level > firstLevel)
        {
            prevImg = currImg;
            prevMask = currMask;

        }

    }

    //imgPyramid_data = imagePyramid.data;
    //cout << "use: " << useOCL << endl;
    //cout << "layerInfo: " << layerInfo[level].x << " " << layerInfo[level].y << " " << layerInfo[level].height << " " << layerInfo[level].width << endl;
   /* for (int j = 0; j < 1859 * 1312; j++)
        {
			#pragma HLS PIPELINE
            imgPyramid_data[j] = imagePyramid.data[j];

        }*/

    //imgPyramid_data[1859000] = 0;

    if (do_keypoints) //提取角點  
    {



            // Get keypoints, those will be far enough from the border that no check will be required for the descriptor
        computeKeyPoints(imagePyramid, maskPyramid,  // need level8
            layerInfo, layerScale, keypoints,
            nfeatures, scaleFactor, edgeThreshold, patchSize, scoreType, fastThreshold, kpSize);
        cout << "kpSize" << kpSize  << endl;

     /*   for (int v = 0; v < 5; v++)
         {
        					#pragma HLS PIPELINE
        	keypoints[v ].pt.x = 0;
        	keypoints[v ].pt.y = 1;
         }*/
    }
   /* else
    {
    	cout << "kpSize" << kpSize  << endl;
        Size image_size(image.cols, image.rows);
        runByImageBorder(keypoints, image_size, edgeThreshold, kpSize);

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
             	 */

/*
        	cout<<"!sortedByLevel"<<endl;
        }
    }
*/
/*

    if (do_descriptors)
    {
    	Mat descriptors;
        //cout << "doing" << endl;
        int dsize = 32; //int dsize = descriptorSize();

        nkeypoints = kpSize;
        if (nkeypoints == 0)
        {
            //descriptors.release();
            return;
        }

        descriptors.create_des();
       // std::vector<Point__> pattern;

        //const int npoints = 512;
        //Point__ patternbuf[npoints];
        const Point__* pattern0 = (const Point__*)bit_pattern_31_;

        if (patchSize != 31)
        {
        	cout << "patchSize != 31~~~" << endl;
            //pattern0 = patternbuf;
            //makeRandomPattern(patchSize, patternbuf, npoints);
        }


        //cout << "wta_k: " << wta_k << endl; 2
       // if (wta_k == 2)
        //    std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));


        for (level = 0; level < nLevels; level++)
        {
            // preprocess the resized image
            Mat workingMat = imagePyramid(layerInfo[level]);

            //boxFilter(working_mat, working_mat, working_mat.depth(), Size(5,5), Point(-1,-1), true, BORDER_REFLECT_101);
            GaussianBlur__(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);


        }

cout<<"out"<<endl;

        //Mat descriptors = _descriptors.getMat();
        computeOrbDescriptors(imagePyramid, layerInfo, layerScale,
            keypoints, descriptors, pattern0, dsize, wta_k, kpSize);

        for (int l = 0; l < 500; l++)
           cout << l << ": " << keypoints[l].pt.x  << endl;


        for (int o = 0; o < 500 * 32; o++)
              des_data[o] = descriptors.data[o];

    }


*/

}



void detect(Mat image,
		KeyPoint* keypoints,
    Mat mask, int& kpSize, uchar* imgPyramid_data)
{

   if (image.empty())
    {
        //keypoints.clear();
        return;
    }
   uchar noArray[1];
    //mask.data[0] = 0;
    detectAndCompute(image, mask, keypoints, noArray, false, kpSize, imgPyramid_data);
}

void compute( uchar* imgPyramid_data,
		KeyPoint* keypoints,
		uchar* des_data, int& kpSize)
{


   //if (image.empty())
   // {
        //descriptors.release();
     //   return;
   // }
  /*  Mat noArray;
    uchar no[1];
    no[1] = 0;
    noArray.data = no;*/
   // cout << "descriptors: " << descriptors.dims << endl;
   // descriptors.dims = 2;
  //  cout << "descriptors: " << descriptors.dims << endl;

    //detectAndCompute(image, noArray, keypoints, des_data, true, kpSize);

	const float scaleFactor = 1.2f;
	const int nlevels = 8;
	const int edgeThreshold = 31;
	const int firstLevel = 0;
	const int wta_k = 2;
	const int patchSize = 31;

	bool do_keypoints = false;
	bool do_descriptors = true;

	const int HARRIS_BLOCK_SIZE = 9; //Harris角點響應需要的邊界大小
	int halfPatchSize = patchSize / 2; //鄰域半徑
	int descPatchSize = cvCeil__(halfPatchSize * sqrt(2.0));
	int border = std::max(edgeThreshold, std::max(descPatchSize, HARRIS_BLOCK_SIZE / 2)) + 1; //採用最大的邊界

	int i, nLevels = nlevels, level, nkeypoints = kpSize; //金字塔層數
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
			// cout << "sortedByLevel: " << sortedByLevel << endl;
		}
		nLevels++;
	}

	Rect layerInfo[8];
	int layerOfs[8];
	float layerScale[8];
	Mat imagePyramid;  //創建尺度金字塔圖像

	imagePyramid.createP();

	//int imcol2 = imagePyramid.cols;
	/*for (int j = 0; j < 1859 * 1312; j++)
	{
		#pragma HLS PIPELINE
		imagePyramid.data[j] = imgPyramid_data[j];

	}*/
	imagePyramid.data = imgPyramid_data;

	float level0_inv_scale = 1.0f / getScale(0, firstLevel, scaleFactor);       //cout << "level0_inv_scale: " << level0_inv_scale << endl; 1
	size_t level0_width = (size_t)cvRound__(1241 * level0_inv_scale);     //cout << "level0_width: " << level0_width << endl; 1241
	size_t level0_height = (size_t)cvRound__(376 * level0_inv_scale);    //cout << "level0_height: " << level0_height << endl; 376
	Size bufSize((int)alignSize__(level0_width + border * 2, 16), 0);  // TODO change alignment to 64


	int level_dy = (int)level0_height + border * 2;     //cout << "level_dy: " << level_dy << endl; 440
	Point__ level_ofs(0, 0);

	for (level = 0; level < nLevels; level++)
	{
#pragma HLS PIPELINE
		float scale = getScale(level, firstLevel, scaleFactor); //每層對應的尺度
		layerScale[level] = scale;
		float inv_scale = 1.0f / scale;
		Size sz(cvRound__(1241 * inv_scale), cvRound__(376 * inv_scale));//每層對應的圖像大小
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
	cout << "bufSize: " << bufSize.height << " " << bufSize.width << endl;

	cout << "kpSize" << kpSize << endl;
	//  cout << "before rbib: " << keypoints.size() << endl;
	Size image_size(1241, 376);
	//KeyPointsFilter__::runByImageBorder(keypoints, image_size, edgeThreshold);
	runByImageBorder(keypoints, image_size, edgeThreshold, kpSize); //
	if (!sortedByLevel)
	{
		/*std::vector<std::vector<KeyPoint> > allKeypoints(nLevels);
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
			 */


		cout<<"!sortedByLevel"<<endl;
	}

	if (do_descriptors)
	{
		Mat descriptors;
		//cout << "doing" << endl;
		int dsize = 32; //int dsize = descriptorSize();

		nkeypoints = kpSize;
		/*if (nkeypoints == 0)
		{
			//descriptors.release();
			return;
		}*/

		descriptors.create_des();
		descriptors.data = des_data;
	   // std::vector<Point__> pattern;

		//const int npoints = 512;
		//Point__ patternbuf[npoints];
		const Point__* pattern0 = (const Point__*)bit_pattern_31_;

		if (patchSize != 31)
		{
			cout << "patchSize != 31~~~" << endl;
			//pattern0 = patternbuf;
			//makeRandomPattern(patchSize, patternbuf, npoints);
		}


		//cout << "wta_k: " << wta_k << endl; 2
	   // if (wta_k == 2)
		//    std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));


		for (level = 0; level < nLevels; level++)
		{
			// preprocess the resized image
			//Mat workingMat = imagePyramid(layerInfo[level]);
			Mat workingMat;// = imagePyramid(layerInfo[level]);

			Rect wholeLinfo = layerInfo[level];

			workingMat.flags = imagePyramid.flags;
			workingMat.dims = 2;
			workingMat.rows = wholeLinfo.height;
			workingMat.cols = wholeLinfo.width;

			workingMat.data = imagePyramid.data + wholeLinfo.y * imagePyramid.step[0];

			workingMat.datastart = imagePyramid.datastart;
			workingMat.dataend = imagePyramid.dataend;

			workingMat.size[0] = workingMat.rows;
			workingMat.size[1] = workingMat.cols;

			size_t eszi = CV_ELEM_SIZE(workingMat.flags);
			workingMat.data += wholeLinfo.x * eszi;

			if (wholeLinfo.width < imagePyramid.cols || wholeLinfo.height < imagePyramid.rows)
				workingMat.flags |= CV_SUBMAT_FLAG;

			workingMat.step[0] = imagePyramid.step[0]; workingMat.step[1] = eszi;
			workingMat.updateContinuityFlag();

			//boxFilter(working_mat, working_mat, working_mat.depth(), Size(5,5), Point(-1,-1), true, BORDER_REFLECT_101);
			//GaussianBlur__(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);
			int type = workingMat.type();

			int sdepth = CV_MAT_DEPTH(type), cn = CV_MAT_CN(type);

			Mat kx, ky;

			kx.create_ker();
			ky.create_ker();

			float kernel_bitexact[7];
			kernel_bitexact[0] = 0.0701593;
			kernel_bitexact[1] = 0.131075;
			kernel_bitexact[2] = 0.190713;
			kernel_bitexact[3] = 0.216106;
			kernel_bitexact[4] = 0.190713;
			kernel_bitexact[5] = 0.131075;
			kernel_bitexact[6] = 0.0701593;

			for (int i = 0; i < 7; i++)
				kx.at<float>(i) = (float)kernel_bitexact[i];

			for (int i = 0; i < 7; i++)
				ky.at<float>(i) = (float)kernel_bitexact[i];

			//workingMat.data[0] = 0;
			sepFilter2D__(workingMat, workingMat, sdepth, kx, ky, Point__(-1, -1), 0, BORDER_REFLECT_101);


		}

cout<<"out"<<endl;
//cout<<"P: "<<static_cast<int>(imagePyramid.data[0])<<endl;
/*if (imagePyramid.data[376000] == 0)
{

	for (int o = 0; o <  32; o++)
		des_data[o] = 0;

}
else
{
	for (int o = 0; o < 32; o++)
			des_data[o] = 1;
}*/

		//Mat descriptors = _descriptors.getMat();
		computeOrbDescriptors(imagePyramid, layerInfo, layerScale,
			keypoints, descriptors, pattern0, dsize, wta_k, kpSize);

		for (int l = 0; l < kpSize; l++)
		   cout << l << ": " << keypoints[l].pt.x  << endl;


		/*for (int o = 0; o < kpSize * 32; o++)
			  des_data[o] = descriptors.data[o];*/

	}




}



void extract_features(unsigned char* image_data, unsigned char* mask_data, float* kp_xy, unsigned char* des_data)
{
#pragma HLS INTERFACE mode=m_axi depth=466616 port=image_data offset=slave
#pragma HLS INTERFACE mode=m_axi depth=466616 port=mask_data offset=slave
#pragma HLS INTERFACE mode=m_axi depth=1000 port=kp_xy offset=slave
#pragma HLS INTERFACE mode=m_axi depth=16000 port=des_data offset=slave

#pragma HLS INTERFACE mode=s_axilite port=image_data
#pragma HLS INTERFACE mode=s_axilite port=mask_data
#pragma HLS INTERFACE mode=s_axilite port=kp_xy
#pragma HLS INTERFACE mode=s_axilite port=des_data

#pragma HLS INTERFACE mode = s_axilite port = return


	cout<<"image_data: "<<static_cast<int>(image_data[0])<<endl;


	//Mat des;
	//des.dims = 1;
    Mat image, mask;//, des;
   // vector<KeyPoint> kp;
  //  vector<int> kp;
   image.flags = 1124024320;
    image.dims = 2;
    image.step[0] = 1241;
    image.step[1] = 1;
    image.rows = 376;
    image.cols = 1241;
    static unsigned char ch_image[1241 * 376];
    image.data = ch_image;
//#pragma HLS PIPELINE

    	//for(int i = 0; i < 10; i++)
    cout<<"data: "<<static_cast<int>(image.data[0])<<endl;

//#pragma HLS ARRAY_PARTITION variable=ch_image complete dim=1


    mask.flags = 1124024320;
    mask.dims = 2;
    mask.step[0] = 1241;
    mask.step[1] = 1;
    mask.rows = 376;
    mask.cols = 1241;
    static unsigned char ch_mask[1241 * 376];
    mask.data = ch_mask;


    for(int i = 0; i < 376*1241; i++)
	{
		#pragma HLS PIPELINE
		image.data[i] = image_data[i];
		mask.data[i] = mask_data[i];
	}
	//for(int i = 0; i < 376*1241; i++)

    //for(int i = 0; i < 376*1241; i++)
    //image.data = image_data;
    //	mask.data = mask_data;

    static KeyPoint kpnew[1200];
    KeyPoint* kp = kpnew;
    int kpSize = 0;

    static uchar imgPbuf[1859 * 1312];
    uchar* imgP = imgPbuf;

    //mask.data[376000] = 0;
    //image.data[376000] =0;

    /*if (mask.data[376000] == 0 || image.data[376000] == 0)
    	for (int v = 0; v < 500; v++)
    	       {
					#pragma HLS PIPELINE
    	           kp_xy[v * 2] = 0;
    	           kp_xy[v * 2 + 1] = 1;
    	       }*/

	detect(image, kp, mask, kpSize, imgP);
    cout << "detect end" << endl;
	//compute(imgP, kp, des_data, kpSize );
    cout << "compute end" << endl;

   //for(int o = 0; o < 500 * 32; o++)
   //         des_data[o] = des.data[o];

   for (int v = 0; v < kpSize; v++)
       {
           kp_xy[v * 2] = kp[v].pt.x;
           kp_xy[v * 2 + 1] = kp[v].pt.y;
       }

    //cout<<"des: "<<static_cast<int>(des_data[0])<<endl;

	//cout << "extract_features" << endl;

    //outmat(des);









}
