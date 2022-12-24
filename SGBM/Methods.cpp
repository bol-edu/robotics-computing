#include "Methods.h"
#include "Range__.h"
#include"opencv2/opencv.hpp"
using namespace cv;
#include <memory>  // std::shared_ptr


class CV_EXPORTS Algorithm__;

//template<typename _Tp, typename _EnumTp = void> struct ParamType {};


/** @brief This is a base class for all more or less complex algorithms in OpenCV

especially for classes of algorithms, for which there can be multiple implementations. The examples
are stereo correspondence (for which there are algorithms like block matching, semi-global block
matching, graph-cut etc.), background subtraction (which can be done using mixture-of-gaussians
models, codebook-based algorithm etc.), optical flow (block matching, Lucas-Kanade, Horn-Schunck
etc.).

Here is example of SimpleBlobDetector use in your application via Algorithm interface:
@snippet snippets/core_various.cpp Algorithm
*/
class CV_EXPORTS_W Algorithm__
{
public:
    Algorithm__();
    virtual ~Algorithm__();

    /** @brief Clears the algorithm state
    */
    CV_WRAP virtual void clear() {}

    /** @brief Stores algorithm parameters in a file storage
    */
    //virtual void write(FileStorage& fs) const { CV_UNUSED(fs); }

    /** @brief simplified API for language bindings
    * @overload
    */
    //CV_WRAP void write(const Ptr<FileStorage>& fs, const String& name = String()) const;

    /** @brief Reads algorithm parameters from a file storage
    */
    //CV_WRAP virtual void read(const FileNode& fn) { CV_UNUSED(fn); }

    /** @brief Returns true if the Algorithm is empty (e.g. in the very beginning or after unsuccessful read
    */
    CV_WRAP virtual bool empty() const { return false; }

    /** @brief Reads algorithm from the file node

    This is static template method of Algorithm. It's usage is following (in the case of SVM):
    @code
    cv::FileStorage fsRead("example.xml", FileStorage::READ);
    Ptr<SVM> svm = Algorithm::read<SVM>(fsRead.root());
    @endcode
    In order to make this method work, the derived class must overwrite Algorithm::read(const
    FileNode& fn) and also have static create() method without parameters
    (or with all the optional parameters)
    */
    /*
    template<typename _Tp> static Ptr<_Tp> read(const FileNode& fn)
    {
        Ptr<_Tp> obj = _Tp::create();
        obj->read(fn);
        return !obj->empty() ? obj : Ptr<_Tp>();
    }*/

    /** @brief Loads algorithm from the file

    @param filename Name of the file to read.
    @param objname The optional name of the node to read (if empty, the first top-level node will be used)

    This is static template method of Algorithm. It's usage is following (in the case of SVM):
    @code
    Ptr<SVM> svm = Algorithm::load<SVM>("my_svm_model.xml");
    @endcode
    In order to make this method work, the derived class must overwrite Algorithm::read(const
    FileNode& fn).
    */
    /*
    template<typename _Tp> static Ptr<_Tp> load(const String& filename, const String& objname = String())
    {
        FileStorage fs(filename, FileStorage::READ);
        CV_Assert(fs.isOpened());
        FileNode fn = objname.empty() ? fs.getFirstTopLevelNode() : fs[objname];
        if (fn.empty()) return Ptr<_Tp>();
        Ptr<_Tp> obj = _Tp::create();
        obj->read(fn);
        return !obj->empty() ? obj : Ptr<_Tp>();
    }*/

    /** @brief Loads algorithm from a String

    @param strModel The string variable containing the model you want to load.
    @param objname The optional name of the node to read (if empty, the first top-level node will be used)

    This is static template method of Algorithm. It's usage is following (in the case of SVM):
    @code
    Ptr<SVM> svm = Algorithm::loadFromString<SVM>(myStringModel);
    @endcode
    */
    /*
    template<typename _Tp> static Ptr<_Tp> loadFromString(const String& strModel, const String& objname = String())
    {
        FileStorage fs(strModel, FileStorage::READ + FileStorage::MEMORY);
        FileNode fn = objname.empty() ? fs.getFirstTopLevelNode() : fs[objname];
        Ptr<_Tp> obj = _Tp::create();
        obj->read(fn);
        return !obj->empty() ? obj : Ptr<_Tp>();
    }*/

    /** Saves the algorithm to a file.
    In order to make this method work, the derived class must implement Algorithm::write(FileStorage& fs). */
    //CV_WRAP virtual void save(const String& filename) const;

    /** Returns the algorithm string identifier.
    This string is used as top level xml/yml node tag when the object is saved to a file or string. */
    //CV_WRAP virtual String getDefaultName() const;
    /*
protected:
    void writeFormat(FileStorage& fs) const;*/
};

/*enum struct Param {
    INT = 0, BOOLEAN = 1, REAL = 2, STRING = 3, MAT = 4, MAT_VECTOR = 5, ALGORITHM = 6, FLOAT = 7,
    UNSIGNED_INT = 8, UINT64 = 9, UCHAR = 11, SCALAR = 12
};*/


Algorithm__::Algorithm__()
{
    //CV_TRACE_FUNCTION();
}

Algorithm__::~Algorithm__()
{
    //CV_TRACE_FUNCTION();
}
/*
void Algorithm__::write(const Ptr<FileStorage>& fs, const String& name) const
{
    //CV_TRACE_FUNCTION();
    if (name.empty())
    {
        write(*fs);
        return;
    }
    *fs << name << "{";
    write(*fs);
    *fs << "}";
}*/
/*
void Algorithm__::save(const String& filename) const
{
    //CV_TRACE_FUNCTION();
    FileStorage fs(filename, FileStorage::WRITE);
    fs << getDefaultName() << "{";
    write(fs);
    fs << "}";
}

String Algorithm__::getDefaultName() const
{
    //CV_TRACE_FUNCTION();
    return String("my_object");
}*/
/*
void Algorithm__::writeFormat(FileStorage& fs) const
{
    //CV_TRACE_FUNCTION();
    fs << "format" << (int)3;
}*/


class CV_EXPORTS_W StereoMatcher__ : public Algorithm__
{
public:
    enum {
        DISP_SHIFT = 4,
        DISP_SCALE = (1 << DISP_SHIFT)
    };

    /** @brief Computes disparity map for the specified stereo pair

    @param left Left 8-bit single-channel image.
    @param right Right image of the same size and the same type as the left one.
    @param disparity Output disparity map. It has the same size as the input images. Some algorithms,
    like StereoBM or StereoSGBM compute 16-bit fixed-point disparity map (where each disparity value
    has 4 fractional bits), whereas other algorithms output 32-bit floating-point disparity map.
     */
    CV_WRAP virtual void compute(InputArray left, InputArray right,
        OutputArray disparity) = 0;

    CV_WRAP virtual int getMinDisparity() const = 0;
    CV_WRAP virtual void setMinDisparity(int minDisparity) = 0;

    CV_WRAP virtual int getNumDisparities() const = 0;
    CV_WRAP virtual void setNumDisparities(int numDisparities) = 0;

    CV_WRAP virtual int getBlockSize() const = 0;
    CV_WRAP virtual void setBlockSize(int blockSize) = 0;

    CV_WRAP virtual int getSpeckleWindowSize() const = 0;
    CV_WRAP virtual void setSpeckleWindowSize(int speckleWindowSize) = 0;

    CV_WRAP virtual int getSpeckleRange() const = 0;
    CV_WRAP virtual void setSpeckleRange(int speckleRange) = 0;

    CV_WRAP virtual int getDisp12MaxDiff() const = 0;
    CV_WRAP virtual void setDisp12MaxDiff(int disp12MaxDiff) = 0;
};


class CV_EXPORTS_W StereoSGBM__ : public StereoMatcher__
{
public:
    enum
    {
        MODE_SGBM = 0,
        MODE_HH = 1,
        MODE_SGBM_3WAY = 2,
        MODE_HH4 = 3
    };

    virtual int getPreFilterCap() const = 0;
    virtual void setPreFilterCap(int preFilterCap) = 0;

    virtual int getUniquenessRatio() const = 0;
    virtual void setUniquenessRatio(int uniquenessRatio) = 0;

    virtual int getP1() const = 0;
    virtual void setP1(int P1) = 0;

    virtual int getP2() const = 0;
    virtual void setP2(int P2) = 0;

    virtual int getMode() const = 0;
    virtual void setMode(int mode) = 0;

    CV_WRAP static Ptr<StereoSGBM__> create(int minDisparity = 0, int numDisparities = 16, int blockSize = 3,
        int P1 = 0, int P2 = 0, int disp12MaxDiff = 0,
        int preFilterCap = 0, int uniquenessRatio = 0,
        int speckleWindowSize = 0, int speckleRange = 0,
        int mode = StereoSGBM__::MODE_SGBM);
};

struct v_int16x8
{
    typedef short lane_type;
    typedef __m128i vector_type;
    enum { nlanes = 8 };

    /* coverity[uninit_ctor]: suppress warning */
    //v_int16x8() {}
    explicit v_int16x8(__m128i v) : val(v) {}
    v_int16x8(short v0, short v1, short v2, short v3, short v4, short v5, short v6, short v7)
    {
        val = _mm_setr_epi16((short)v0, (short)v1, (short)v2, (short)v3,
            (short)v4, (short)v5, (short)v6, (short)v7);
    }

    short get0() const
    {
        return (short)_mm_cvtsi128_si32(val);
    }

    __m128i val;
};


#define CV_SIMD CV_SIMD128
#define CV_SIMD_64F CV_SIMD128_64F
#define CV_SIMD_WIDTH 16

    typedef v_int16x8   v_int16;

    //CV_INTRIN_DEFINE_WIDE_INTRIN_ALL_TYPES(v)
#if CV_SIMD128_64F
        typedef v_float64x2 v_float64;
    CV_INTRIN_DEFINE_WIDE_INTRIN(double, v_float64, f64, v, load)
#endif
        //inline void vx_cleanup() { v_cleanup(); }



#if CV_SIMD
#if CV_SIMD_WIDTH == 16
static inline v_int16 vx_setseq_s16()
{
    return v_int16(0, 1, 2, 3, 4, 5, 6, 7);
}
#elif CV_SIMD_WIDTH == 32
static inline v_int16 vx_setseq_s16()
{
    return v_int16(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);
}
#elif CV_SIMD_WIDTH == 64
static inline v_int16 vx_setseq_s16()
{
    return v_int16(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31);
}
#else
struct vseq_s16
{
    short data[v_int16::nlanes];
    vseq_s16()
    {
        for (int i = 0; i < v_int16::nlanes; i++)
            data[i] = i;
    }
};
static inline v_int16 vx_setseq_s16()
{
    static vseq_s16 vseq;
    return vx_load(vseq.data);
}
#endif
// define some additional reduce operations:
static inline void min_pos(const v_int16& val, const v_int16& pos, short& min_val, short& min_pos)
{
    min_val = v_reduce_min(val);
    v_int16 v_mask = (vx_setall_s16(min_val) == val);
    min_pos = v_reduce_min(((pos + vx_setseq_s16()) & v_mask) | (vx_setall_s16(SHRT_MAX) & ~v_mask));
}
#endif


typedef uchar PixType;
typedef short CostType;
typedef short DispType;

// NR - the number of directions. the loop on x that computes Lr assumes that NR == 8.
// if you change NR, please, modify the loop as well.
enum { NR = 8, NR2 = NR / 2 };


struct StereoSGBMParams__
{
    StereoSGBMParams__()
    {
        minDisparity = numDisparities = 0;
        SADWindowSize = 0;
        P1 = P2 = 0;
        disp12MaxDiff = 0;
        preFilterCap = 0;
        uniquenessRatio = 0;
        speckleWindowSize = 0;
        speckleRange = 0;
        mode = StereoSGBM__::MODE_SGBM;
    }

    StereoSGBMParams__(int _minDisparity, int _numDisparities, int _SADWindowSize,
        int _P1, int _P2, int _disp12MaxDiff, int _preFilterCap,
        int _uniquenessRatio, int _speckleWindowSize, int _speckleRange,
        int _mode)
    {
        minDisparity = _minDisparity;
        numDisparities = _numDisparities;
        SADWindowSize = _SADWindowSize;
        P1 = _P1;
        P2 = _P2;
        disp12MaxDiff = _disp12MaxDiff;
        preFilterCap = _preFilterCap;
        uniquenessRatio = _uniquenessRatio;
        speckleWindowSize = _speckleWindowSize;
        speckleRange = _speckleRange;
        mode = _mode;
    }

    inline bool isFullDP() const
    {
        return mode == StereoSGBM__::MODE_HH || mode == StereoSGBM__::MODE_HH4;
    }
    inline Size calcSADWindowSize() const
    {
        const int dim = SADWindowSize > 0 ? SADWindowSize : 5;
        return Size(dim, dim);
    }

    int minDisparity;
    int numDisparities;
    int SADWindowSize;
    int preFilterCap;
    int uniquenessRatio;
    int P1;
    int P2;
    int speckleWindowSize;
    int speckleRange;
    int disp12MaxDiff;
    int mode;
};

//alignSize---------------------------------------------------------------------------//
static inline size_t alignSize__(size_t sz, int n)
{
    //CV_DbgAssert((n & (n - 1)) == 0); // n is a power of 2
    return (sz + n - 1) & -n;
}


static const int DEFAULT_RIGHT_BORDER = -1;


static void calcPixelCostBT__(const Mat& img1, const Mat& img2, int y,
    int minD, int maxD, CostType* cost,
    PixType* buffer, const PixType* tab,
    int xrange_min = 0,int xrange_max = -1
    )
{

    int x, c, width = img1.cols, cn = img1.channels();
    int minX1 = max(maxD, 0), maxX1 = width + min(minD, 0);
    int D = (int)alignSize__(maxD - minD, v_int16::nlanes), width1 = maxX1 - minX1;
    //This minX1 & maxX2 correction is defining which part of calculatable line must be calculated
    //That is needs of parallel algorithm
    xrange_min = (xrange_min < 0) ? 0 : xrange_min;
    xrange_max = (xrange_max == -1) || (xrange_max > width1) ? width1 : xrange_max;
    maxX1 = minX1 + xrange_max;
    minX1 += xrange_min;
    width1 = maxX1 - minX1;
    int minX2 = max(minX1 - maxD, 0), maxX2 = min(maxX1 - minD, width);
    int width2 = maxX2 - minX2;
    const PixType* row1 = img1.ptr<PixType>(y), * row2 = img2.ptr<PixType>(y);
    PixType* prow1 = buffer + width2 * 2, * prow2 = prow1 + width * cn * 2;

    for (c = 0; c < cn * 2; c++)
    {
        prow1[width * c] = prow1[width * c + width - 1] =
            prow2[width * c] = prow2[width * c + width - 1] = tab[0];
    }

    int n1 = y > 0 ? -(int)img1.step : 0, s1 = y < img1.rows - 1 ? (int)img1.step : 0;
    int n2 = y > 0 ? -(int)img2.step : 0, s2 = y < img2.rows - 1 ? (int)img2.step : 0;

    int minX_cmn = min(minX1, minX2) - 1;
    int maxX_cmn = max(maxX1, maxX2) + 1;
    minX_cmn = max(minX_cmn, 1);
    maxX_cmn = min(maxX_cmn, width - 1);
    if (cn == 1)
    {
        for (x = minX_cmn; x < maxX_cmn; x++)
        {
            prow1[x] = tab[(row1[x + 1] - row1[x - 1]) * 2 + row1[x + n1 + 1] - row1[x + n1 - 1] + row1[x + s1 + 1] - row1[x + s1 - 1]];
            prow2[width - 1 - x] = tab[(row2[x + 1] - row2[x - 1]) * 2 + row2[x + n2 + 1] - row2[x + n2 - 1] + row2[x + s2 + 1] - row2[x + s2 - 1]];

            prow1[x + width] = row1[x];
            prow2[width - 1 - x + width] = row2[x];
        }
    }
    else
    {
        for (x = minX_cmn; x < maxX_cmn; x++)
        {
            prow1[x] = tab[(row1[x * 3 + 3] - row1[x * 3 - 3]) * 2 + row1[x * 3 + n1 + 3] - row1[x * 3 + n1 - 3] + row1[x * 3 + s1 + 3] - row1[x * 3 + s1 - 3]];
            prow1[x + width] = tab[(row1[x * 3 + 4] - row1[x * 3 - 2]) * 2 + row1[x * 3 + n1 + 4] - row1[x * 3 + n1 - 2] + row1[x * 3 + s1 + 4] - row1[x * 3 + s1 - 2]];
            prow1[x + width * 2] = tab[(row1[x * 3 + 5] - row1[x * 3 - 1]) * 2 + row1[x * 3 + n1 + 5] - row1[x * 3 + n1 - 1] + row1[x * 3 + s1 + 5] - row1[x * 3 + s1 - 1]];

            prow2[width - 1 - x] = tab[(row2[x * 3 + 3] - row2[x * 3 - 3]) * 2 + row2[x * 3 + n2 + 3] - row2[x * 3 + n2 - 3] + row2[x * 3 + s2 + 3] - row2[x * 3 + s2 - 3]];
            prow2[width - 1 - x + width] = tab[(row2[x * 3 + 4] - row2[x * 3 - 2]) * 2 + row2[x * 3 + n2 + 4] - row2[x * 3 + n2 - 2] + row2[x * 3 + s2 + 4] - row2[x * 3 + s2 - 2]];
            prow2[width - 1 - x + width * 2] = tab[(row2[x * 3 + 5] - row2[x * 3 - 1]) * 2 + row2[x * 3 + n2 + 5] - row2[x * 3 + n2 - 1] + row2[x * 3 + s2 + 5] - row2[x * 3 + s2 - 1]];

            prow1[x + width * 3] = row1[x * 3];
            prow1[x + width * 4] = row1[x * 3 + 1];
            prow1[x + width * 5] = row1[x * 3 + 2];

            prow2[width - 1 - x + width * 3] = row2[x * 3];
            prow2[width - 1 - x + width * 4] = row2[x * 3 + 1];
            prow2[width - 1 - x + width * 5] = row2[x * 3 + 2];
        }
    }

    memset(cost + xrange_min * D, 0, width1 * D * sizeof(cost[0]));

    buffer -= width - maxX2;
    cost -= (minX1 - xrange_min) * D + minD; // simplify the cost indices inside the loop

    for (c = 0; c < cn * 2; c++, prow1 += width, prow2 += width)
    {
        int diff_scale = c < cn ? 0 : 2;

        // precompute
        //   v0 = min(row2[x-1/2], row2[x], row2[x+1/2]) and
        //   v1 = max(row2[x-1/2], row2[x], row2[x+1/2]) and
        //   to process values from [minX2, maxX2) we should check memory location (width - 1 - maxX2, width - 1 - minX2]
        //   so iterate through [width - maxX2, width - minX2)
        for (x = width - maxX2; x < width - minX2; x++)
        {
            int v = prow2[x];
            int vl = x > 0 ? (v + prow2[x - 1]) / 2 : v;
            int vr = x < width - 1 ? (v + prow2[x + 1]) / 2 : v;
            int v0 = min(vl, vr); v0 = min(v0, v);
            int v1 = max(vl, vr); v1 = max(v1, v);
            buffer[x] = (PixType)v0;
            buffer[x + width2] = (PixType)v1;
        }

        for (x = minX1; x < maxX1; x++)
        {
            int u = prow1[x];
            int ul = x > 0 ? (u + prow1[x - 1]) / 2 : u;
            int ur = x < width - 1 ? (u + prow1[x + 1]) / 2 : u;
            int u0 = min(ul, ur); u0 = min(u0, u);
            int u1 = max(ul, ur); u1 = max(u1, u);

            int d = minD;
#if CV_SIMD
            v_uint8 _u = vx_setall_u8((uchar)u), _u0 = vx_setall_u8((uchar)u0);
            v_uint8 _u1 = vx_setall_u8((uchar)u1);

            for (; d <= maxD - 2 * v_int16::nlanes; d += 2 * v_int16::nlanes)
            {
                v_uint8 _v = vx_load(prow2 + width - x - 1 + d);
                v_uint8 _v0 = vx_load(buffer + width - x - 1 + d);
                v_uint8 _v1 = vx_load(buffer + width - x - 1 + d + width2);
                v_uint8 c0 = v_max(_u - _v1, _v0 - _u);
                v_uint8 c1 = v_max(_v - _u1, _u0 - _v);
                v_uint8 diff = v_min(c0, c1);

                v_int16 _c0 = vx_load_aligned(cost + x * D + d);
                v_int16 _c1 = vx_load_aligned(cost + x * D + d + v_int16::nlanes);

                v_uint16 diff1, diff2;
                v_expand(diff, diff1, diff2);
                v_store_aligned(cost + x * D + d, _c0 + v_reinterpret_as_s16(diff1 >> diff_scale));
                v_store_aligned(cost + x * D + d + v_int16::nlanes, _c1 + v_reinterpret_as_s16(diff2 >> diff_scale));
            }
#endif
            for (; d < maxD; d++)
            {
                int v = prow2[width - x - 1 + d];
                int v0 = buffer[width - x - 1 + d];
                int v1 = buffer[width - x - 1 + d + width2];
                int c0 = max(0, u - v1); c0 = max(c0, v0 - u);
                int c1 = max(0, v - u1); c1 = max(c1, u0 - v);

                cost[x * D + d] = (CostType)(cost[x * D + d] + (min(c0, c1) >> diff_scale));
            }
        }
    }
}




#ifndef OPENCV_UTILS_BUFFER_AREA_HPP
#define OPENCV_UTILS_BUFFER_AREA_HPP


        //! @addtogroup core_utils
        //! @{

        /** @brief Manages memory block shared by muliple buffers.

        This class allows to allocate one large memory block and split it into several smaller
        non-overlapping buffers. In safe mode each buffer allocation will be performed independently,
        this mode allows dynamic memory access instrumentation using valgrind or memory sanitizer.

        Safe mode can be explicitly switched ON in constructor. It will also be enabled when compiling with
        memory sanitizer support or in runtime with the environment variable `OPENCV_BUFFER_AREA_ALWAYS_SAFE`.

        Example of usage:
        @code
        int * buf1 = 0;
        double * buf2 = 0;
        cv::util::BufferArea area;
        area.allocate(buf1, 200); // buf1 = new int[200];
        area.allocate(buf2, 1000, 64); // buf2 = new double[1000]; - aligned by 64
        area.commit();
        @endcode

        @note This class is considered private and should be used only in OpenCV itself. API can be changed.
        */
        class CV_EXPORTS BufferArea__
        {
        public:
            /** @brief Class constructor.

            @param safe Enable _safe_ operation mode, each allocation will be performed independently.
            */
            BufferArea__(bool safe = false);

            /** @brief Class destructor

            All allocated memory well be freed. Each bound pointer will be reset to NULL.
            */
            ~BufferArea__();

            /** @brief Bind a pointer to local area.

            BufferArea will store reference to the pointer and allocation parameters effectively owning the
            pointer and allocated memory. This operation has the same parameters and does the same job
            as the operator `new`, except allocation can be performed later during the BufferArea::commit call.

            @param ptr Reference to a pointer of type T. Must be NULL
            @param count Count of objects to be allocated, it has the same meaning as in the operator `new`.
            @param alignment Alignment of allocated memory. same meaning as in the operator `new` (C++17).
                             Must be divisible by sizeof(T). Must be power of two.

            @note In safe mode allocation will be performed immediatly.
            */
            template <typename T>
            void allocate(T*& ptr, size_t count, ushort alignment = sizeof(T))
            {
                //CV_Assert(ptr == NULL);
                //CV_Assert(count > 0);
                //CV_Assert(alignment > 0);
                //CV_Assert(alignment % sizeof(T) == 0);
                //CV_Assert((alignment & (alignment - 1)) == 0);
                allocate_((void**)(&ptr), static_cast<ushort>(sizeof(T)), count, alignment);
/*#ifndef OPENCV_ENABLE_MEMORY_SANITIZER
                if (safe)
#endif
                    CV_Assert(ptr != NULL);*/
            }

            /** @brief Fill one of buffers with zeroes

            @param ptr pointer to memory block previously added using BufferArea::allocate

            BufferArea::commit must be called before using this method
            */
            template <typename T>
            void zeroFill(T*& ptr)
            {
                //CV_Assert(ptr);
                zeroFill_((void**)&ptr);
            }

            /** @brief Fill all buffers with zeroes

            BufferArea::commit must be called before using this method
            */
            void zeroFill();

            /** @brief Allocate memory and initialize all bound pointers

            Each pointer bound to the area with the BufferArea::allocate will be initialized and will be set
            to point to a memory block with requested size and alignment.

            @note Does nothing in safe mode as all allocations will be performed by BufferArea::allocate
            */
            void commit();

            /** @brief Release all memory and unbind all pointers

            All memory will be freed and all pointers will be reset to NULL and untied from the area allowing
            to call `allocate` and `commit` again.
            */
            void release();

        private:
            BufferArea__(const BufferArea__&); // = delete
            BufferArea__& operator=(const BufferArea__&); // = delete
            void allocate_(void** ptr, ushort type_size, size_t count, ushort alignment);
            void zeroFill_(void** ptr);

        private:
            class Block;
            std::vector<Block> blocks;
#ifndef OPENCV_ENABLE_MEMORY_SANITIZER
            void* oneBuf;
            size_t totalSize;
            const bool safe;
#endif
        };

        //! @}

    
 // cv::utils::

#endif


        CV_EXPORTS void fastFree__(void* ptr);

#ifdef OPENCV_ALLOC_ENABLE_STATISTICS
        static inline
            void fastFree_(void* ptr)
#else
        void fastFree__(void* ptr)
#endif
        {
#if defined HAVE_POSIX_MEMALIGN || defined HAVE_MEMALIGN
            if (isAlignedAllocationEnabled())
            {
                free(ptr);
                return;
            }
#endif
            if (ptr)
            {
                uchar* udata = ((uchar**)ptr)[-1];
                //CV_DbgAssert(udata < (uchar*)ptr &&
                    //((uchar*)ptr - udata) <= (ptrdiff_t)(sizeof(void*) + CV_MALLOC_ALIGN));
                free(udata);
            }
        }

        template<typename _Tp> static inline _Tp* alignPtr__(_Tp* ptr, int n = (int)sizeof(_Tp))
        {
            CV_DbgAssert((n & (n - 1)) == 0); // n is a power of 2
            return (_Tp*)(((size_t)ptr + n - 1) & -n);
        }

#define  CV_MALLOC_ALIGN    64

        CV_EXPORTS void* fastMalloc__(size_t bufSize);

        
#ifdef OPENCV_ALLOC_ENABLE_STATISTICS
        static inline
            void* fastMalloc_(size_t size)
#else
        void* fastMalloc__(size_t size)
#endif
        {
#ifdef HAVE_POSIX_MEMALIGN
            if (isAlignedAllocationEnabled())
            {
                void* ptr = NULL;
                if (posix_memalign(&ptr, CV_MALLOC_ALIGN, size))
                    ptr = NULL;
                if (!ptr)
                    return OutOfMemoryError(size);
                return ptr;
            }
#elif defined HAVE_MEMALIGN
            if (isAlignedAllocationEnabled())
            {
                void* ptr = memalign(CV_MALLOC_ALIGN, size);
                if (!ptr)
                    return OutOfMemoryError(size);
                return ptr;
            }
#endif
            uchar* udata = (uchar*)malloc(size + sizeof(void*) + CV_MALLOC_ALIGN);
            //if (!udata)
                //return OutOfMemoryError(size);
            uchar** adata = alignPtr__((uchar**)udata + 1, CV_MALLOC_ALIGN);
            adata[-1] = udata;
            return adata;
        }
        
//#pragma warning( disable : 4996 )

static inline const char* envRead(const char* name)
{
#ifdef NO_GETENV
    CV_UNUSED(name);
    return NULL;
#else
    cout << "UUUUUUUsing getenv";
    char* s;
    //s = getenv(name);
    size_t len;
    errno_t err = _dupenv_s(&s, &len, name);
    if (err) exit(-1);

    //return getenv(name);
    return s;
#endif
}
class ParseError
{
    std::string bad_value;
public:
    ParseError(const std::string bad_value_) :bad_value(bad_value_) {}
    std::string toString(const std::string& param) const
    {
        std::ostringstream out;
        out << "Invalid value for parameter " << param << ": " << bad_value;
        return out.str();
    }
};
template <typename T>
T parseOption(const std::string&);

template<>
inline bool parseOption(const std::string& value)
{
    if (value == "1" || value == "True" || value == "true" || value == "TRUE")
    {
        return true;
    }
    if (value == "0" || value == "False" || value == "false" || value == "FALSE")
    {
        return false;
    }
    cout << "Invalid value for parameter ";
    throw ParseError(value);


}

template<>
inline size_t parseOption(const std::string& value)
{
    size_t pos = 0;
    for (; pos < value.size(); pos++)
    {
        if (!isdigit(value[pos]))
            break;
    }
    cv::String valueStr = value.substr(0, pos);
    cv::String suffixStr = value.substr(pos, value.length() - pos);
    size_t v = (size_t)std::stoull(valueStr);
    if (suffixStr.length() == 0)
        return v;
    else if (suffixStr == "MB" || suffixStr == "Mb" || suffixStr == "mb")
        return v * 1024 * 1024;
    else if (suffixStr == "KB" || suffixStr == "Kb" || suffixStr == "kb")
        return v * 1024;
    cout << "Invalid value for parameter ";
    throw ParseError(value);


}
template<typename T>
inline T read(const std::string& k, const T& defaultValue)
{
    try
    {
        const char* res = envRead(k.c_str());
        if (res)
            return parseOption<T>(std::string(res));
    }
    
    catch (const ParseError& err)
    {
        //CV_Error(cv::Error::StsBadArg, err.toString(k));
    }
    cout << "return defaultV";
    return defaultValue;
}
bool getConfigurationParameterBool(const char* name, bool defaultValue)
{
    cout << "Buffer__";
    return read<bool>(name, defaultValue);
}
#ifndef OPENCV_ENABLE_MEMORY_SANITIZER
static bool CV_BUFFER_AREA_OVERRIDE_SAFE_MODE =
getConfigurationParameterBool("OPENCV_BUFFER_AREA_ALWAYS_SAFE", false);
#endif




        //==================================================================================================

        class BufferArea__::Block
        {
        private:
            inline size_t reserve_count() const
            {
                return alignment / type_size - 1;
            }
        public:
            Block(void** ptr_, ushort type_size_, size_t count_, ushort alignment_)
                : ptr(ptr_), raw_mem(0), count(count_), type_size(type_size_), alignment(alignment_)
            {
                //CV_Assert(ptr && *ptr == NULL);
            }
            void cleanup() const
            {
                //CV_Assert(ptr && *ptr);
                *ptr = 0;
                if (raw_mem)
                    fastFree__(raw_mem);
            }
            size_t getByteCount() const
            {
                return type_size * (count + reserve_count());
            }
            void real_allocate()
            {
                //CV_Assert(ptr && *ptr == NULL);
                const size_t allocated_count = count + reserve_count();
uchar* udata = (uchar*)malloc(allocated_count + sizeof(void*) + CV_MALLOC_ALIGN);
                cout << udata << endl;
                raw_mem = fastMalloc__(type_size * allocated_count);
                
                if (alignment != type_size)
                {
                    *ptr = alignPtr__(raw_mem, alignment);
                    //CV_Assert(reinterpret_cast<size_t>(*ptr) % alignment == 0);
                    //CV_Assert(static_cast<uchar*>(*ptr) + type_size * count <= static_cast<uchar*>(raw_mem) + type_size * allocated_count);
                }
                else
                {
                    *ptr = raw_mem;
                }
            }
#ifndef OPENCV_ENABLE_MEMORY_SANITIZER
            void* fast_allocate(void* buf) const
            {
                //CV_Assert(ptr && *ptr == NULL);
                buf = alignPtr__(buf, alignment);
                //CV_Assert(reinterpret_cast<size_t>(buf) % alignment == 0);
                *ptr = buf;
                return static_cast<void*>(static_cast<uchar*>(*ptr) + type_size * count);
            }
#endif
            bool operator==(void** other) const
            {
                //CV_Assert(ptr && other);
                return *ptr == *other;
            }
            void zeroFill() const
            {
                //CV_Assert(ptr && *ptr);
                memset(static_cast<uchar*>(*ptr), 0, count * type_size);
            }
        private:
            void** ptr;
            void* raw_mem;
            size_t count;
            ushort type_size;
            ushort alignment;
        };

        //==================================================================================================

#ifndef OPENCV_ENABLE_MEMORY_SANITIZER
        BufferArea__::BufferArea__(bool safe_) :
            oneBuf(0),
            totalSize(0),
            safe(safe_ || CV_BUFFER_AREA_OVERRIDE_SAFE_MODE)
        {
            // nothing
        }
#else
        BufferArea__::BufferArea__(bool safe_)
        {
            CV_UNUSED(safe_);
        }
#endif

        BufferArea__::~BufferArea__()
        {
            release();
        }

        void BufferArea__::allocate_(void** ptr, ushort type_size, size_t count, ushort alignment)
        {
            blocks.push_back(Block(ptr, type_size, count, alignment));
#ifndef OPENCV_ENABLE_MEMORY_SANITIZER
            if (!safe)
            {
                totalSize += blocks.back().getByteCount();
            }
            else
#endif
            {
                blocks.back().real_allocate();
            }
        }

        void BufferArea__::zeroFill_(void** ptr)
        {
            for (std::vector<Block>::const_iterator i = blocks.begin(); i != blocks.end(); ++i)
            {
                if (*i == ptr)
                {
                    i->zeroFill();
                    break;
                }
            }
        }

        void BufferArea__::zeroFill()
        {
            for (std::vector<Block>::const_iterator i = blocks.begin(); i != blocks.end(); ++i)
            {
                i->zeroFill();
            }
        }

        void BufferArea__::commit()
        {
#ifndef OPENCV_ENABLE_MEMORY_SANITIZER
            if (!safe)
            {
                //CV_Assert(totalSize > 0);
                //CV_Assert(oneBuf == NULL);
                //CV_Assert(!blocks.empty());
                oneBuf = fastMalloc__(totalSize);
                void* ptr = oneBuf;
                for (std::vector<Block>::const_iterator i = blocks.begin(); i != blocks.end(); ++i)
                {
                    ptr = i->fast_allocate(ptr);
                }
            }
#endif
        }

        void BufferArea__::release()
        {
            for (std::vector<Block>::const_iterator i = blocks.begin(); i != blocks.end(); ++i)
            {
                i->cleanup();
            }
            blocks.clear();
#ifndef OPENCV_ENABLE_MEMORY_SANITIZER
            if (oneBuf)
            {
                fastFree__(oneBuf);
                oneBuf = 0;
            }
#endif
        }

        //==================================================================================================

    
 // cv::utils::




class BufferSGBM__
{
private:
    size_t width1;
    size_t Da;
    size_t Dlra;
    size_t costWidth;
    size_t costHeight;
    size_t hsumRows;
    bool fullDP;
    uchar dirs;
    uchar dirs2;
    static const size_t TAB_OFS = 256 * 4;

public:
    CostType* Cbuf;
    CostType* Sbuf;
    CostType* hsumBuf;
    CostType* pixDiff;
    CostType* disp2cost;
    DispType* disp2ptr;
    PixType* tempBuf;
    std::vector<CostType*> Lr;
    std::vector<CostType*> minLr;
    PixType* clipTab;

private:
    BufferArea__ area;

public:
    BufferSGBM__(size_t width1_,
        size_t Da_,
        size_t Dlra_,
        size_t cn,
        size_t width,
        size_t height,
        const StereoSGBMParams__& params)
        : width1(width1_),
        Da(Da_),
        Dlra(Dlra_),
        Cbuf(NULL),
        Sbuf(NULL),
        hsumBuf(NULL),
        pixDiff(NULL),
        disp2cost(NULL),
        disp2ptr(NULL),
        tempBuf(NULL),
        Lr(2, (CostType*)NULL),
        minLr(2, (CostType*)NULL),
        clipTab(NULL)
    {
        const size_t TAB_SIZE = 256 + TAB_OFS * 2;
        fullDP = params.isFullDP();
        costWidth = width1 * Da;
        costHeight = fullDP ? height : 1;
        hsumRows = params.calcSADWindowSize().height + 2;
        dirs = params.mode == StereoSGBM__::MODE_HH4 ? 1 : NR;
        dirs2 = params.mode == StereoSGBM__::MODE_HH4 ? 1 : NR2;
        // for each possible stereo match (img1(x,y) <=> img2(x-d,y))
        // we keep pixel difference cost (C) and the summary cost over NR directions (S).
        // we also keep all the partial costs for the previous line L_r(x,d) and also min_k L_r(x, k)
        area.allocate(Cbuf, costWidth * costHeight, CV_SIMD_WIDTH); // summary cost over different (nDirs) directions
        area.allocate(Sbuf, costWidth * costHeight, CV_SIMD_WIDTH);
        area.allocate(hsumBuf, costWidth * hsumRows, CV_SIMD_WIDTH);
        area.allocate(pixDiff, costWidth, CV_SIMD_WIDTH);
        area.allocate(disp2cost, width, CV_SIMD_WIDTH);
        area.allocate(disp2ptr, width, CV_SIMD_WIDTH);
        area.allocate(tempBuf, width * (4 * cn + 2), CV_SIMD_WIDTH);
        // the number of L_r(.,.) and min_k L_r(.,.) lines in the buffer:
        // for 8-way dynamic programming we need the current row and
        // the previous row, i.e. 2 rows in total
        for (size_t i = 0; i < 2; ++i)
        {
            // 2D: [ NR ][ w1 * NR2 ][ NR ] * [ Dlra ]
            area.allocate(Lr[i], calcLrCount() * Dlra, CV_SIMD_WIDTH);
            // 1D: [ NR ][ w1 * NR2 ][ NR ]
            area.allocate(minLr[i], calcLrCount(), CV_SIMD_WIDTH);
        }
        area.allocate(clipTab, TAB_SIZE, CV_SIMD_WIDTH);
        area.commit();

        // init clipTab
        const int ftzero = max(params.preFilterCap, 15) | 1;
        for (int i = 0; i < (int)TAB_SIZE; i++)
            clipTab[i] = (PixType)(min(max(i - (int)TAB_OFS, -ftzero), ftzero) + ftzero);
    }
    inline const PixType* getClipTab() const
    {
        return clipTab + TAB_OFS;
    }
    inline void initCBuf(CostType val) const
    {
        for (size_t i = 0; i < costWidth * costHeight; ++i)
            Cbuf[i] = val;
    }
    inline void clearLr(const Range__& range = Range__::all()) const
    {
        for (uchar i = 0; i < 2; ++i)
        {
            if (range == Range__::all())
            {
                memset(Lr[i], 0, calcLrCount() * Dlra * sizeof(CostType));
                memset(minLr[i], 0, calcLrCount() * sizeof(CostType));
            }
            else
            {
                memset(getLr(i, range.start), 0, range.size() * sizeof(CostType) * Dlra);
                memset(getMinLr(i, range.start), 0, range.size() * sizeof(CostType));
            }
        }
    }
    inline size_t calcLrCount() const
    {
        return width1 * dirs2 + 2 * dirs;
    }
    inline void swapLr()
    {
        std::swap(Lr[0], Lr[1]);
        std::swap(minLr[0], minLr[1]);
    }
    inline CostType* getHSumBuf(int row) const
    {
        return hsumBuf + (row % hsumRows) * costWidth;
    }
    inline CostType* getCBuf(int row) const
    {
        //CV_Assert(row >= 0);
        return Cbuf + (!fullDP ? 0 : (row * costWidth));
    }
    inline CostType* getSBuf(int row) const
    {
        //CV_Assert(row >= 0);
        return Sbuf + (!fullDP ? 0 : (row * costWidth));
    }
    inline void clearSBuf(int row, const Range__& range = Range__::all()) const
    {
        if (range == Range__::all())
            memset(getSBuf(row), 0, costWidth * sizeof(CostType));
        else
            memset(getSBuf(row) + range.start * Da, 0, range.size() * Da * sizeof(CostType));
    }

    // shift Lr[k] and minLr[k] pointers, because we allocated them with the borders,
    // and will occasionally use negative indices with the arrays
    // we need to shift Lr[k] pointers by 1, to give the space for d=-1.
    inline CostType* getLr(uchar id, int idx, uchar shift = 0) const
    {
        //CV_Assert(id < 2);
        const size_t fixed_offset = dirs * Dlra;
        return Lr[id] + fixed_offset + (idx * (int)dirs2 + (int)shift) * (int)Dlra;
    }
    inline CostType* getMinLr(uchar id, int idx, uchar shift = 0) const
    {
        //CV_Assert(id < 2);
        const size_t fixed_offset = dirs;
        return minLr[id] + fixed_offset + (idx * dirs2 + shift);
    }
};

template<typename _Tp> static inline _Tp saturate_cast__(int v) { return _Tp(v); }
template<> inline short saturate_cast__<short>(int v) { return (short)((unsigned)(v - SHRT_MIN) <= (unsigned)USHRT_MAX ? v : v > 0 ? SHRT_MAX : SHRT_MIN); }

static void computeDisparitySGBM__(const Mat& img1, const Mat& img2,
    Mat& disp1, const StereoSGBMParams__& params)
{
    const int DISP_SHIFT = StereoMatcher__::DISP_SHIFT;
    const int DISP_SCALE = (1 << DISP_SHIFT);
    const CostType MAX_COST = SHRT_MAX;

    int minD = params.minDisparity, maxD = minD + params.numDisparities;
    int uniquenessRatio = params.uniquenessRatio >= 0 ? params.uniquenessRatio : 10;
    int disp12MaxDiff = params.disp12MaxDiff > 0 ? params.disp12MaxDiff : 1;
    int P1 = params.P1 > 0 ? params.P1 : 2, P2 = max(params.P2 > 0 ? params.P2 : 5, P1 + 1);
    int k, width = disp1.cols, height = disp1.rows;
    int minX1 = max(maxD, 0), maxX1 = width + min(minD, 0);
    const int D = params.numDisparities;
    int width1 = maxX1 - minX1;
    int Da = (int)alignSize__(D, v_int16::nlanes);
    int Dlra = Da + v_int16::nlanes;//Additional memory is necessary to store disparity values(MAX_COST) for d=-1 and d=D
    int INVALID_DISP = minD - 1, INVALID_DISP_SCALED = INVALID_DISP * DISP_SCALE;
    int SW2 = params.calcSADWindowSize().width / 2, SH2 = params.calcSADWindowSize().height / 2;
    int npasses = params.isFullDP() ? 2 : 1;

    if (minX1 >= maxX1)
    {
        disp1 = Scalar__::all(INVALID_DISP_SCALED);
        return;
    }

    BufferSGBM__ mem(width1, Da, Dlra, img1.channels(), width, height, params);
    mem.initCBuf((CostType)P2); // add P2 to every C(x,y). it saves a few operations in the inner loops

    for (int pass = 1; pass <= npasses; pass++)
    {
        int x1, y1, x2, y2, dx, dy;

        if (pass == 1)
        {
            y1 = 0; y2 = height; dy = 1;
            x1 = 0; x2 = width1; dx = 1;
        }
        else
        {
            y1 = height - 1; y2 = -1; dy = -1;
            x1 = width1 - 1; x2 = -1; dx = -1;
        }

        uchar lrID = 0;
        mem.clearLr();

        for (int y = y1; y != y2; y += dy)
        {
            int x, d;
            DispType* disp1ptr = disp1.ptr<DispType>(y);
            CostType* const C = mem.getCBuf(y);
            CostType* const S = mem.getSBuf(y);

            if (pass == 1) // compute C on the first pass, and reuse it on the second pass, if any.
            {
                int dy1 = y == 0 ? 0 : y + SH2, dy2 = y == 0 ? SH2 : dy1;

                for (k = dy1; k <= dy2; k++)
                {
                    CostType* hsumAdd = mem.getHSumBuf(min(k, height - 1));

                    if (k < height)
                    {
                        calcPixelCostBT__(img1, img2, k, minD, maxD, mem.pixDiff, mem.tempBuf, mem.getClipTab());

                        memset(hsumAdd, 0, Da * sizeof(CostType));
#if CV_SIMD
                        v_int16 h_scale = vx_setall_s16((short)SW2 + 1);
                        for (d = 0; d < Da; d += v_int16::nlanes)
                        {
                            v_int16 v_hsumAdd = vx_load_aligned(mem.pixDiff + d) * h_scale;
                            for (x = Da; x <= SW2 * Da; x += Da)
                                v_hsumAdd += vx_load_aligned(mem.pixDiff + x + d);
                            v_store_aligned(hsumAdd + d, v_hsumAdd);
                        }
#else
                        for (d = 0; d < D; d++)
                        {
                            hsumAdd[d] = (CostType)(mem.pixDiff[d] * (SW2 + 1));
                            for (x = Da; x <= SW2 * Da; x += Da)
                                hsumAdd[d] = (CostType)(hsumAdd[d] + mem.pixDiff[x + d]);
                        }
#endif

                        if (y > 0)
                        {
                            const CostType* hsumSub = mem.getHSumBuf(max(y - SH2 - 1, 0));
                            const CostType* Cprev = mem.getCBuf(y - 1);

#if CV_SIMD
                            for (d = 0; d < Da; d += v_int16::nlanes)
                                v_store_aligned(C + d, vx_load_aligned(Cprev + d) + vx_load_aligned(hsumAdd + d) - vx_load_aligned(hsumSub + d));
#else
                            for (d = 0; d < D; d++)
                                C[d] = (CostType)(Cprev[d] + hsumAdd[d] - hsumSub[d]);
#endif

                            for (x = Da; x < width1 * Da; x += Da)
                            {
                                const CostType* pixAdd = mem.pixDiff + min(x + SW2 * Da, (width1 - 1) * Da);
                                const CostType* pixSub = mem.pixDiff + max(x - (SW2 + 1) * Da, 0);
#if CV_SIMD
                                for (d = 0; d < Da; d += v_int16::nlanes)
                                {
                                    v_int16 hv = vx_load_aligned(hsumAdd + x - Da + d) - vx_load_aligned(pixSub + d) + vx_load_aligned(pixAdd + d);
                                    v_store_aligned(hsumAdd + x + d, hv);
                                    v_store_aligned(C + x + d, vx_load_aligned(Cprev + x + d) - vx_load_aligned(hsumSub + x + d) + hv);
                                }
#else
                                for (d = 0; d < D; d++)
                                {
                                    int hv = hsumAdd[x + d] = (CostType)(hsumAdd[x - Da + d] + pixAdd[d] - pixSub[d]);
                                    C[x + d] = (CostType)(Cprev[x + d] + hv - hsumSub[x + d]);
                                }
#endif
                            }
                        }
                        else
                        {
#if CV_SIMD
                            v_int16 v_scale = vx_setall_s16(k == 0 ? (short)SH2 + 1 : 1);
                            for (d = 0; d < Da; d += v_int16::nlanes)
                                v_store_aligned(C + d, vx_load_aligned(C + d) + vx_load_aligned(hsumAdd + d) * v_scale);
#else
                            int scale = k == 0 ? SH2 + 1 : 1;
                            for (d = 0; d < D; d++)
                                C[d] = (CostType)(C[d] + hsumAdd[d] * scale);
#endif
                            for (x = Da; x < width1 * Da; x += Da)
                            {
                                const CostType* pixAdd = mem.pixDiff + min(x + SW2 * Da, (width1 - 1) * Da);
                                const CostType* pixSub = mem.pixDiff + max(x - (SW2 + 1) * Da, 0);

#if CV_SIMD
                                for (d = 0; d < Da; d += v_int16::nlanes)
                                {
                                    v_int16 hv = vx_load_aligned(hsumAdd + x - Da + d) + vx_load_aligned(pixAdd + d) - vx_load_aligned(pixSub + d);
                                    v_store_aligned(hsumAdd + x + d, hv);
                                    v_store_aligned(C + x + d, vx_load_aligned(C + x + d) + hv * v_scale);
                                }
#else
                                for (d = 0; d < D; d++)
                                {
                                    CostType hv = (CostType)(hsumAdd[x - Da + d] + pixAdd[d] - pixSub[d]);
                                    hsumAdd[x + d] = hv;
                                    C[x + d] = (CostType)(C[x + d] + hv * scale);
                                }
#endif
                            }
                        }
                    }
                    else
                    {
                        if (y > 0)
                        {
                            const CostType* hsumSub = mem.getHSumBuf(max(y - SH2 - 1, 0));
                            const CostType* Cprev = mem.getCBuf(y - 1);
#if CV_SIMD
                            for (x = 0; x < width1 * Da; x += v_int16::nlanes)
                                v_store_aligned(C + x, vx_load_aligned(Cprev + x) - vx_load_aligned(hsumSub + x) + vx_load_aligned(hsumAdd + x));
#else
                            for (x = 0; x < width1 * Da; x++)
                                C[x] = (CostType)(Cprev[x] + hsumAdd[x] - hsumSub[x]);
#endif
                        }
                        else
                        {
#if CV_SIMD
                            for (x = 0; x < width1 * Da; x += v_int16::nlanes)
                                v_store_aligned(C + x, vx_load_aligned(C + x) + vx_load_aligned(hsumAdd + x));
#else
                            for (x = 0; x < width1 * Da; x++)
                                C[x] = (CostType)(C[x] + hsumAdd[x]);
#endif
                        }
                    }

                }

                // also, clear the S buffer
                mem.clearSBuf(y);
            }

            /*
             [formula 13 in the paper]
             compute L_r(p, d) = C(p, d) +
             min(L_r(p-r, d),
             L_r(p-r, d-1) + P1,
             L_r(p-r, d+1) + P1,
             min_k L_r(p-r, k) + P2) - min_k L_r(p-r, k)
             where p = (x,y), r is one of the directions.
             we process all the directions at once:
             0: r=(-dx, 0)
             1: r=(-1, -dy)
             2: r=(0, -dy)
             3: r=(1, -dy)   !!!Note that only directions 0 to 3 are processed
             4: r=(-2, -dy)
             5: r=(-1, -dy*2)
             6: r=(1, -dy*2)
             7: r=(2, -dy)
             */

            for (x = x1; x != x2; x += dx)
            {
                int delta0 = P2 + *mem.getMinLr(lrID, x - dx);
                int delta1 = P2 + *mem.getMinLr(1 - lrID, x - 1, 1);
                int delta2 = P2 + *mem.getMinLr(1 - lrID, x, 2);
                int delta3 = P2 + *mem.getMinLr(1 - lrID, x + 1, 3);

                CostType* Lr_p0 = mem.getLr(lrID, x - dx);
                CostType* Lr_p1 = mem.getLr(1 - lrID, x - 1, 1);
                CostType* Lr_p2 = mem.getLr(1 - lrID, x, 2);
                CostType* Lr_p3 = mem.getLr(1 - lrID, x + 1, 3);

                Lr_p0[-1] = Lr_p0[D] = MAX_COST;
                Lr_p1[-1] = Lr_p1[D] = MAX_COST;
                Lr_p2[-1] = Lr_p2[D] = MAX_COST;
                Lr_p3[-1] = Lr_p3[D] = MAX_COST;

                CostType* Lr_p = mem.getLr(lrID, x);
                const CostType* Cp = C + x * Da;
                CostType* Sp = S + x * Da;

                CostType* minL = mem.getMinLr(lrID, x);
                d = 0;
#if CV_SIMD
                v_int16 _P1 = vx_setall_s16((short)P1);

                v_int16 _delta0 = vx_setall_s16((short)delta0);
                v_int16 _delta1 = vx_setall_s16((short)delta1);
                v_int16 _delta2 = vx_setall_s16((short)delta2);
                v_int16 _delta3 = vx_setall_s16((short)delta3);
                v_int16 _minL0 = vx_setall_s16((short)MAX_COST);
                v_int16 _minL1 = vx_setall_s16((short)MAX_COST);
                v_int16 _minL2 = vx_setall_s16((short)MAX_COST);
                v_int16 _minL3 = vx_setall_s16((short)MAX_COST);

                for (; d <= D - v_int16::nlanes; d += v_int16::nlanes)
                {
                    v_int16 Cpd = vx_load_aligned(Cp + d);
                    v_int16 Spd = vx_load_aligned(Sp + d);
                    v_int16 L;

                    L = v_min(v_min(v_min(vx_load_aligned(Lr_p0 + d), vx_load(Lr_p0 + d - 1) + _P1), vx_load(Lr_p0 + d + 1) + _P1), _delta0) - _delta0 + Cpd;
                    v_store_aligned(Lr_p + d, L);
                    _minL0 = v_min(_minL0, L);
                    Spd += L;

                    L = v_min(v_min(v_min(vx_load_aligned(Lr_p1 + d), vx_load(Lr_p1 + d - 1) + _P1), vx_load(Lr_p1 + d + 1) + _P1), _delta1) - _delta1 + Cpd;
                    v_store_aligned(Lr_p + d + Dlra, L);
                    _minL1 = v_min(_minL1, L);
                    Spd += L;

                    L = v_min(v_min(v_min(vx_load_aligned(Lr_p2 + d), vx_load(Lr_p2 + d - 1) + _P1), vx_load(Lr_p2 + d + 1) + _P1), _delta2) - _delta2 + Cpd;
                    v_store_aligned(Lr_p + d + Dlra * 2, L);
                    _minL2 = v_min(_minL2, L);
                    Spd += L;

                    L = v_min(v_min(v_min(vx_load_aligned(Lr_p3 + d), vx_load(Lr_p3 + d - 1) + _P1), vx_load(Lr_p3 + d + 1) + _P1), _delta3) - _delta3 + Cpd;
                    v_store_aligned(Lr_p + d + Dlra * 3, L);
                    _minL3 = v_min(_minL3, L);
                    Spd += L;

                    v_store_aligned(Sp + d, Spd);
                }

#if CV_SIMD_WIDTH > 32
                minL[0] = v_reduce_min(_minL0);
                minL[1] = v_reduce_min(_minL1);
                minL[2] = v_reduce_min(_minL2);
                minL[3] = v_reduce_min(_minL3);
#else
                // Get minimum for L0-L3
                v_int16 t0, t1, t2, t3;
                v_zip(_minL0, _minL2, t0, t2);
                v_zip(_minL1, _minL3, t1, t3);
                v_zip(v_min(t0, t2), v_min(t1, t3), t0, t1);
                t0 = v_min(t0, t1);
                t0 = v_min(t0, v_rotate_right<4>(t0));
#if CV_SIMD_WIDTH == 32
                CostType buf[v_int16::nlanes];
                v_store_low(buf, v_min(t0, v_rotate_right<8>(t0)));
                minL[0] = buf[0];
                minL[1] = buf[1];
                minL[2] = buf[2];
                minL[3] = buf[3];
#else
                v_store_low(minL, t0);
#endif
#endif
#else
                minL[0] = MAX_COST;
                minL[1] = MAX_COST;
                minL[2] = MAX_COST;
                minL[3] = MAX_COST;
#endif
                for (; d < D; d++)
                {
                    int Cpd = Cp[d], L;
                    int Spd = Sp[d];

                    L = Cpd + min((int)Lr_p0[d], min(Lr_p0[d - 1] + P1, min(Lr_p0[d + 1] + P1, delta0))) - delta0;
                    Lr_p[d] = (CostType)L;
                    minL[0] = min(minL[0], (CostType)L);
                    Spd += L;

                    L = Cpd + min((int)Lr_p1[d], min(Lr_p1[d - 1] + P1, min(Lr_p1[d + 1] + P1, delta1))) - delta1;
                    Lr_p[d + Dlra] = (CostType)L;
                    minL[1] = min(minL[1], (CostType)L);
                    Spd += L;

                    L = Cpd + min((int)Lr_p2[d], min(Lr_p2[d - 1] + P1, min(Lr_p2[d + 1] + P1, delta2))) - delta2;
                    Lr_p[d + Dlra * 2] = (CostType)L;
                    minL[2] = min(minL[2], (CostType)L);
                    Spd += L;

                    L = Cpd + min((int)Lr_p3[d], min(Lr_p3[d - 1] + P1, min(Lr_p3[d + 1] + P1, delta3))) - delta3;
                    Lr_p[d + Dlra * 3] = (CostType)L;
                    minL[3] = min(minL[3], (CostType)L);
                    Spd += L;

                    Sp[d] = saturate_cast__<CostType>(Spd);
                }
            }

            if (pass == npasses)
            {
                x = 0;
#if CV_SIMD
                v_int16 v_inv_dist = vx_setall_s16((DispType)INVALID_DISP_SCALED);
                v_int16 v_max_cost = vx_setall_s16(MAX_COST);
                for (; x <= width - v_int16::nlanes; x += v_int16::nlanes)
                {
                    v_store(disp1ptr + x, v_inv_dist);
                    v_store(mem.disp2ptr + x, v_inv_dist);
                    v_store(mem.disp2cost + x, v_max_cost);
                }
#endif
                for (; x < width; x++)
                {
                    disp1ptr[x] = mem.disp2ptr[x] = (DispType)INVALID_DISP_SCALED;
                    mem.disp2cost[x] = MAX_COST;
                }

                for (x = width1 - 1; x >= 0; x--)
                {
                    CostType* Sp = S + x * Da;
                    CostType minS = MAX_COST;
                    short bestDisp = -1;

                    if (npasses == 1)
                    {
                        CostType* Lr_p0 = mem.getLr(lrID, x + 1);
                        Lr_p0[-1] = Lr_p0[D] = MAX_COST;
                        CostType* Lr_p = mem.getLr(lrID, x);

                        const CostType* Cp = C + x * Da;

                        d = 0;
                        int delta0 = P2 + *mem.getMinLr(lrID, x + 1);
                        int minL0 = MAX_COST;
#if CV_SIMD
                        v_int16 _P1 = vx_setall_s16((short)P1);
                        v_int16 _delta0 = vx_setall_s16((short)delta0);

                        v_int16 _minL0 = vx_setall_s16((short)MAX_COST);
                        v_int16 _minS = vx_setall_s16(MAX_COST), _bestDisp = vx_setall_s16(-1);
                        for (; d <= D - v_int16::nlanes; d += v_int16::nlanes)
                        {
                            v_int16 Cpd = vx_load_aligned(Cp + d);
                            v_int16 L0 = v_min(v_min(v_min(vx_load_aligned(Lr_p0 + d), vx_load(Lr_p0 + d - 1) + _P1), vx_load(Lr_p0 + d + 1) + _P1), _delta0) - _delta0 + Cpd;

                            v_store_aligned(Lr_p + d, L0);
                            _minL0 = v_min(_minL0, L0);
                            L0 += vx_load_aligned(Sp + d);
                            v_store_aligned(Sp + d, L0);

                            _bestDisp = v_select(_minS > L0, vx_setall_s16((short)d), _bestDisp);
                            _minS = v_min(_minS, L0);
                        }
                        minL0 = (CostType)v_reduce_min(_minL0);
                        min_pos(_minS, _bestDisp, minS, bestDisp);
#endif
                        for (; d < D; d++)
                        {
                            int L0 = Cp[d] + min((int)Lr_p0[d], min(Lr_p0[d - 1] + P1, min(Lr_p0[d + 1] + P1, delta0))) - delta0;

                            Lr_p[d] = (CostType)L0;
                            minL0 = min(minL0, L0);

                            CostType Sval = Sp[d] = saturate_cast__<CostType>(Sp[d] + L0);
                            if (Sval < minS)
                            {
                                minS = Sval;
                                bestDisp = (short)d;
                            }
                        }
                        *mem.getMinLr(lrID, x) = (CostType)minL0;
                    }
                    else
                    {
                        d = 0;
#if CV_SIMD
                        v_int16 _minS = vx_setall_s16(MAX_COST), _bestDisp = vx_setall_s16(-1);
                        for (; d <= D - v_int16::nlanes; d += v_int16::nlanes)
                        {
                            v_int16 L0 = vx_load_aligned(Sp + d);
                            _bestDisp = v_select(_minS > L0, vx_setall_s16((short)d), _bestDisp);
                            _minS = v_min(L0, _minS);
                        }
                        min_pos(_minS, _bestDisp, minS, bestDisp);
#endif
                        for (; d < D; d++)
                        {
                            int Sval = Sp[d];
                            if (Sval < minS)
                            {
                                minS = (CostType)Sval;
                                bestDisp = (short)d;
                            }
                        }
                    }

                    for (d = 0; d < D; d++)
                    {
                        if (Sp[d] * (100 - uniquenessRatio) < minS * 100 && std::abs(bestDisp - d) > 1)
                            break;
                    }
                    if (d < D)
                        continue;
                    d = bestDisp;
                    int _x2 = x + minX1 - d - minD;
                    if (mem.disp2cost[_x2] > minS)
                    {
                        mem.disp2cost[_x2] = (CostType)minS;
                        mem.disp2ptr[_x2] = (DispType)(d + minD);
                    }

                    if (0 < d && d < D - 1)
                    {
                        // do subpixel quadratic interpolation:
                        //   fit parabola into (x1=d-1, y1=Sp[d-1]), (x2=d, y2=Sp[d]), (x3=d+1, y3=Sp[d+1])
                        //   then find minimum of the parabola.
                        int denom2 = max(Sp[d - 1] + Sp[d + 1] - 2 * Sp[d], 1);
                        d = d * DISP_SCALE + ((Sp[d - 1] - Sp[d + 1]) * DISP_SCALE + denom2) / (denom2 * 2);
                    }
                    else
                        d *= DISP_SCALE;
                    disp1ptr[x + minX1] = (DispType)(d + minD * DISP_SCALE);
                }

                for (x = minX1; x < maxX1; x++)
                {
                    // we round the computed disparity both towards -inf and +inf and check
                    // if either of the corresponding disparities in disp2 is consistent.
                    // This is to give the computed disparity a chance to look valid if it is.
                    int d1 = disp1ptr[x];
                    if (d1 == INVALID_DISP_SCALED)
                        continue;
                    int _d = d1 >> DISP_SHIFT;
                    int d_ = (d1 + DISP_SCALE - 1) >> DISP_SHIFT;
                    int _x = x - _d, x_ = x - d_;
                    if (0 <= _x && _x < width && mem.disp2ptr[_x] >= minD && std::abs(mem.disp2ptr[_x] - _d) > disp12MaxDiff &&
                        0 <= x_ && x_ < width && mem.disp2ptr[x_] >= minD && std::abs(mem.disp2ptr[x_] - d_) > disp12MaxDiff)
                        disp1ptr[x] = (DispType)INVALID_DISP_SCALED;
                }
            }

            lrID = 1 - lrID; // now shift the cyclic buffers
        }
    }
}

CV_EXPORTS InputOutputArray noArray__();

CV_EXPORTS_W void filterSpeckles__(InputOutputArray img, double newVal,
    int maxSpeckleSize, double maxDiff,
    InputOutputArray buf = noArray__());

void filterSpeckles__(InputOutputArray _img, double _newval, int maxSpeckleSize,
    double _maxDiff, InputOutputArray __buf)
{
    //CV_INSTRUMENT_REGION();

    Mat img = _img.getMat();
    int type = img.type();
    Mat temp, & _buf = __buf.needed() ? __buf.getMatRef() : temp;
    //CV_Assert(type == CV_8UC1 || type == CV_16SC1);

    int newVal = cvRound(_newval), maxDiff = cvRound(_maxDiff);

   /* CV_IPP_RUN_FAST(ipp_filterSpeckles(img, maxSpeckleSize, newVal, maxDiff, _buf));

    if (type == CV_8UC1)
        filterSpecklesImpl<uchar>(img, newVal, maxSpeckleSize, maxDiff, _buf);
    else
        filterSpecklesImpl<short>(img, newVal, maxSpeckleSize, maxDiff, _buf);*/
}


class StereoSGBMImpl__ CV_FINAL : public StereoSGBM__
{
public:
    StereoSGBMImpl__()
    {
        
        params = StereoSGBMParams__();
    }

    StereoSGBMImpl__(int _minDisparity, int _numDisparities, int _SADWindowSize,
        int _P1, int _P2, int _disp12MaxDiff, int _preFilterCap,
        int _uniquenessRatio, int _speckleWindowSize, int _speckleRange,
        int _mode)
    {cout << "StereoSGBMImpl__" << endl;
        params = StereoSGBMParams__(_minDisparity, _numDisparities, _SADWindowSize,
            _P1, _P2, _disp12MaxDiff, _preFilterCap,
            _uniquenessRatio, _speckleWindowSize, _speckleRange,
            _mode);
    }

    void compute(InputArray leftarr, InputArray rightarr, OutputArray disparr) CV_OVERRIDE
    {
        //CV_INSTRUMENT_REGION();

        Mat left = leftarr.getMat(), right = rightarr.getMat();
        //CV_Assert(left.size() == right.size() && left.type() == right.type() &&
            //left.depth() == CV_8U);

        disparr.create(left.size(), CV_16S);
        Mat disp = disparr.getMat();

        if (params.mode == MODE_SGBM)
            // the number of stripes is fixed, disregarding the number of threads/processors
            // to make the results fully reproducible
            //computeDisparity3WaySGBM<4>( left, right, disp, params );
      /*  else if(params.mode==MODE_HH4)
            computeDisparitySGBM_HH4( left, right, disp, params );*/
            //else
            computeDisparitySGBM__(left, right, disp, params);

        //medianBlur(disp, disp, 3);

        if (params.speckleWindowSize > 0)
            filterSpeckles__(disp, (params.minDisparity - 1) * StereoMatcher__::DISP_SCALE, params.speckleWindowSize,
                StereoMatcher__::DISP_SCALE * params.speckleRange, buffer);
    }

    int getMinDisparity() const CV_OVERRIDE { return params.minDisparity; }
    void setMinDisparity(int minDisparity) CV_OVERRIDE { params.minDisparity = minDisparity; }

    int getNumDisparities() const CV_OVERRIDE { return params.numDisparities; }
    void setNumDisparities(int numDisparities) CV_OVERRIDE { params.numDisparities = numDisparities; }

    int getBlockSize() const CV_OVERRIDE { return params.SADWindowSize; }
    void setBlockSize(int blockSize) CV_OVERRIDE { params.SADWindowSize = blockSize; }

    int getSpeckleWindowSize() const CV_OVERRIDE { return params.speckleWindowSize; }
    void setSpeckleWindowSize(int speckleWindowSize) CV_OVERRIDE { params.speckleWindowSize = speckleWindowSize; }

    int getSpeckleRange() const CV_OVERRIDE { return params.speckleRange; }
    void setSpeckleRange(int speckleRange) CV_OVERRIDE { params.speckleRange = speckleRange; }

    int getDisp12MaxDiff() const CV_OVERRIDE { return params.disp12MaxDiff; }
    void setDisp12MaxDiff(int disp12MaxDiff) CV_OVERRIDE { params.disp12MaxDiff = disp12MaxDiff; }

    int getPreFilterCap() const CV_OVERRIDE { return params.preFilterCap; }
    void setPreFilterCap(int preFilterCap) CV_OVERRIDE { params.preFilterCap = preFilterCap; }

    int getUniquenessRatio() const CV_OVERRIDE { return params.uniquenessRatio; }
    void setUniquenessRatio(int uniquenessRatio) CV_OVERRIDE { params.uniquenessRatio = uniquenessRatio; }

    int getP1() const CV_OVERRIDE { return params.P1; }
    void setP1(int P1) CV_OVERRIDE { params.P1 = P1; }

    int getP2() const CV_OVERRIDE { return params.P2; }
    void setP2(int P2) CV_OVERRIDE { params.P2 = P2; }

    int getMode() const CV_OVERRIDE { return params.mode; }
    void setMode(int mode) CV_OVERRIDE { params.mode = mode; }
    /*
    void write(FileStorage& fs) const CV_OVERRIDE
    {
        writeFormat(fs);
        fs << "name" << name_
            << "minDisparity" << params.minDisparity
            << "numDisparities" << params.numDisparities
            << "blockSize" << params.SADWindowSize
            << "speckleWindowSize" << params.speckleWindowSize
            << "speckleRange" << params.speckleRange
            << "disp12MaxDiff" << params.disp12MaxDiff
            << "preFilterCap" << params.preFilterCap
            << "uniquenessRatio" << params.uniquenessRatio
            << "P1" << params.P1
            << "P2" << params.P2
            << "mode" << params.mode;
    }

    void read(const FileNode& fn) CV_OVERRIDE
    {
        FileNode n = fn["name"];
        CV_Assert(n.isString() && String(n) == name_);
        params.minDisparity = (int)fn["minDisparity"];
        params.numDisparities = (int)fn["numDisparities"];
        params.SADWindowSize = (int)fn["blockSize"];
        params.speckleWindowSize = (int)fn["speckleWindowSize"];
        params.speckleRange = (int)fn["speckleRange"];
        params.disp12MaxDiff = (int)fn["disp12MaxDiff"];
        params.preFilterCap = (int)fn["preFilterCap"];
        params.uniquenessRatio = (int)fn["uniquenessRatio"];
        params.P1 = (int)fn["P1"];
        params.P2 = (int)fn["P2"];
        params.mode = (int)fn["mode"];
    }
    */
    StereoSGBMParams__ params;
    Mat buffer;

    static const char* name_;
};

const char* StereoSGBMImpl__::name_ = "StereoMatcher.SGBM";


//create-----------------------------------------------------------------------------//
Ptr<StereoSGBM__> StereoSGBM__::create(int minDisparity, int numDisparities, int SADWindowSize,
    int P1, int P2, int disp12MaxDiff,
    int preFilterCap, int uniquenessRatio,
    int speckleWindowSize, int speckleRange,
    int mode)
{
    cout << "SSSS" << endl;
    return Ptr<StereoSGBM__>(
        new StereoSGBMImpl__(minDisparity, numDisparities, SADWindowSize,
            P1, P2, disp12MaxDiff,
            preFilterCap, uniquenessRatio,
            speckleWindowSize, speckleRange,
            mode));
}


//----------------------------------------------------------------------------------------------------------------------------//

void Methods::read_calib(const char* filePath, cv::Mat* P0, cv::Mat* P1)
/*******************************************************************************
    Read the calibration file from 'filePath', and return projection matices
    of camera0 and camera1 as P0 and P1.

    Arguments:
        filePath -- file path of the calibration file
        P0 -- pointer to projection matrix of camera0
        P1 -- pointer to projection matrix of camera1

*******************************************************************************/
{
    FILE* fp;
    fopen_s(&fp, filePath, "r");
    char* next_token1 = NULL;
    char* next_token2 = NULL;

    *P0 = cv::Mat(3, 4, CV_32F);
    *P1 = cv::Mat(3, 4, CV_32F);

    if (!fp)
    {
        printf("Could not open the calibration file\n");
    }

    int count = 0;
    bool p;
    char content[1024];
    while (fgets(content, 1024, fp))
    {
        char* v = strtok_s(content, " ,", &next_token1);
        while (v)
        {
            if (--count > 0)
            {
                istringstream os(v);
                float d;
                os >> d;
                if (p)
                    P1->at<float>((12 - count) / 4, (12 - count) % 4) = d;
                else
                    P0->at<float>((12 - count) / 4, (12 - count) % 4) = d;
            }
            if (!strcmp(v, "P0:"))
            {
                count = 13;
                p = 0;
            }
            else if (!strcmp(v, "P1:"))
            {
                count = 13;
                p = 1;
            }
            v = strtok_s(NULL, " ,", &next_token1);
        }
    }

    fclose(fp);
}


vector<Mat> Methods::groundTruthTrajectory(const char* filePath, int data_num)
/*******************************************************************************
    Read the ground truth poses data from 'filePath', return Matrices of
    ground truth poses in a vector.

    Arguments:
        filePath -- file path of the poses data

    Return:
        poses -- a vector of poses in the form of matrix

*******************************************************************************/
{
    vector<Mat> poses;
    FILE* fp;
    fopen_s(&fp, filePath, "r");
    int cols = 12;
    for (int i = 0; i < data_num; i++)
    {
        Mat mat_i = Mat(3, 4, CV_32F);
        for (int j = 0; j < cols; j++)
        {
            fscanf_s(fp, "%e", &mat_i.at<float>(j / 4, j % 4));
        }
        poses.push_back(mat_i);
    }

    fclose(fp);
    return poses;
}


Mat Methods::computeLeftDisparityMap(Mat img_left, Mat img_right, int matcher_name, bool rgb)
/***************************************************************************************
    Takes a left and right stereo pair of images and computes the disparity
    map for the left image. Pass rgb = true if the images are RGB.

    Arguments:
        img_left -- image from left camera
        img_right -- image from right camera

    Optional Arguments:
        matcher -- (bool) can be 'BM' for StereoBM or 'SGBM' for StereoSGBM matching
        rgb -- (bool) set to true if passing RGB images as input

    Returns:
        disp_left -- disparity map for the left camera image

***************************************************************************************/

{
    // Feel free to read OpenCV documentationand tweak these values.These work well
    int sad_window = 6;
    int num_disparities = sad_window * 16;
    int block_size = 11;

    Ptr<StereoMatcher__> matcher;
    Mat disp_left;

    /*if (matcher_name == BM)
    {
        matcher = StereoBM::create(num_disparities, block_size);
    }
    else */ if (matcher_name == SGBM)
    {
        matcher = StereoSGBM__::create(0, num_disparities, block_size, 8 * 3 * pow(sad_window, 2), 32 * 3 * pow(sad_window, 2), 0, 0, 0, 0, 0, 0);
    }
    /*
    if (rgb)
    {
        cvtColor(img_left, img_left, COLOR_BGR2GRAY);
        cvtColor(img_right, img_right, COLOR_BGR2GRAY);
    }*/

    printf("\n\tComputing disparity map using Stereo%s...\n", (matcher_name == BM) ? "BM" : "SGBM");
    clock_t start = clock();
    /*if (matcher_name == BM)
    {
        matcher->compute(img_left, img_right, disp_left);
        disp_left.convertTo(disp_left, CV_32F, 1.0 / 16);
    }
    else */if (matcher_name == SGBM)
    {
        matcher->compute(img_left, img_right, disp_left);
        disp_left.convertTo(disp_left, CV_32F, 1.0 / 16);
    }

    clock_t end = clock();
    //printf("\tTime to compute disparity map using Stereo%s: %lld ms\n", (matcher_name == BM) ? "BM" : "SGBM", end - start);

    int x = 300, y = 1200;
    //printf("\ncompare with python tutorial, disp_left[%d, %d] = %f\n\n", x, y, disp_left.at<float>(x, y));

    return disp_left;
}



void Methods::decompose_Projection_Matrix(Mat p, Mat* k, Mat* r, Mat* t)
/***************************************************************************************
    Shortcut to use cv::decomposeProjectionMatrix(), which only returns k, r, t, and
    divides t by the scale, then returns them through pointers

    Arguments:
    p -- projection matrix to be decomposed

    Returns (call by address):
    k, r, t -- intrinsic matrix, rotation matrix, and 3D translation vector

***************************************************************************************/
{
    Mat rotMatrixX;
    Mat rotMatrixY;
    Mat rotMatrixZ;
    Mat eulerAngles;
    decomposeProjectionMatrix(p, *k, *r, *t);

    *t = *t / (t->at<float>(3));
}


Mat Methods::calc_depth_map(Mat disp_left, Mat k_left, Mat t_left, Mat t_right, bool rectified)
/***************************************************************************************
    Calculate depth map using a disparity map, intrinsic camera matrix, and translation
    vectors from camera extrinsic matrices(to calculate baseline).
    Note that default behavior is for rectified projection matrix for right camera.
    If using a regular projection matrix, pass rectified = false to avoid issues.

    Arguments:
        disp_left -- disparity map of left camera
        k_left -- intrinsic matrix for left camera
        t_left -- translation vector for left camera
        t_right -- translation vector for right camera
        rectified-- (bool)set to False if t_right is not from rectified projection
                    matrix

    Returns :
        depth_map -- calculated depth map for left camera

***************************************************************************************/
{
    Mat depth_map = Mat::ones(disp_left.rows, disp_left.cols, CV_32F);
    disp_left.convertTo(disp_left, CV_32F);

    // Get focal length of x axis for left camera
    float f = k_left.at<float>(0, 0);

    // Calculate baseline of stereo pair
    float b;
    if (rectified)
        b = t_right.at<float>(0, 0) - t_left.at<float>(0, 0);
    else
        b = t_left.at<float>(0, 0) - t_right.at<float>(0, 0);


    for (int i = 0; i < disp_left.rows; i++)
    {
        for (int j = 0; j < disp_left.cols; j++)
        {
            // Avoid instability and division by zero
            if (disp_left.at<float>(i, j) == 0.0 ||
                disp_left.at<float>(i, j) == -1.0)
                disp_left.at<float>(i, j) = 0.1;

            // Make empty depth map then fill with depth
            depth_map.at<float>(i, j) = f * b / disp_left.at<float>(i, j);
        }
    }

    return depth_map;
}


Mat Methods::stereo_2_depth(Mat img_left, Mat img_right, Mat P0, Mat P1, bool matcher, bool rgb, bool rectified)
/***************************************************************************************
    Takes stereo pair of images and returns a depth map for the left camera.If your
    projection matrices are not rectified, set rectified = false.

    Arguments:
        img_left -- image of left camera
        img_right -- image of right camera
        P0 -- Projection matrix for the left camera
        P1 -- Projection matrix for the right camera

    Optional Arguments :
        matcher-- (str)can be 'bm' for StereoBM or 'sgbm' for StereoSGBM
        rgb-- (bool)set to True if images passed are RGB.Default is False
        rectified-- (bool)set to False if P1 not rectified to P0.Default is True

    Returns :
        depth -- depth map for left camera

***************************************************************************************/
{
    Methods method;
    // Compute disparity map
    Mat disp = method.computeLeftDisparityMap(img_left, img_right, matcher, rgb);

    // Decompose projection matrices
    Mat k_left, r_left, t_left;
    Mat k_right, r_right, t_right;
    decompose_Projection_Matrix(P0, &k_left, &r_left, &t_left);
    decompose_Projection_Matrix(P1, &k_right, &r_right, &t_right);

    // Calculate depth map for left camera
    Mat depth = calc_depth_map(disp, k_left, t_left, t_right, true);

    return depth;
}



Mat Methods::extract_features(Mat image, int detector, Mat mask, vector<KeyPoint>* kp)
/***************************************************************************************
    Find keypoints and descriptors for the image

    Arguments :
        image -- a grayscale image
        detector-- (bool)can be 'Sift' or 'Orb'
        mask -- (Mat) mask to reduce feature search area to where depth information
                available.

    Returns :
        kp (call by address) -- list of the extracted keypoints(features) in an image
        des -- list of the keypoint descriptors in an image

***************************************************************************************/
{

    Ptr<Feature2D> det;
    Mat des;
    //Ptr<SURF> det_surf;

    if (detector == Sift)
    {
        det = SIFT::create();
        det->Feature2D::detect(image, *kp, mask);
        det->Feature2D::compute(image, *kp, des);

    }
    else if (detector == Orb)
    {
        det = ORB::create();
        det->Feature2D::detect(image, *kp, mask);
        det->Feature2D::compute(image, *kp, des);
        /*det = ORB::create();
        det->Feature2D::detectAndCompute(image, mask, *kp, des);*/
    }
    /*elif detector == 'surf' :
    det = cv2.xfeatures2d.SURF_create()*/


    return des;
}



vector<vector<DMatch>> Methods::match_features(Mat des1, Mat des2, bool matching, int detector, bool sorting, int k)
/***************************************************************************************
    Match features from two images

    Arguments :
        des1 -- list of the keypoint descriptors in the first image
        des2 -- list of the keypoint descriptors in the second image
        matching-- (bool)can be 'BF' for Brute Force or 'FLANN'
        detector-- (int)can be 'Sift or 'Orb'
        sort-- (bool)whether to sort matches by distance.Default is True
        k-- (int)number of neighbors to match to each feature.

    Returns:
        matches -- list of matched features from two images.Each match[i] is k or less
        matches for the same query descriptor
***************************************************************************************/
{
    BFMatcher matcher;
    vector<vector<DMatch>> matches;

    if (matching == BF)
    {
        if (detector == Sift)
            matcher.BFMatcher::create(NORM_L2, false);
        else if (detector == Orb)
            matcher.BFMatcher::create(NORM_HAMMING2, false);
        matcher.BFMatcher::knnMatch(des1, des2, matches, k);
    }

    return matches;
    /*else if (matching == FLANN)	// not finish yet
    {
        int FLANN_INDEX_KDTREE = 1;
        flann::Index Kdtree;
        Kdtree.build()

            index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
            search_params = dict(checks = 50)
            matcher = cv2.FlannBasedMatcher(index_params, search_params)
            matches = matcher.knnMatch(des1, des2, k = k)

    }
    */

    //if (sorting)
        //sort()
        //matches = sorted(matches, key = lambda x : x[0].distance);
}


void Methods::visualize_matches(Mat image1, vector<KeyPoint> kp1, Mat image2, vector<KeyPoint> kp2, vector<vector<DMatch>> match)
/***************************************************************************************
    Visualize corresponding matches in two images

    Arguments :
        image1 -- the first image in a matched image pair
        kp1 -- list of the keypoints in the first image
        image2 -- the second image in a matched image pair
        kp2 -- list of the keypoints in the second image
        match -- list of matched features from the pair of images

    Returns :
        image_matches -- an image showing the corresponding matches on both image1 and
        image2 or None if you don't use this function

***************************************************************************************/
{
    Mat image_matches;
    drawMatches(image1, kp1, image2, kp2, match, image_matches);
    imshow("image matches", image_matches);
    waitKey();
    destroyWindow("image matches");
    system("cls");
}


vector<vector<DMatch>> Methods::filter_matches_distance(vector<vector<DMatch>> matches, float dist_threshold)
/***************************************************************************************
    Filter matched features from two images by distance between the best matches

    Arguments :
        match -- list of matched features from two images
        dist_threshold -- maximum allowed relative distance between the best matches, (0.0, 1.0)

    Returns :
        filtered_match -- list of good matches, satisfying the distance threshold

***************************************************************************************/
{
    vector<vector<DMatch>> filtered_match;
    for (int m = 0; m < matches.size(); m++)
    {

        if (matches[m][0].distance <= dist_threshold * matches[m][1].distance)
        {
            vector<DMatch> match_i;
            match_i.push_back(matches[m][0]);
            filtered_match.push_back(match_i);
        }
    }
    return filtered_match;
}



void Methods::estimate_motion(vector<vector<DMatch>> match, vector<KeyPoint> kp1, vector<KeyPoint> kp2, Mat k, Mat depth1, int max_depth,
    Mat& rmat, Mat& tvec, Mat& image1_points, Mat& image2_points)
    /***************************************************************************************
        Estimate camera motion from a pair of subsequent image frames

        Arguments :
            match -- list of matched features from the pair of images
            kp1 -- list of the keypoints in the first image
            kp2 -- list of the keypoints in the second image
            k -- camera intrinsic calibration matrix
            depth1 -- Depth map of the first frame.Set to None to use Essential Matrix
                    decomposition
            max_depth -- Threshold of depth to ignore matched features. 3000 is default

        Returns (call by reference) :
            rmat -- estimated 3x3 rotation matrix
            tvec -- estimated 3x1 translation vector
            image1_points -- matched feature pixel coordinates in the first image.
            image1_points[i] = [u, v]->pixel coordinates of i - th match
            image2_points -- matched feature pixel coordinates in the second image.
            image2_points[i] = [u, v]->pixel coordinates of i - th match
    ***************************************************************************************/
{
    Mat image1_points__ = Mat(0, 2, CV_32F);
    image1_points = Mat(0, 2, CV_32F);
    Mat image2_points__ = Mat(0, 2, CV_32F);
    image2_points = Mat(0, 2, CV_32F);
    Mat rvec;
    Mat distCoef = Mat::zeros(1, 5, CV_32F);

    for (int m = 0; m < match.size(); m++)
    {
        image1_points__.push_back(kp1[match[m][0].queryIdx].pt);
        image2_points__.push_back(kp2[match[m][0].trainIdx].pt);
    }

    if (!depth1.empty())
    {
        float cx = k.at<float>(0, 2);
        float cy = k.at<float>(1, 2);
        float fx = k.at<float>(0, 0);
        float fy = k.at<float>(1, 1);
        Mat object_points = Mat::zeros(0, 3, CV_32F);

        for (int i = 0; i < image1_points__.rows; i++)
        {

            float u = image1_points__.at<float>(i, 0);
            float v = image1_points__.at<float>(i, 1);
            float z = depth1.at<float>((int)v, (int)u);

            if (z > max_depth)
            {
                continue;
            }


            float x = z * (u - cx) / fx;
            float y = z * (v - cy) / fy;


            Mat vec = Mat(1, 3, CV_32F);
            vec.at<float>(0, 0) = x;
            vec.at<float>(0, 1) = y;
            vec.at<float>(0, 2) = z;


            object_points.push_back(vec);
            image1_points.push_back(image1_points__.row(i));
            image2_points.push_back(image2_points__.row(i));
        }

        cv::solvePnPRansac(object_points, image2_points, k, distCoef, rvec, tvec,
            false, 100, 8.0, 0.99, noArray(), SOLVEPNP_ITERATIVE);


        rmat = Mat::eye(3, 3, CV_32F);
        Rodrigues(rvec, rmat);
    }
}


vector<Mat> Methods::visual_odometry(Dataset_Handler handler, int detector, bool matching,
    float filter_match_distance, bool stereo_matcher, int subset, Mat mask)
    /***************************************************************************************
        Function to perform visual odometry on a sequence from the KITTI visual odometry
        dataset.
        Takes as input a Dataset_Handler object and optional parameters.

        Arguments:
            handler -- Dataset_Handler object instance
            detector -- (str) can be 'Sift' or 'Orb'.
            matching -- (str) can be 'BF' for Brute Force or 'FLANN'.
            filter_match_distance -- (float) value for ratio test on matched features.
                                    Default is None.
            stereo_matcher -- (str) can be 'BM' (faster) or 'SGBM' (more accurate).
            mask -- (array) mask to reduce feature search area to where depth information
                        available.
            subset -- (int) number of frames to compute. Defaults to None to compute
                            all frames.

        Returns:
            trajectory -- Array of shape Nx3x4 of estimated poses of vehicle for each
                        computed frame.
    ***************************************************************************************/
{
    int num_frames;

    printf("Generating disparities with Stereo %s\n", stereo_matcher ? "SGBM" : "BM");
    printf("Detecting features with %s and matching with %s\n", (detector == Sift) ? "SIFT" : (detector == Orb) ? "ORB" : "SURF",
        matching ? "BF" : "FLANN");

    if (filter_match_distance)
        printf("Filtering feature matches at threshold of %f * distance\n", filter_match_distance);

    if (subset)
        num_frames = subset;
    else
        num_frames = handler.num_frames;

    Mat T_tot = Mat::eye(4, 4, CV_64F);


    vector<Mat> trajectory(num_frames);
    Rect rect(0, 0, 4, 3);
    T_tot(rect).copyTo(trajectory[0]);

    int imwidth = handler.imwidth;
    int imheight = handler.imheight;

    Mat k_left, r_left, t_left;
    decompose_Projection_Matrix(handler.P0, &k_left, &r_left, &t_left);

    Mat image_plus1 = imread(handler.left_image_files[0], IMREAD_GRAYSCALE);
    Mat image_left, image_right;
    Mat depth;

    for (int i = 0; i < num_frames - 1; i++)
    {
        printf("Computing frame %d\n", i + 1);
        clock_t start = clock();
        image_left = image_plus1;
        image_right = imread(handler.right_image_files[i], IMREAD_GRAYSCALE);
        image_plus1 = imread(handler.left_image_files[i + 1], IMREAD_GRAYSCALE);

        depth = stereo_2_depth(image_left, image_right, handler.P0, handler.P1, stereo_matcher, false, true);

        vector<KeyPoint> kp0, kp1;
        Mat des0 = extract_features(image_left, detector, mask, &kp0);
        Mat des1 = extract_features(image_plus1, detector, mask, &kp1);

        vector<vector<DMatch>> matches_unfilt = match_features(des0, des1, matching, detector, false, 2);

        vector<vector<DMatch>> matches;
        if (filter_match_distance)
            matches = filter_matches_distance(matches_unfilt, filter_match_distance);
        else
            matches = matches_unfilt;

        Mat rmat, tvec, img1_points, img2_points;
        estimate_motion(matches, kp0, kp1, k_left, depth, 3000, rmat, tvec, img1_points, img2_points);

        Mat T_mat;
        Mat I4 = Mat::eye(4, 4, CV_64F);
        hconcat(rmat, tvec, T_mat);
        vconcat(T_mat, I4.row(3), T_mat);
        T_tot = T_tot * T_mat.inv();
        T_tot(rect).copyTo(trajectory[i + 1]);
        clock_t end = clock();

        printf("Time to compute frame %d: %lld ms\n\n", i + 1, end - start);
    }

    return trajectory;
}

