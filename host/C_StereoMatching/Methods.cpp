
# include <stdio.h>
# include <iostream>
# include <stdlib.h>
# include <vector>
#include <string.h>
#include <limits.h>
#include <math.h>
#include <algorithm>
#include "Mat.h"
#include "Methods.h"

using namespace std;




/*
typedef union __declspec(intrin_type) __declspec(align(16)) __m128i {
    __int8              m128i_i8[16];
    __int16             m128i_i16[8];
    __int32             m128i_i32[4];
    __int64             m128i_i64[2];
    //unsigned __int8     m128i_u8[16];
    //unsigned __int16    m128i_u16[8];
    //unsigned __int32    m128i_u32[4];
    //unsigned __int64    m128i_u64[2];
} __m128i;

extern int _mm_cvtsi128_si32(__m128i _A);
extern __m128i _mm_setr_epi16(short _W0, short _W1, short _W2, short _W3,
    short _W4, short _W5, short _W6, short _W7);

struct v_int16x8
{
    typedef short lane_type;
    typedef __m128i vector_type;
    enum { nlanes = 8 };

    v_int16x8() {}
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
};*/


#define CV_32F  5
#define CV_SIMD CV_SIMD128
#define CV_SIMD_64F CV_SIMD128_64F
#define CV_SIMD_WIDTH 16
typedef uchar PixType;
typedef short CostType;
typedef short DispType;
//typedef v_int16x8   v_int16;
typedef unsigned short ushort;
enum {
    DISP_SHIFT = 4,
    DISP_SCALE = (1 << DISP_SHIFT)
};

enum { NR = 8, NR2 = NR / 2 };

//-------------------------------------------------------------------------
//alignSize---------------------------------------------------------------------------//
static inline size_t alignSize__(size_t sz, int n)
{
    //CV_DbgAssert((n & (n - 1)) == 0); // n is a power of 2
    return (sz + n - 1) & -n;
}

static void calcPixelCostBT__(const Mat& img1, const Mat& img2, int y,
    int minD, int maxD, CostType* cost,
    PixType* buffer, const PixType* tab,
    int xrange_min = 0, int xrange_max = -1
)
{

    int x, c, width = img1.cols, cn = img1.channels();
    int minX1 = max(maxD, 0), maxX1 = width + min(minD, 0);
    int D = (int)alignSize__(maxD - minD, 8), width1 = maxX1 - minX1;
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

    int n1 = y > 0 ? -(int)img1.step[0] : 0, s1 = y < img1.rows - 1 ? (int)img1.step[0] : 0;
    int n2 = y > 0 ? -(int)img2.step[0] : 0, s2 = y < img2.rows - 1 ? (int)img2.step[0] : 0;

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



class  BufferArea__
{
public:

    BufferArea__(bool safe = false);

    ~BufferArea__();

    template <typename T>
    void allocate(T*& ptr, size_t count, ushort alignment = sizeof(T))
    {
        //CV_Assert(ptr == NULL);
        //CV_Assert(count > 0);
        //CV_Assert(alignment > 0);
        //CV_Assert(alignment % sizeof(T) == 0);
        //CV_Assert((alignment & (alignment - 1)) == 0);
        allocate_((void**)(&ptr), static_cast<ushort>(sizeof(T)), count, alignment);

    }

    template <typename T>
    void zeroFill(T*& ptr)
    {
        //CV_Assert(ptr);
        zeroFill_((void**)&ptr);
    }

    void zeroFill();

    void commit();

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


#endif


void fastFree__(void* ptr);

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
    //CV_DbgAssert((n & (n - 1)) == 0); // n is a power of 2
    return (_Tp*)(((size_t)ptr + n - 1) & -n);
}

#define  CV_MALLOC_ALIGN    64

void* fastMalloc__(size_t bufSize);


#ifdef OPENCV_ALLOC_ENABLE_STATISTICS
static inline
void* fastMalloc_(size_t size)
#else
void* fastMalloc__(size_t size)
#endif
{
    uchar* udata = (uchar*)malloc(size + sizeof(void*) + CV_MALLOC_ALIGN);
    //if (!udata)
        //return OutOfMemoryError(size);
    uchar** adata = alignPtr__((uchar**)udata + 1, CV_MALLOC_ALIGN);
    adata[-1] = udata;
    return adata;
}

//#pragma warning( disable : 4996 )
typedef int                           errno_t;

static inline const char* envRead(const char* name)
{

    cout << "UUUUUUUsing getenv";
    char* s;
    //s = getenv(name);
   // size_t len;
  //  errno_t err = _dupenv_s(&s, &len, name);
   // if (err) exit(-1);

    //return getenv(name);
    return s;
    //#endif
}

class ParseError
{
    std::string bad_value;
public:
    ParseError(const std::string bad_value_) :bad_value(bad_value_) {}

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
        cout << "read" << endl;
    }

    return defaultValue;
}

bool getConfigurationParameterBool(const char* name, bool defaultValue)
{
    cout << "getConfigurationParameterBool" << endl;
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
        //cout << "cleanup" << endl;
    }
    size_t getByteCount() const
    {
        //cout << "getByteCount" << endl;
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

//Range----------------------------------------------------------------------
class  Range__
{
public:

    Range__(int _start, int _end);
    int size() const;
    static Range__ all();

    int start, end;
};

inline
Range__::Range__(int _start, int _end)
    : start(_start), end(_end) {}

inline
int Range__::size() const
{
    return end - start;
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
        size_t height
    )
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
        fullDP = 0;
        costWidth = width1 * Da;
        costHeight = fullDP ? height : 1;
        int ps;
        ps = 11;
        hsumRows = ps + 2;//hsumRows = params.calcSADWindowSize().height + 2;
        dirs = NR;
        dirs2 = NR2;
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
        const int ftzero = 15 | 1;
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
    Mat& disp1)
{
    int minDisparity = 0;
    int numDisparities = 96;
    int SADWindowSize = 11;
    int P1 = 864;
    int P2 = 3456;
    int preFilterCap = 0;
    int uniquenessRatio = 0;
    int speckleWindowSize = 0;
    int speckleRange = 0;
    int mode = 0;

    const int DISP_SHIFT = 4;
    const int DISP_SCALE = (1 << DISP_SHIFT);
    const CostType MAX_COST = SHRT_MAX;

    int minD = minDisparity, maxD = minD + numDisparities;
    int disp12MaxDiff = 1;
    int k, width = disp1.cols, height = disp1.rows;
    int minX1 = max(maxD, 0), maxX1 = width + min(minD, 0);
    const int D = numDisparities;
    int width1 = maxX1 - minX1;
    int Da = (int)alignSize__(D, 8);
    int Dlra = Da + 8;//Additional memory is necessary to store disparity values(MAX_COST) for d=-1 and d=D
    int INVALID_DISP = minD - 1, INVALID_DISP_SCALED = INVALID_DISP * DISP_SCALE;
    int ps2;
    ps2 = SADWindowSize > 0 ? SADWindowSize : 5;
    int SW2 = ps2 / 2, SH2 = ps2 / 2;
    int npasses = 1;


    BufferSGBM__ mem(width1, Da, Dlra, img1.channels(), width, height);
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

                minL[0] = MAX_COST;
                minL[1] = MAX_COST;
                minL[2] = MAX_COST;
                minL[3] = MAX_COST;

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


Mat buffer;
void compute(Mat& left, Mat& right, Mat& disp) //CV_OVERRIDE
{
    int minDisparity = 0;
    int numDisparities = 96;
    int SADWindowSize = 11;
    int P1 = 864;
    int P2 = 3456;
    int disp12MaxDiff = 0;
    int preFilterCap = 0;
    int uniquenessRatio = 0;
    int speckleWindowSize = 0;
    int speckleRange = 0;
    int mode = 0;

    //CV_INSTRUMENT_REGION();

    //CV_Assert(left.size() == right.size() && left.type() == right.type() &&
        //left.depth() == CV_8U);

    //disp.create(left.size(), CV_16S);
    //outmat(disp);
    //Mat disp;
    disp.create_disp();//project3

    computeDisparitySGBM__(left, right, disp);

    //medianBlur(disp, disp, 3);

}


Mat computeLeftDisparityMap(Mat img_left, Mat img_right, int matcher_name, bool rgb)
{
    // Feel free to read OpenCV documentationand tweak these values.These work well
    int sad_window = 6;
    int num_disparities = sad_window * 16;
    int block_size = 11;

    Mat disp_left;
    Mat new_disp;

    printf("\n\tComputing disparity map using Stereo%s...\n", (matcher_name == BM) ? "BM" : "SGBM");
    clock_t start = clock();
    if (matcher_name == SGBM)
    {
        compute(img_left, img_right, disp_left);

        new_disp.cols = disp_left.cols;
        new_disp.rows = disp_left.rows;
        new_disp.step[0] = 4964;
        new_disp.step[1] = 4;
        //new_disp.size = disp_left.size;
        new_disp.flags = 1124024325;
        //new_disp.dims = disp_left.dims;

        static unsigned char tempch[4*376*1241];
        new_disp.data = tempch;

        short temp;
        float temp2;

        for (int i = 0; i < new_disp.rows; i++)
        {
            for (int j = 0; j < new_disp.cols; j++)
            {
                temp = disp_left.at<short>(i, j);
                temp2 = static_cast<float>(temp) / 16;
                new_disp.at<float>(i, j) = temp2;
            }
        }
        //outmat(new_disp);


    }

    clock_t end = clock();
    //printf("\tTime to compute disparity map using Stereo%s: %lld ms\n", (matcher_name == BM) ? "BM" : "SGBM", end - start);

    int x = 300, y = 1200;
    //printf("\ncompare with python tutorial, disp_left[%d, %d] = %f\n\n", x, y, disp_left.at<float>(x, y));

    return new_disp;
}


Mat calc_depth_map(Mat disp_left, Mat k_left, Mat t_left, Mat t_right, bool rectified)
{
    //Mat depth_map = Mat::ones(disp_left.rows, disp_left.cols, CV_32F);
    Mat depth_map;
    depth_map.create_ones();

    //Mat depth_map = Mat(disp_left.rows, disp_left.cols, CV_32F, Scalar(1, 0, 0));
    //Mat depth_map = Mat(disp_left.rows, disp_left.cols, CV_32F, 1);
    //disp_left.convertTo(disp_left, CV_32F);

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


void stereo_2_depth(unsigned char* image_left_test_data, unsigned char* image_right_test_data, unsigned char* k_left_test_data, unsigned char* t_left_test_data, unsigned char* t_right_test_data, bool matcher, bool rgb, bool rectified, unsigned char* depth_map)

{
	Mat img_left, img_right, k_left, t_left, t_right, depth;
	Mat disp;

	img_left.flags = 1124024320;
	img_left.dims = 2;
	img_left.step[0] = 1241;
	img_left.step[1] = 1;
	img_left.rows = 376;
	img_left.cols = 1241;
	img_left.data = image_left_test_data;

	img_right.flags = 1124024320;
	img_right.dims = 2;
	img_right.step[0] = 1241;
	img_right.step[1] = 1;
	img_right.rows = 376;
	img_right.cols = 1241;
	img_right.data = image_right_test_data;

	k_left.flags = 1124024325;
	k_left.dims = 2;
	k_left.step[0] = 12;
	k_left.step[1] = 4;
	k_left.rows = 3;
	k_left.cols = 3;
	k_left.data = k_left_test_data;

	t_left.flags = 1124024320;
	t_left.dims = 2;
	t_left.step[0] = 4;
	t_left.step[1] = 4;
	t_left.rows = 4;
	t_left.cols = 1;
	t_left.data = t_left_test_data;

	t_right.flags = 1124024325;
	t_right.dims = 2;
	t_right.step[0] = 4;
	t_right.step[1] = 4;
	t_right.rows = 4;
	t_right.cols = 1;
	t_right.data = t_right_test_data;

    //Methods method;
    // Compute disparity map
     disp = computeLeftDisparityMap(img_left, img_right, SGBM, 0);

    // Decompose projection matrices
    //Mat k_left, r_left, t_left;
    //Mat k_right, r_right, t_right;
    //decompose_Projection_Matrix(P0, &k_left, &r_left, &t_left);
    //decompose_Projection_Matrix(P1, &k_right, &r_right, &t_right);

    // Calculate depth map for left camera

     depth = calc_depth_map(disp, k_left, t_left, t_right, true);

    for (int k = 0; k < 4*376*1241; k++)
    {
    	depth_map[k] = depth.data[k];

    }


    cout<<"d: "<<depth.data[0]<<" "<<static_cast<int>(depth_map[0])<<endl;

    //return depth;
}

