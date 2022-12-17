#pragma once
#include <opencv2/opencv.hpp>

using namespace std;

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

    @param safe Enable safe operation mode, each allocation will be performed independently.
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
    @param alignment Alignment of allocated memory. same meaning as in the operator new (C++17).
                     Must be divisible by sizeof(T). Must be power of two.

    @note In safe mode allocation will be performed immediatly.
    */
    template <typename T>
    void allocate(T*& ptr, size_t count, ushort alignment = sizeof(T))
    {
        CV_Assert(ptr == NULL);
        CV_Assert(count > 0);
        CV_Assert(alignment > 0);
        CV_Assert(alignment % sizeof(T) == 0);
        CV_Assert((alignment & (alignment - 1)) == 0);
        allocate_((void**)(&ptr), static_cast<ushort>(sizeof(T)), count, alignment);
#ifndef OPENCV_ENABLE_MEMORY_SANITIZER
        if (safe)
#endif
            CV_Assert(ptr != NULL);
    }

    /** @brief Fill one of buffers with zeroes

    @param ptr pointer to memory block previously added using BufferArea::allocate

    BufferArea::commit must be called before using this method
    */
    template <typename T>
    void zeroFill(T*& ptr)
    {
        CV_Assert(ptr);
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
    to call allocate and commit again.
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
        CV_Assert(ptr && *ptr == NULL);
    }
    void cleanup() const
    {
        CV_Assert(ptr && *ptr);
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
        CV_Assert(ptr && *ptr == NULL);
        const size_t allocated_count = count + reserve_count();
        uchar* udata = (uchar*)malloc(allocated_count + sizeof(void*) + CV_MALLOC_ALIGN);
        cout << udata << endl;
        raw_mem = fastMalloc__(type_size * allocated_count);

        if (alignment != type_size)
        {
            *ptr = alignPtr__(raw_mem, alignment);
            CV_Assert(reinterpret_cast<size_t>(*ptr) % alignment == 0);
            CV_Assert(static_cast<uchar*>(*ptr) + type_size * count <= static_cast<uchar*>(raw_mem) + type_size * allocated_count);
        }
        else
        {
            *ptr = raw_mem;
        }
    }
#ifndef OPENCV_ENABLE_MEMORY_SANITIZER
    void* fast_allocate(void* buf) const
    {
        CV_Assert(ptr && *ptr == NULL);
        buf = alignPtr__(buf, alignment);
        CV_Assert(reinterpret_cast<size_t>(buf) % alignment == 0);
        *ptr = buf;
        return static_cast<void*>(static_cast<uchar*>(*ptr) + type_size * count);
    }
#endif
    bool operator==(void** other) const
    {
        CV_Assert(ptr && other);
        return *ptr == *other;
    }
    void zeroFill() const
    {
        CV_Assert(ptr && *ptr);
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
        CV_Assert(totalSize > 0);
        CV_Assert(oneBuf == NULL);
        CV_Assert(!blocks.empty());
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