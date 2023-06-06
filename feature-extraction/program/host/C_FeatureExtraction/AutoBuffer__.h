#pragma once
//#include <opencv2/opencv.hpp>
using namespace std;

#ifdef OPENCV_ENABLE_MEMORY_SANITIZER
template<typename _Tp, size_t fixed_size = 0> class AutoBuffer__
#else
template<typename _Tp, size_t fixed_size = 1024 / sizeof(_Tp) + 8> class AutoBuffer__
#endif
{
public:
    typedef _Tp value_type;

    //! the default constructor
    AutoBuffer__();
    //! constructor taking the real buffer size
    explicit AutoBuffer__(size_t _size);

    //! the copy constructor
    AutoBuffer__(const AutoBuffer__<_Tp, fixed_size>& buf);
    //! the assignment operator
    AutoBuffer__<_Tp, fixed_size>& operator = (const AutoBuffer__<_Tp, fixed_size>& buf);

    //! destructor. calls deallocate()
    ~AutoBuffer__();

    //! allocates the new buffer of size _size. if the _size is small enough, stack-allocated buffer is used
    void allocate(size_t _size);
    //! deallocates the buffer if it was dynamically allocated
    void deallocate();
    //! resizes the buffer and preserves the content
    void resize(size_t _size);
    //! returns the current buffer size
    size_t size() const;
    //! returns pointer to the real buffer, stack-allocated or heap-allocated
    inline _Tp* data() { return ptr; }
    //! returns read-only pointer to the real buffer, stack-allocated or heap-allocated
    inline const _Tp* data() const { return ptr; }

#if !defined(OPENCV_DISABLE_DEPRECATED_COMPATIBILITY) // use to .data() calls instead
    //! returns pointer to the real buffer, stack-allocated or heap-allocated
    operator _Tp* () { return ptr; }
    //! returns read-only pointer to the real buffer, stack-allocated or heap-allocated
    operator const _Tp* () const { return ptr; }
#else
    //! returns a reference to the element at specified location. No bounds checking is performed in Release builds.
    inline _Tp& operator[] (size_t i) { CV_DbgCheckLT(i, sz, "out of range"); return ptr[i]; }
    //! returns a reference to the element at specified location. No bounds checking is performed in Release builds.
    inline const _Tp& operator[] (size_t i) const { CV_DbgCheckLT(i, sz, "out of range"); return ptr[i]; }
#endif

protected:
    //! pointer to the real buffer, can point to buf if the buffer is small enough
    _Tp* ptr;
    //! size of the real buffer
    size_t sz;
    //! pre-allocated buffer. At least 1 element to confirm C++ standard requirements
    _Tp buf[(fixed_size > 0) ? fixed_size : 1];
};


template<typename _Tp, size_t fixed_size> inline void
AutoBuffer__<_Tp, fixed_size>::deallocate()
{
    if (ptr != buf)
    {
        delete[] ptr;
        ptr = buf;
        sz = fixed_size;
    }
}

template<typename _Tp, size_t fixed_size> inline
AutoBuffer__<_Tp, fixed_size>::~AutoBuffer__()
{
    deallocate();
}

template<typename _Tp, size_t fixed_size> inline void
AutoBuffer__<_Tp, fixed_size>::allocate(size_t _size)
{
    if (_size <= sz)
    {
        sz = _size;
        return;
    }
    deallocate();
    sz = _size;
    if (_size > fixed_size)
    {
        ptr = new _Tp[_size];
    }
}


template<typename _Tp, size_t fixed_size> inline
AutoBuffer__<_Tp, fixed_size>::AutoBuffer__(size_t _size)
{
    ptr = buf;
    sz = fixed_size;
    allocate(_size);
}