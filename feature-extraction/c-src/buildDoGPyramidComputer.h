#pragma once
//#include <opencv2/opencv.hpp>
using namespace std;

#define GET_OPTIMIZED(func) (func)



typedef void (*BinaryFuncC)(const uchar* src1, size_t step1,
    const uchar* src2, size_t step2,
    uchar* dst, size_t step, int width, int height,
    void*);

typedef void (*BinaryFunc)(const uchar* src1, size_t step1,
    const uchar* src2, size_t step2,
    uchar* dst, size_t step, Size sz,
    void*);


enum {
    OCL_OP_ADD = 0, OCL_OP_SUB = 1, OCL_OP_RSUB = 2, OCL_OP_ABSDIFF = 3, OCL_OP_MUL = 4,
    OCL_OP_MUL_SCALE = 5, OCL_OP_DIV_SCALE = 6, OCL_OP_RECIP_SCALE = 7, OCL_OP_ADDW = 8,
    OCL_OP_AND = 9, OCL_OP_OR = 10, OCL_OP_XOR = 11, OCL_OP_NOT = 12, OCL_OP_MIN = 13, OCL_OP_MAX = 14,
    OCL_OP_RDIV_SCALE = 15
};

enum { BLOCK_SIZE = 1024 };



/*#define cv_hal_sub8u hal_ni_sub8u
#define cv_hal_sub8s hal_ni_sub8s
#define cv_hal_sub16u hal_ni_sub16u
#define cv_hal_sub16s hal_ni_sub16s
#define cv_hal_sub32s hal_ni_sub32s
#define cv_hal_sub32f hal_ni_sub32f
#define cv_hal_sub64f hal_ni_sub64f

inline int hal_ni_sub8u(const uchar* src1_data, size_t src1_step, const uchar* src2_data, size_t src2_step, uchar* dst_data, size_t dst_step, int width, int height) { return CV_HAL_ERROR_NOT_IMPLEMENTED; }
inline int hal_ni_sub8s(const schar* src1_data, size_t src1_step, const schar* src2_data, size_t src2_step, schar* dst_data, size_t dst_step, int width, int height) { return CV_HAL_ERROR_NOT_IMPLEMENTED; }
inline int hal_ni_sub16u(const ushort* src1_data, size_t src1_step, const ushort* src2_data, size_t src2_step, ushort* dst_data, size_t dst_step, int width, int height) { return CV_HAL_ERROR_NOT_IMPLEMENTED; }
inline int hal_ni_sub16s(const short* src1_data, size_t src1_step, const short* src2_data, size_t src2_step, short* dst_data, size_t dst_step, int width, int height) { return CV_HAL_ERROR_NOT_IMPLEMENTED; }
inline int hal_ni_sub32s(const int* src1_data, size_t src1_step, const int* src2_data, size_t src2_step, int* dst_data, size_t dst_step, int width, int height) { return CV_HAL_ERROR_NOT_IMPLEMENTED; }
inline int hal_ni_sub32f(const float* src1_data, size_t src1_step, const float* src2_data, size_t src2_step, float* dst_data, size_t dst_step, int width, int height) { return CV_HAL_ERROR_NOT_IMPLEMENTED; }
inline int hal_ni_sub64f(const double* src1_data, size_t src1_step, const double* src2_data, size_t src2_step, double* dst_data, size_t dst_step, int width, int height) { return CV_HAL_ERROR_NOT_IMPLEMENTED; }
*/
template<typename T1, typename T2 = T1, typename T3 = T1> struct OpSub
{
    typedef T1 type1;
    typedef T2 type2;
    typedef T3 rtype;
    T3 operator ()(const T1 a, const T2 b) const { return saturate_cast<T3>(a - b); }
};

template < typename T, class  Op /*, class  VOp*/ >
void  vBinOp(const  T* src1, size_t  step1, const  T* src2, size_t  step2, T* dst, size_t  step, int  width, int  height)
{
/*# if CV_SSE2 || CV_NEON
    VOp vop;
# endif*/
    Op op;

    for (; height--; src1 = (const T*)((const uchar*)src1 + step1),
        src2 = (const T*)((const uchar*)src2 + step2),
        dst = (T*)((uchar*)dst + step))
    {
        int x = 0;

        /*
        # if CV_NEON || CV_SSE2
        # if CV_AVX2 if ( USE_AVX2 )
                { for ( ; x <= width - 32 /( int ) sizeof (T); x += 32 / sizeof (T) )
                { typename VLoadStore256<T>::reg_type r0 = VLoadStore256<T>::load(src1 + x);
                r0 = vop(r0, VLoadStore256<T>::load(src2 + x));
                VLoadStore256<T>::store(dst + x, r0);
            }
            }
        # else
        # if CV_SSE2 if ( USE_SSE2 ) {
        # endif  // CV_SSE2
                for ( ; x <= width - 32 /( int ) sizeof (T); x += 32 / sizeof (T) )
                { typename VLoadStore128<T>::reg_type r0 = VLoadStore128<T>::load(src1 + x );
                typename VLoadStore128<T>::reg_type r1 = VLoadStore128<T>::load(src1 + x + 16 / sizeof (T));
                r0 = vop(r0, VLoadStore128<T>::load(src2 + x ));
                r1 = vop(r1, VLoadStore128<T>::load(src2 + x + 16 / sizeof (T)));


                 VLoadStore128<T>::store(dst + x, r0);
                VLoadStore128<T>::store(dst + x + 16 / sizeof(T), r1);
                }
        # if CV_SSE2
            }
        # endif  // CV_SSE2
        # endif  // CV_AVX2
        # endif  // CV_NEON || CV_SSE2

        # if CV_AVX2 // nothing
        # elif CV_SSE2 if ( USE_SSE2 )
                { for ( ; x <= width - 8 /( int ) sizeof (T); x += 8 / sizeof (T) )
                { typename VLoadStore64<T>::reg_type r = VLoadStore64<T>::load(src1 + x);
                r = vop(r, VLoadStore64<T>::load(src2 + x));
                VLoadStore64<T>::store(dst + x, r);
                }
                }
        # endif






        # if CV_ENABLE_UNROLLED for ( ; x <= width - 4 ; x += 4 )
                {             T v0 = op(src1[x], src2[x]);
                T v1 = op(src1[x+ 1 ], src2[x+ 1 ]);
                dst[x] = v0; dst[x+ 1 ] = v1;
                v0 = op(src1[x+ 2 ], src2[x+ 2 ]);
                v1 = op(src1[x+ 3 ], src2[x+ 3 ]);
                dst[x+ 2 ] = v0; dst[x+ 3 ] = v1;
                }
        # endif


        */








        for (; x < width; x++)
            dst[x] = op(src1[x], src2[x]);
    }
}

//CV_EXPORTS void sub8u(const uchar* src1, size_t step1, const uchar* src2, size_t step2, uchar* dst, size_t step, int width, int height, void*);
void sub8u(const uchar* src1, size_t step1,
    const uchar* src2, size_t step2,
    uchar* dst, size_t step, int width, int height, void*)
{
   // CALL_HAL(sub8u, cv_hal_sub8u, src1, step1, src2, step2, dst, step, width, height);
       // CALL_IPP_BIN_E_21(ippiSub_8u_C1RSfs)
        (vBinOp<uchar, OpSub<uchar>>(src1, step1, src2, step2, dst,
            step, width, height));
}

/*CV_EXPORTS void sub8s(const schar* src1, size_t step1, const schar* src2, size_t step2, schar* dst, size_t step, int width, int height, void*) { CALL_HAL(sub8s, cv_hal_sub8s, src1, step1, src2, step2, dst, step, width, height); }
CV_EXPORTS void sub16u(const ushort* src1, size_t step1, const ushort* src2, size_t step2, ushort* dst, size_t step, int width, int height, void*) { CALL_HAL(sub16u, cv_hal_sub16u, src1, step1, src2, step2, dst, step, width, height); }
CV_EXPORTS void sub16s(const short* src1, size_t step1, const short* src2, size_t step2, short* dst, size_t step, int width, int height, void*) { CALL_HAL(sub16s, cv_hal_sub16s, src1, step1, src2, step2, dst, step, width, height); }
CV_EXPORTS void sub32s(const int* src1, size_t step1, const int* src2, size_t step2, int* dst, size_t step, int width, int height, void*) { CALL_HAL(sub32s, cv_hal_sub32s, src1, step1, src2, step2, dst, step, width, height); }
*/
CV_EXPORTS void sub32f(const float* src1, size_t step1, const float* src2, size_t step2, float* dst, size_t step, int width, int height, void*) 
{
    (vBinOp<float, OpSub<float>>(src1, step1, src2, step2, dst,
        step, width, height));
} 
//{ CALL_HAL(sub32f, cv_hal_sub32f, src1, step1, src2, step2, dst, step, width, height); }
//CV_EXPORTS void sub64f(const double* src1, size_t step1, const double* src2, size_t step2, double* dst, size_t step, int width, int height, void*) { CALL_HAL(sub64f, cv_hal_sub64f, src1, step1, src2, step2, dst, step, width, height); }


inline int _v_cvtsi512_si32(const __m512i& a)
{
    return _mm_cvtsi128_si32(_mm512_castsi512_si128(a));
}

#define _v512_set_epu64(a7, a6, a5, a4, a3, a2, a1, a0) _mm512_set_epi64((int64)(a7),(int64)(a6),(int64)(a5),(int64)(a4),(int64)(a3),(int64)(a2),(int64)(a1),(int64)(a0))
#define _v512_set_epu32(a15, a14, a13, a12, a11, a10,  a9,  a8,  a7,  a6,  a5,  a4,  a3,  a2,  a1,  a0) \
        _mm512_set_epi64(((int64)(a15)<<32)|(int64)(a14), ((int64)(a13)<<32)|(int64)(a12), ((int64)(a11)<<32)|(int64)(a10), ((int64)( a9)<<32)|(int64)( a8), \
                         ((int64)( a7)<<32)|(int64)( a6), ((int64)( a5)<<32)|(int64)( a4), ((int64)( a3)<<32)|(int64)( a2), ((int64)( a1)<<32)|(int64)( a0))

#define _v512_set_epu16(a31, a30, a29, a28, a27, a26, a25, a24, a23, a22, a21, a20, a19, a18, a17, a16, \
                        a15, a14, a13, a12, a11, a10,  a9,  a8,  a7,  a6,  a5,  a4,  a3,  a2,  a1,  a0) \
        _v512_set_epu32(((unsigned)(a31)<<16)|(unsigned)(a30), ((unsigned)(a29)<<16)|(unsigned)(a28), ((unsigned)(a27)<<16)|(unsigned)(a26), ((unsigned)(a25)<<16)|(unsigned)(a24), \
                        ((unsigned)(a23)<<16)|(unsigned)(a22), ((unsigned)(a21)<<16)|(unsigned)(a20), ((unsigned)(a19)<<16)|(unsigned)(a18), ((unsigned)(a17)<<16)|(unsigned)(a16), \
                        ((unsigned)(a15)<<16)|(unsigned)(a14), ((unsigned)(a13)<<16)|(unsigned)(a12), ((unsigned)(a11)<<16)|(unsigned)(a10), ((unsigned)( a9)<<16)|(unsigned)( a8), \
                        ((unsigned)( a7)<<16)|(unsigned)( a6), ((unsigned)( a5)<<16)|(unsigned)( a4), ((unsigned)( a3)<<16)|(unsigned)( a2), ((unsigned)( a1)<<16)|(unsigned)( a0))
#define _v512_set_epu8(a63, a62, a61, a60, a59, a58, a57, a56, a55, a54, a53, a52, a51, a50, a49, a48, \
                       a47, a46, a45, a44, a43, a42, a41, a40, a39, a38, a37, a36, a35, a34, a33, a32, \
                       a31, a30, a29, a28, a27, a26, a25, a24, a23, a22, a21, a20, a19, a18, a17, a16, \
                       a15, a14, a13, a12, a11, a10,  a9,  a8,  a7,  a6,  a5,  a4,  a3,  a2,  a1,  a0) \
        _v512_set_epu32(((unsigned)(a63)<<24)|((unsigned)(a62)<<16)|((unsigned)(a61)<<8)|(unsigned)(a60),((unsigned)(a59)<<24)|((unsigned)(a58)<<16)|((unsigned)(a57)<<8)|(unsigned)(a56), \
                        ((unsigned)(a55)<<24)|((unsigned)(a54)<<16)|((unsigned)(a53)<<8)|(unsigned)(a52),((unsigned)(a51)<<24)|((unsigned)(a50)<<16)|((unsigned)(a49)<<8)|(unsigned)(a48), \
                        ((unsigned)(a47)<<24)|((unsigned)(a46)<<16)|((unsigned)(a45)<<8)|(unsigned)(a44),((unsigned)(a43)<<24)|((unsigned)(a42)<<16)|((unsigned)(a41)<<8)|(unsigned)(a40), \
                        ((unsigned)(a39)<<24)|((unsigned)(a38)<<16)|((unsigned)(a37)<<8)|(unsigned)(a36),((unsigned)(a35)<<24)|((unsigned)(a34)<<16)|((unsigned)(a33)<<8)|(unsigned)(a32), \
                        ((unsigned)(a31)<<24)|((unsigned)(a30)<<16)|((unsigned)(a29)<<8)|(unsigned)(a28),((unsigned)(a27)<<24)|((unsigned)(a26)<<16)|((unsigned)(a25)<<8)|(unsigned)(a24), \
                        ((unsigned)(a23)<<24)|((unsigned)(a22)<<16)|((unsigned)(a21)<<8)|(unsigned)(a20),((unsigned)(a19)<<24)|((unsigned)(a18)<<16)|((unsigned)(a17)<<8)|(unsigned)(a16), \
                        ((unsigned)(a15)<<24)|((unsigned)(a14)<<16)|((unsigned)(a13)<<8)|(unsigned)(a12),((unsigned)(a11)<<24)|((unsigned)(a10)<<16)|((unsigned)( a9)<<8)|(unsigned)( a8), \
                        ((unsigned)( a7)<<24)|((unsigned)( a6)<<16)|((unsigned)( a5)<<8)|(unsigned)( a4),((unsigned)( a3)<<24)|((unsigned)( a2)<<16)|((unsigned)( a1)<<8)|(unsigned)( a0))
#define _v512_set_epi8(a63, a62, a61, a60, a59, a58, a57, a56, a55, a54, a53, a52, a51, a50, a49, a48, \
                       a47, a46, a45, a44, a43, a42, a41, a40, a39, a38, a37, a36, a35, a34, a33, a32, \
                       a31, a30, a29, a28, a27, a26, a25, a24, a23, a22, a21, a20, a19, a18, a17, a16, \
                       a15, a14, a13, a12, a11, a10,  a9,  a8,  a7,  a6,  a5,  a4,  a3,  a2,  a1,  a0) \
        _v512_set_epu8((uchar)(a63), (uchar)(a62), (uchar)(a61), (uchar)(a60), (uchar)(a59), (uchar)(a58), (uchar)(a57), (uchar)(a56), \
                       (uchar)(a55), (uchar)(a54), (uchar)(a53), (uchar)(a52), (uchar)(a51), (uchar)(a50), (uchar)(a49), (uchar)(a48), \
                       (uchar)(a47), (uchar)(a46), (uchar)(a45), (uchar)(a44), (uchar)(a43), (uchar)(a42), (uchar)(a41), (uchar)(a40), \
                       (uchar)(a39), (uchar)(a38), (uchar)(a37), (uchar)(a36), (uchar)(a35), (uchar)(a34), (uchar)(a33), (uchar)(a32), \
                       (uchar)(a31), (uchar)(a30), (uchar)(a29), (uchar)(a28), (uchar)(a27), (uchar)(a26), (uchar)(a25), (uchar)(a24), \
                       (uchar)(a23), (uchar)(a22), (uchar)(a21), (uchar)(a20), (uchar)(a19), (uchar)(a18), (uchar)(a17), (uchar)(a16), \
                       (uchar)(a15), (uchar)(a14), (uchar)(a13), (uchar)(a12), (uchar)(a11), (uchar)(a10), (uchar)( a9), (uchar)( a8), \
                       (uchar)( a7), (uchar)( a6), (uchar)( a5), (uchar)( a4), (uchar)( a3), (uchar)( a2), (uchar)( a1), (uchar)( a0))


struct v_uint8x64
{
    typedef uchar lane_type;
    enum { nlanes = 64 };
    __m512i val;

    explicit v_uint8x64(__m512i v) : val(v) {}
    v_uint8x64(uchar v0, uchar v1, uchar v2, uchar v3,
        uchar v4, uchar v5, uchar v6, uchar v7,
        uchar v8, uchar v9, uchar v10, uchar v11,
        uchar v12, uchar v13, uchar v14, uchar v15,
        uchar v16, uchar v17, uchar v18, uchar v19,
        uchar v20, uchar v21, uchar v22, uchar v23,
        uchar v24, uchar v25, uchar v26, uchar v27,
        uchar v28, uchar v29, uchar v30, uchar v31,
        uchar v32, uchar v33, uchar v34, uchar v35,
        uchar v36, uchar v37, uchar v38, uchar v39,
        uchar v40, uchar v41, uchar v42, uchar v43,
        uchar v44, uchar v45, uchar v46, uchar v47,
        uchar v48, uchar v49, uchar v50, uchar v51,
        uchar v52, uchar v53, uchar v54, uchar v55,
        uchar v56, uchar v57, uchar v58, uchar v59,
        uchar v60, uchar v61, uchar v62, uchar v63)
    {
        val = _v512_set_epu8(v63, v62, v61, v60, v59, v58, v57, v56, v55, v54, v53, v52, v51, v50, v49, v48,
            v47, v46, v45, v44, v43, v42, v41, v40, v39, v38, v37, v36, v35, v34, v33, v32,
            v31, v30, v29, v28, v27, v26, v25, v24, v23, v22, v21, v20, v19, v18, v17, v16,
            v15, v14, v13, v12, v11, v10, v9, v8, v7, v6, v5, v4, v3, v2, v1, v0);
    }
    v_uint8x64() {}

    static inline v_uint8x64 zero() { return v_uint8x64(_mm512_setzero_si512()); }

    uchar get0() const { return (uchar)_v_cvtsi512_si32(val); }
};

struct v_int8x64
{
    typedef schar lane_type;
    enum { nlanes = 64 };
    __m512i val;

    explicit v_int8x64(__m512i v) : val(v) {}
    v_int8x64(schar v0, schar v1, schar v2, schar v3,
        schar v4, schar v5, schar v6, schar v7,
        schar v8, schar v9, schar v10, schar v11,
        schar v12, schar v13, schar v14, schar v15,
        schar v16, schar v17, schar v18, schar v19,
        schar v20, schar v21, schar v22, schar v23,
        schar v24, schar v25, schar v26, schar v27,
        schar v28, schar v29, schar v30, schar v31,
        schar v32, schar v33, schar v34, schar v35,
        schar v36, schar v37, schar v38, schar v39,
        schar v40, schar v41, schar v42, schar v43,
        schar v44, schar v45, schar v46, schar v47,
        schar v48, schar v49, schar v50, schar v51,
        schar v52, schar v53, schar v54, schar v55,
        schar v56, schar v57, schar v58, schar v59,
        schar v60, schar v61, schar v62, schar v63)
    {
        val = _v512_set_epi8(v63, v62, v61, v60, v59, v58, v57, v56, v55, v54, v53, v52, v51, v50, v49, v48,
            v47, v46, v45, v44, v43, v42, v41, v40, v39, v38, v37, v36, v35, v34, v33, v32,
            v31, v30, v29, v28, v27, v26, v25, v24, v23, v22, v21, v20, v19, v18, v17, v16,
            v15, v14, v13, v12, v11, v10, v9, v8, v7, v6, v5, v4, v3, v2, v1, v0);
    }
    v_int8x64() {}

    static inline v_int8x64 zero() { return v_int8x64(_mm512_setzero_si512()); }

    schar get0() const { return (schar)_v_cvtsi512_si32(val); }
};

struct v_uint16x32
{
    typedef ushort lane_type;
    enum { nlanes = 32 };
    __m512i val;

    explicit v_uint16x32(__m512i v) : val(v) {}
    v_uint16x32(ushort v0, ushort v1, ushort v2, ushort v3,
        ushort v4, ushort v5, ushort v6, ushort v7,
        ushort v8, ushort v9, ushort v10, ushort v11,
        ushort v12, ushort v13, ushort v14, ushort v15,
        ushort v16, ushort v17, ushort v18, ushort v19,
        ushort v20, ushort v21, ushort v22, ushort v23,
        ushort v24, ushort v25, ushort v26, ushort v27,
        ushort v28, ushort v29, ushort v30, ushort v31)
    {
        val = _v512_set_epu16(v31, v30, v29, v28, v27, v26, v25, v24, v23, v22, v21, v20, v19, v18, v17, v16,
            v15, v14, v13, v12, v11, v10, v9, v8, v7, v6, v5, v4, v3, v2, v1, v0);
    }
    v_uint16x32() {}

    static inline v_uint16x32 zero() { return v_uint16x32(_mm512_setzero_si512()); }

    ushort get0() const { return (ushort)_v_cvtsi512_si32(val); }
};

struct v_int16x32
{
    typedef short lane_type;
    enum { nlanes = 32 };
    __m512i val;

    explicit v_int16x32(__m512i v) : val(v) {}
    v_int16x32(short v0, short v1, short v2, short v3, short v4, short v5, short v6, short v7,
        short v8, short v9, short v10, short v11, short v12, short v13, short v14, short v15,
        short v16, short v17, short v18, short v19, short v20, short v21, short v22, short v23,
        short v24, short v25, short v26, short v27, short v28, short v29, short v30, short v31)
    {
        val = _v512_set_epu16((ushort)v31, (ushort)v30, (ushort)v29, (ushort)v28, (ushort)v27, (ushort)v26, (ushort)v25, (ushort)v24,
            (ushort)v23, (ushort)v22, (ushort)v21, (ushort)v20, (ushort)v19, (ushort)v18, (ushort)v17, (ushort)v16,
            (ushort)v15, (ushort)v14, (ushort)v13, (ushort)v12, (ushort)v11, (ushort)v10, (ushort)v9, (ushort)v8,
            (ushort)v7, (ushort)v6, (ushort)v5, (ushort)v4, (ushort)v3, (ushort)v2, (ushort)v1, (ushort)v0);
    }
    v_int16x32() {}

    static inline v_int16x32 zero() { return v_int16x32(_mm512_setzero_si512()); }

    short get0() const { return (short)_v_cvtsi512_si32(val); }
};

struct v_uint32x16
{
    typedef unsigned lane_type;
    enum { nlanes = 16 };
    __m512i val;

    explicit v_uint32x16(__m512i v) : val(v) {}
    v_uint32x16(unsigned v0, unsigned v1, unsigned v2, unsigned v3,
        unsigned v4, unsigned v5, unsigned v6, unsigned v7,
        unsigned v8, unsigned v9, unsigned v10, unsigned v11,
        unsigned v12, unsigned v13, unsigned v14, unsigned v15)
    {
        val = _mm512_setr_epi32((int)v0, (int)v1, (int)v2, (int)v3, (int)v4, (int)v5, (int)v6, (int)v7,
            (int)v8, (int)v9, (int)v10, (int)v11, (int)v12, (int)v13, (int)v14, (int)v15);
    }
    v_uint32x16() {}

    static inline v_uint32x16 zero() { return v_uint32x16(_mm512_setzero_si512()); }

    unsigned get0() const { return (unsigned)_v_cvtsi512_si32(val); }
};

struct v_int32x16
{
    typedef int lane_type;
    enum { nlanes = 16 };
    __m512i val;

    explicit v_int32x16(__m512i v) : val(v) {}
    v_int32x16(int v0, int v1, int v2, int v3, int v4, int v5, int v6, int v7,
        int v8, int v9, int v10, int v11, int v12, int v13, int v14, int v15)
    {
        val = _mm512_setr_epi32(v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15);
    }
    v_int32x16() {}

    static inline v_int32x16 zero() { return v_int32x16(_mm512_setzero_si512()); }

    int get0() const { return _v_cvtsi512_si32(val); }
};

struct v_float32x16
{
    typedef float lane_type;
    enum { nlanes = 16 };
    __m512 val;

    explicit v_float32x16(__m512 v) : val(v) {}
    v_float32x16(float v0, float v1, float v2, float v3, float v4, float v5, float v6, float v7,
        float v8, float v9, float v10, float v11, float v12, float v13, float v14, float v15)
    {
        val = _mm512_setr_ps(v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15);
    }
    v_float32x16() {}

    static inline v_float32x16 zero() { return v_float32x16(_mm512_setzero_ps()); }

    float get0() const { return _mm_cvtss_f32(_mm512_castps512_ps128(val)); }
};

struct v_uint64x8
{
    typedef uint64 lane_type;
    enum { nlanes = 8 };
    __m512i val;

    explicit v_uint64x8(__m512i v) : val(v) {}
    v_uint64x8(uint64 v0, uint64 v1, uint64 v2, uint64 v3, uint64 v4, uint64 v5, uint64 v6, uint64 v7)
    {
        val = _mm512_setr_epi64((int64)v0, (int64)v1, (int64)v2, (int64)v3, (int64)v4, (int64)v5, (int64)v6, (int64)v7);
    }
    v_uint64x8() {}

    static inline v_uint64x8 zero() { return v_uint64x8(_mm512_setzero_si512()); }

    uint64 get0() const
    {
#if defined __x86_64__ || defined _M_X64
        return (uint64)_mm_cvtsi128_si64(_mm512_castsi512_si128(val));
#else
        int a = _mm_cvtsi128_si32(_mm512_castsi512_si128(val));
        int b = _mm_cvtsi128_si32(_mm512_castsi512_si128(_mm512_srli_epi64(val, 32)));
        return (unsigned)a | ((uint64)(unsigned)b << 32);
#endif
    }
};

struct v_int64x8
{
    typedef int64 lane_type;
    enum { nlanes = 8 };
    __m512i val;

    explicit v_int64x8(__m512i v) : val(v) {}
    v_int64x8(int64 v0, int64 v1, int64 v2, int64 v3, int64 v4, int64 v5, int64 v6, int64 v7)
    {
        val = _mm512_setr_epi64(v0, v1, v2, v3, v4, v5, v6, v7);
    }
    v_int64x8() {}

    static inline v_int64x8 zero() { return v_int64x8(_mm512_setzero_si512()); }

    int64 get0() const
    {
#if defined __x86_64__ || defined _M_X64
        return (int64)_mm_cvtsi128_si64(_mm512_castsi512_si128(val));
#else
        int a = _mm_cvtsi128_si32(_mm512_castsi512_si128(val));
        int b = _mm_cvtsi128_si32(_mm512_castsi512_si128(_mm512_srli_epi64(val, 32)));
        return (int64)((unsigned)a | ((uint64)(unsigned)b << 32));
#endif
    }
};

struct v_float64x8
{
    typedef double lane_type;
    enum { nlanes = 8 };
    __m512d val;

    explicit v_float64x8(__m512d v) : val(v) {}
    v_float64x8(double v0, double v1, double v2, double v3, double v4, double v5, double v6, double v7)
    {
        val = _mm512_setr_pd(v0, v1, v2, v3, v4, v5, v6, v7);
    }
    v_float64x8() {}

    static inline v_float64x8 zero() { return v_float64x8(_mm512_setzero_pd()); }

    double get0() const { return _mm_cvtsd_f64(_mm512_castpd512_pd128(val)); }
};



typedef v_uint8x64    v_uint8;
typedef v_int8x64     v_int8;
typedef v_uint16x32   v_uint16;
typedef v_int16x32    v_int16;
typedef v_uint32x16   v_uint32;
typedef v_int32x16    v_int32;
typedef v_uint64x8    v_uint64;
typedef v_int64x8     v_int64;
//typedef v_float32x16  v_float32;

static void cvtCopy(const uchar* src, size_t sstep,
    uchar* dst, size_t dstep, Size size, size_t elemsize)
{
    size_t len = size.width * elemsize;
    for (int i = 0; i < size.height; i++, src += sstep, dst += dstep)
    {
        memcpy(dst, src, len);
    }
}

static void cvt8u(const uchar* src, size_t sstep, const uchar*, size_t, uchar* dst, size_t dstep, Size size, void*)
{
    //CV_INSTRUMENT_REGION(); 
    cvtCopy(src, sstep, dst, dstep, size, 1);
}

static void cvt16u(const uchar* src, size_t sstep, const uchar*, size_t, uchar* dst, size_t dstep, Size size, void*)
{
    //CV_INSTRUMENT_REGION(); 
    cvtCopy((const uchar*)src, sstep, (uchar*)dst, dstep, size, 2);
}

static void cvt32s(const uchar* src, size_t sstep, const uchar*, size_t, uchar* dst, size_t dstep, Size size, void*)
{
    //CV_INSTRUMENT_REGION(); 
    cvtCopy((const uchar*)src, sstep, (uchar*)dst, dstep, size, 4);
}

static void cvt64s(const uchar* src, size_t sstep, const uchar*, size_t, uchar* dst, size_t dstep, Size size, void*)
{
   // CV_INSTRUMENT_REGION(); 
    cvtCopy((const uchar*)src, sstep, (uchar*)dst, dstep, size, 8);
}


 //CV_INSTRUMENT_REGION(); \

template<typename _Ts, typename _Td, typename _Twvec> static inline void
cvt_(const _Ts* src, size_t sstep, _Td* dst, size_t dstep, Size size)
{
    sstep /= sizeof(src[0]);
    dstep /= sizeof(dst[0]);

    for (int i = 0; i < size.height; i++, src += sstep, dst += dstep)
    {
        int j = 0;
#if CV_SIMD
        const int VECSZ = _Twvec::nlanes * 2;
        for (; j < size.width; j += VECSZ)
        {
            if (j > size.width - VECSZ)
            {
                if (j == 0 || src == (_Ts*)dst)
                    break;
                j = size.width - VECSZ;
            }
            _Twvec v0, v1;
            vx_load_pair_as(src + j, v0, v1);
            v_store_pair_as(dst + j, v0, v1);
        }
#endif
        for (; j < size.width; j++)
            dst[j] = saturate_cast<_Td>(src[j]);
    }
}

template<typename _Ts, typename _Td, typename _Twvec> static inline void
cvt1_(const _Ts* src, size_t sstep, _Td* dst, size_t dstep, Size size)
{
    sstep /= sizeof(src[0]);
    dstep /= sizeof(dst[0]);

    for (int i = 0; i < size.height; i++, src += sstep, dst += dstep)
    {
        int j = 0;
#if CV_SIMD
        const int VECSZ = _Twvec::nlanes;
        for (; j < size.width; j += VECSZ)
        {
            if (j > size.width - VECSZ)
            {
                if (j == 0 || src == (_Ts*)dst)
                    break;
                j = size.width - VECSZ;
            }
            _Twvec v;
            vx_load_as(src + j, v);
            v_store_as(dst + j, v);
        }
        vx_cleanup();
#endif
        for (; j < size.width; j++)
            dst[j] = saturate_cast<_Td>(src[j]);
    }
}


#define DEF_CVT_FUNC(suffix, cvtfunc, _Ts, _Td, _Twvec) \
static void cvt##suffix(const uchar* src_, size_t sstep, const uchar*, size_t, \
                        uchar* dst_, size_t dstep, Size size, void*) \
{ \
    const _Ts* src = (const _Ts*)src_; \
    _Td* dst = (_Td*)dst_; \
    cvtfunc<_Ts, _Td, _Twvec>(src, sstep, dst, dstep, size); \
}

////////////////////// 8u -> ... ////////////////////////

DEF_CVT_FUNC(8u8s, cvt_, uchar, schar, v_int16)
DEF_CVT_FUNC(8u16u, cvt_, uchar, ushort, v_uint16)
DEF_CVT_FUNC(8u16s, cvt_, uchar, short, v_int16)
DEF_CVT_FUNC(8u32s, cvt_, uchar, int, v_int32)
DEF_CVT_FUNC(8u32f, cvt_, uchar, float, v_float32)
DEF_CVT_FUNC(8u64f, cvt_, uchar, double, v_int32)
DEF_CVT_FUNC(8u16f, cvt1_, uchar, float16_t, v_float32)

////////////////////// 8s -> ... ////////////////////////

DEF_CVT_FUNC(8s8u, cvt_, schar, uchar, v_int16)
DEF_CVT_FUNC(8s16u, cvt_, schar, ushort, v_uint16)
DEF_CVT_FUNC(8s16s, cvt_, schar, short, v_int16)
DEF_CVT_FUNC(8s32s, cvt_, schar, int, v_int32)
DEF_CVT_FUNC(8s32f, cvt_, schar, float, v_float32)
DEF_CVT_FUNC(8s64f, cvt_, schar, double, v_int32)
DEF_CVT_FUNC(8s16f, cvt1_, schar, float16_t, v_float32)

////////////////////// 16u -> ... ////////////////////////

DEF_CVT_FUNC(16u8u, cvt_, ushort, uchar, v_uint16)
DEF_CVT_FUNC(16u8s, cvt_, ushort, schar, v_uint16)
DEF_CVT_FUNC(16u16s, cvt_, ushort, short, v_int32)
DEF_CVT_FUNC(16u32s, cvt_, ushort, int, v_int32)
DEF_CVT_FUNC(16u32f, cvt_, ushort, float, v_float32)
DEF_CVT_FUNC(16u64f, cvt_, ushort, double, v_int32)
DEF_CVT_FUNC(16u16f, cvt1_, ushort, float16_t, v_float32)

////////////////////// 16s -> ... ////////////////////////

DEF_CVT_FUNC(16s8u, cvt_, short, uchar, v_int16)
DEF_CVT_FUNC(16s8s, cvt_, short, schar, v_int16)
DEF_CVT_FUNC(16s16u, cvt_, short, ushort, v_int32)
DEF_CVT_FUNC(16s32s, cvt_, short, int, v_int32)
DEF_CVT_FUNC(16s32f, cvt_, short, float, v_float32)
DEF_CVT_FUNC(16s64f, cvt_, short, double, v_int32)
DEF_CVT_FUNC(16s16f, cvt1_, short, float16_t, v_float32)

////////////////////// 32s -> ... ////////////////////////

DEF_CVT_FUNC(32s8u, cvt_, int, uchar, v_int32)
DEF_CVT_FUNC(32s8s, cvt_, int, schar, v_int32)
DEF_CVT_FUNC(32s16u, cvt_, int, ushort, v_int32)
DEF_CVT_FUNC(32s16s, cvt_, int, short, v_int32)
DEF_CVT_FUNC(32s32f, cvt_, int, float, v_float32)
DEF_CVT_FUNC(32s64f, cvt_, int, double, v_int32)
DEF_CVT_FUNC(32s16f, cvt1_, int, float16_t, v_float32)

////////////////////// 32f -> ... ////////////////////////

DEF_CVT_FUNC(32f8u, cvt_, float, uchar, v_float32)
DEF_CVT_FUNC(32f8s, cvt_, float, schar, v_float32)
DEF_CVT_FUNC(32f16u, cvt_, float, ushort, v_float32)
DEF_CVT_FUNC(32f16s, cvt_, float, short, v_float32)
DEF_CVT_FUNC(32f32s, cvt_, float, int, v_float32)
DEF_CVT_FUNC(32f64f, cvt_, float, double, v_float32)
DEF_CVT_FUNC(32f16f, cvt1_, float, float16_t, v_float32)

////////////////////// 64f -> ... ////////////////////////

DEF_CVT_FUNC(64f8u, cvt_, double, uchar, v_int32)
DEF_CVT_FUNC(64f8s, cvt_, double, schar, v_int32)
DEF_CVT_FUNC(64f16u, cvt_, double, ushort, v_int32)
DEF_CVT_FUNC(64f16s, cvt_, double, short, v_int32)
DEF_CVT_FUNC(64f32s, cvt_, double, int, v_int32)
DEF_CVT_FUNC(64f32f, cvt_, double, float, v_float32)
DEF_CVT_FUNC(64f16f, cvt1_, double, float16_t, v_float32)

////////////////////// 16f -> ... ////////////////////////

DEF_CVT_FUNC(16f8u, cvt_, float16_t, uchar, v_float32)
DEF_CVT_FUNC(16f8s, cvt_, float16_t, schar, v_float32)
DEF_CVT_FUNC(16f16u, cvt1_, float16_t, ushort, v_float32)
DEF_CVT_FUNC(16f16s, cvt1_, float16_t, short, v_float32)
DEF_CVT_FUNC(16f32s, cvt1_, float16_t, int, v_float32)
DEF_CVT_FUNC(16f32f, cvt1_, float16_t, float, v_float32)
DEF_CVT_FUNC(16f64f, cvt1_, float16_t, double, v_float32)

BinaryFunc getConvertFunc(int sdepth, int ddepth)
{
    static BinaryFunc cvtTab[][8] =
    {
        {
            (cvt8u), (cvt8s8u), (cvt16u8u),
            (cvt16s8u), (cvt32s8u), (cvt32f8u),
            (cvt64f8u), (cvt16f8u)
        },
        {
            (cvt8u8s), cvt8u, (cvt16u8s),
            (cvt16s8s), (cvt32s8s), (cvt32f8s),
            (cvt64f8s), (cvt16f8s)
        },
        {
            (cvt8u16u), (cvt8s16u), cvt16u,
            (cvt16s16u), (cvt32s16u), (cvt32f16u),
            (cvt64f16u), (cvt16f16u)
        },
        {
            (cvt8u16s), (cvt8s16s), (cvt16u16s),
            cvt16u, (cvt32s16s), (cvt32f16s),
            (cvt64f16s), (cvt16f16s)
        },
        {
            (cvt8u32s), (cvt8s32s), (cvt16u32s),
            (cvt16s32s), cvt32s, (cvt32f32s),
            (cvt64f32s), (cvt16f32s)
        },
        {
            (cvt8u32f), (cvt8s32f), (cvt16u32f),
            (cvt16s32f), (cvt32s32f), cvt32s,
            (cvt64f32f), (cvt16f32f)
        },
        {
            (cvt8u64f), (cvt8s64f), (cvt16u64f),
            (cvt16s64f), (cvt32s64f), (cvt32f64f),
            (cvt64s), (cvt16f64f)
        },
        {
            (cvt8u16f), (cvt8s16f), (cvt16u16f), (cvt16s16f),
            (cvt32s16f), (cvt32f16f), (cvt64f16f), (cvt16u)
        }
    };
    return cvtTab[CV_MAT_DEPTH(ddepth)][CV_MAT_DEPTH(sdepth)];
}

inline bool checkScalar(InputArray sc, int atype, _InputArray::KindFlag sckind, _InputArray::KindFlag akind)
{
    if (sc.dims() > 2 || !sc.isContinuous())
        return false;
    Size sz = sc.size();
    if (sz.width != 1 && sz.height != 1)
        return false;
    int cn = CV_MAT_CN(atype);
    if (akind == _InputArray::MATX && sckind != _InputArray::MATX)
        return false;
    return sz == Size(1, 1) || sz == Size(1, cn) || sz == Size(cn, 1) ||
        (sz == Size(1, 4) && sc.type() == CV_64F && cn <= 4);
}



static inline Size getContinuousSize_(int flags, int cols, int rows, int widthScale)
{
    int64 sz = (int64)cols * rows * widthScale;
    bool has_int_overflow = sz >= INT_MAX;
    bool isContiguous = (flags & Mat::CONTINUOUS_FLAG) != 0;
    return (isContiguous && !has_int_overflow)
        ? Size((int)sz, 1)
        : Size(cols * widthScale, rows);
}

Size getContinuousSize2D(Mat& m1, Mat& m2, Mat& m3, int widthScale = 1);

Size getContinuousSize2D(Mat& m1, Mat& m2, Mat& m3, int widthScale)
{
    CV_CheckLE(m1.dims, 2, "");
    CV_CheckLE(m2.dims, 2, "");
    CV_CheckLE(m3.dims, 2, "");
    const Size sz1 = m1.size();
    if (sz1 != m2.size() || sz1 != m3.size())  // reshape all matrixes to the same size (#4159)
    {
        size_t total_sz = m1.total();
        CV_CheckEQ(total_sz, m2.total(), "");
        CV_CheckEQ(total_sz, m3.total(), "");
        bool is_m1_vector = m1.cols == 1 || m1.rows == 1;
        bool is_m2_vector = m2.cols == 1 || m2.rows == 1;
        bool is_m3_vector = m3.cols == 1 || m3.rows == 1;
        CV_Assert(is_m1_vector); CV_Assert(is_m2_vector); CV_Assert(is_m3_vector);
        int total = (int)total_sz;  // vector-column
        bool isContiguous = ((m1.flags & m2.flags & m3.flags) & Mat::CONTINUOUS_FLAG) != 0;
        bool has_int_overflow = ((int64)total_sz * widthScale) >= INT_MAX;
        if (isContiguous && !has_int_overflow)
            total = 1; // vector-row
        m1 = m1.reshape(0, total);
        m2 = m2.reshape(0, total);
        m3 = m3.reshape(0, total);
        CV_Assert(m1.cols == m2.cols && m1.rows == m2.rows && m1.cols == m3.cols && m1.rows == m3.rows);
        return Size(m1.cols * widthScale, m1.rows);
    }
    return getContinuousSize_(m1.flags & m2.flags & m3.flags,
        m1.cols, m1.rows, widthScale);
}

static int actualScalarDepth(const double* data, int len)
{
    
    int i = 0, minval = INT_MAX, maxval = INT_MIN;
    for (; i < len; ++i)
    {
        int ival = cvRound(data[i]);
        if (ival != data[i])
            break;
        minval = MIN(minval, ival);
        maxval = MAX(maxval, ival);
    }
    return i < len ? CV_64F :
        minval >= 0 && maxval <= (int)UCHAR_MAX ? CV_8U :
        minval >= (int)SCHAR_MIN && maxval <= (int)SCHAR_MAX ? CV_8S :
        minval >= 0 && maxval <= (int)USHRT_MAX ? CV_16U :
        minval >= (int)SHRT_MIN && maxval <= (int)SHRT_MAX ? CV_16S :
        CV_32S;
}

template<typename T> static void
copyMask_(const uchar* _src, size_t sstep, const uchar* mask, size_t mstep, uchar* _dst, size_t dstep, Size size)
{
    for (; size.height--; mask += mstep, _src += sstep, _dst += dstep)
    {
        const T* src = (const T*)_src;
        T* dst = (T*)_dst;
        int x = 0;
#if CV_ENABLE_UNROLLED
        for (; x <= size.width - 4; x += 4)
        {
            if (mask[x])
                dst[x] = src[x];
            if (mask[x + 1])
                dst[x + 1] = src[x + 1];
            if (mask[x + 2])
                dst[x + 2] = src[x + 2];
            if (mask[x + 3])
                dst[x + 3] = src[x + 3];
        }
#endif
        for (; x < size.width; x++)
            if (mask[x])
                dst[x] = src[x];
    }
}

template<> void
copyMask_<uchar>(const uchar* _src, size_t sstep, const uchar* mask, size_t mstep, uchar* _dst, size_t dstep, Size size)
{
    CV_IPP_RUN_FAST(CV_INSTRUMENT_FUN_IPP(ippiCopy_8u_C1MR, _src, (int)sstep, _dst, (int)dstep, ippiSize(size), mask, (int)mstep) >= 0)

        for (; size.height--; mask += mstep, _src += sstep, _dst += dstep)
        {
            const uchar* src = (const uchar*)_src;
            uchar* dst = (uchar*)_dst;
            int x = 0;
#if CV_SIMD
            {
                v_uint8 v_zero = vx_setzero_u8();

                for (; x <= size.width - v_uint8::nlanes; x += v_uint8::nlanes)
                {
                    v_uint8 v_src = vx_load(src + x),
                        v_dst = vx_load(dst + x),
                        v_nmask = vx_load(mask + x) == v_zero;

                    v_dst = v_select(v_nmask, v_dst, v_src);
                    v_store(dst + x, v_dst);
                }
            }
            vx_cleanup();
#endif
            for (; x < size.width; x++)
                if (mask[x])
                    dst[x] = src[x];
        }
}


template<> void
copyMask_<ushort>(const uchar* _src, size_t sstep, const uchar* mask, size_t mstep, uchar* _dst, size_t dstep, Size size)
{
    CV_IPP_RUN_FAST(CV_INSTRUMENT_FUN_IPP(ippiCopy_16u_C1MR, (const Ipp16u*)_src, (int)sstep, (Ipp16u*)_dst, (int)dstep, ippiSize(size), mask, (int)mstep) >= 0)

        for (; size.height--; mask += mstep, _src += sstep, _dst += dstep)
        {
            const ushort* src = (const ushort*)_src;
            ushort* dst = (ushort*)_dst;
            int x = 0;
#if CV_SIMD
            {
                v_uint8 v_zero = vx_setzero_u8();

                for (; x <= size.width - v_uint8::nlanes; x += v_uint8::nlanes)
                {
                    v_uint16 v_src1 = vx_load(src + x), v_src2 = vx_load(src + x + v_uint16::nlanes),
                        v_dst1 = vx_load(dst + x), v_dst2 = vx_load(dst + x + v_uint16::nlanes);

                    v_uint8 v_nmask1, v_nmask2;
                    v_uint8 v_nmask = vx_load(mask + x) == v_zero;
                    v_zip(v_nmask, v_nmask, v_nmask1, v_nmask2);

                    v_dst1 = v_select(v_reinterpret_as_u16(v_nmask1), v_dst1, v_src1);
                    v_dst2 = v_select(v_reinterpret_as_u16(v_nmask2), v_dst2, v_src2);
                    v_store(dst + x, v_dst1);
                    v_store(dst + x + v_uint16::nlanes, v_dst2);
                }
            }
            vx_cleanup();
#endif
            for (; x < size.width; x++)
                if (mask[x])
                    dst[x] = src[x];
        }
}

#define DEF_COPY_MASK(suffix, type) \
static void copyMask##suffix(const uchar* src, size_t sstep, const uchar* mask, size_t mstep, \
                             uchar* dst, size_t dstep, Size size, void*) \
{ \
    copyMask_<type>(src, sstep, mask, mstep, dst, dstep, size); \
}

#if defined HAVE_IPP
#define DEF_COPY_MASK_F(suffix, type, ippfavor, ipptype) \
static void copyMask##suffix(const uchar* src, size_t sstep, const uchar* mask, size_t mstep, \
                             uchar* dst, size_t dstep, Size size, void*) \
{ \
    CV_IPP_RUN_FAST(CV_INSTRUMENT_FUN_IPP(ippiCopy_##ippfavor, (const ipptype *)src, (int)sstep, (ipptype *)dst, (int)dstep, ippiSize(size), (const Ipp8u *)mask, (int)mstep) >= 0)\
    copyMask_<type>(src, sstep, mask, mstep, dst, dstep, size); \
}
#else
#define DEF_COPY_MASK_F(suffix, type, ippfavor, ipptype) \
static void copyMask##suffix(const uchar* src, size_t sstep, const uchar* mask, size_t mstep, \
                             uchar* dst, size_t dstep, Size size, void*) \
{ \
    copyMask_<type>(src, sstep, mask, mstep, dst, dstep, size); \
}
#endif

#if IPP_VERSION_X100 == 901 // bug in IPP 9.0.1
DEF_COPY_MASK(32sC3, Vec3i)
DEF_COPY_MASK(8uC3, Vec3b)
#else
DEF_COPY_MASK_F(8uC3, Vec3b, 8u_C3MR, Ipp8u)
DEF_COPY_MASK_F(32sC3, Vec3i, 32s_C3MR, Ipp32s)
#endif
DEF_COPY_MASK(8u, uchar)
DEF_COPY_MASK(16u, ushort)
DEF_COPY_MASK_F(32s, int, 32s_C1MR, Ipp32s)
DEF_COPY_MASK_F(16uC3, Vec3s, 16u_C3MR, Ipp16u)
DEF_COPY_MASK(32sC2, Vec2i)
DEF_COPY_MASK_F(32sC4, Vec4i, 32s_C4MR, Ipp32s)
DEF_COPY_MASK(32sC6, Vec6i)
DEF_COPY_MASK(32sC8, Vec8i)



BinaryFunc copyMaskTab[] =
{
    0,
    copyMask8u,
    copyMask16u,
    copyMask8uC3,
    copyMask32s,
    0,
    copyMask16uC3,
    0,
    copyMask32sC2,
    0, 0, 0,
    copyMask32sC3,
    0, 0, 0,
    copyMask32sC4,
    0, 0, 0, 0, 0, 0, 0,
    copyMask32sC6,
    0, 0, 0, 0, 0, 0, 0,
    copyMask32sC8
};


static void
copyMaskGeneric(const uchar* _src, size_t sstep, const uchar* mask, size_t mstep, uchar* _dst, size_t dstep, Size size, void* _esz)
{
    size_t k, esz = *(size_t*)_esz;
    for (; size.height--; mask += mstep, _src += sstep, _dst += dstep)
    {
        const uchar* src = _src;
        uchar* dst = _dst;
        int x = 0;
        for (; x < size.width; x++, src += esz, dst += esz)
        {
            if (!mask[x])
                continue;
            for (k = 0; k < esz; k++)
                dst[k] = src[k];
        }
    }
}


BinaryFunc getCopyMaskFunc(size_t esz)
{
    return esz <= 32 && copyMaskTab[esz] ? copyMaskTab[esz] : copyMaskGeneric;
}


void convertAndUnrollScalar(const Mat& sc, int buftype, uchar* scbuf, size_t blocksize)
{
    int scn = (int)sc.total(), cn = CV_MAT_CN(buftype);
    size_t esz = CV_ELEM_SIZE(buftype);
    BinaryFunc cvtFn = getConvertFunc(sc.depth(), buftype);
    CV_Assert(cvtFn);
    cvtFn(sc.ptr(), 1, 0, 1, scbuf, 1, Size(min(cn, scn), 1), 0);
    // unroll the scalar
    if (scn < cn)
    {
        CV_Assert(scn == 1);
        size_t esz1 = CV_ELEM_SIZE1(buftype);
        for (size_t i = esz1; i < esz; i++)
            scbuf[i] = scbuf[i - esz1];
    }
    for (size_t i = esz; i < blocksize * esz; i++)
        scbuf[i] = scbuf[i - esz];
}



static void arithm_op(InputArray _src1, InputArray _src2, OutputArray _dst,
    InputArray _mask, int dtype, BinaryFuncC* tab, bool muldiv = false,
    void* usrdata = 0, int oclop = -1)
{
    const _InputArray* psrc1 = &_src1, * psrc2 = &_src2;
    _InputArray::KindFlag kind1 = psrc1->kind(), kind2 = psrc2->kind();
    bool haveMask = !_mask.empty();
    bool reallocate = false;
    int type1 = psrc1->type(), depth1 = CV_MAT_DEPTH(type1), cn = CV_MAT_CN(type1);
    int type2 = psrc2->type(), depth2 = CV_MAT_DEPTH(type2), cn2 = CV_MAT_CN(type2);
    int wtype, dims1 = psrc1->dims(), dims2 = psrc2->dims();
    Size sz1 = dims1 <= 2 ? psrc1->size() : Size();
    Size sz2 = dims2 <= 2 ? psrc2->size() : Size();
#ifdef HAVE_OPENCL
    bool use_opencl = OCL_PERFORMANCE_CHECK(_dst.isUMat()) && dims1 <= 2 && dims2 <= 2;
#endif
    bool src1Scalar = checkScalar(*psrc1, type2, kind1, kind2);
    bool src2Scalar = checkScalar(*psrc2, type1, kind2, kind1);
    
    if ((kind1 == kind2 || cn == 1) && sz1 == sz2 && dims1 <= 2 && dims2 <= 2 && type1 == type2 &&
        !haveMask && ((!_dst.fixedType() && (dtype < 0 || CV_MAT_DEPTH(dtype) == depth1)) ||
            (_dst.fixedType() && _dst.type() == type1)) &&
        (src1Scalar == src2Scalar))
    {
       // cout << "depth1 : " << depth1 ;
        _dst.createSameSize(*psrc1, type1);
       /* CV_OCL_RUN(use_opencl,
            ocl_arithm_op(*psrc1, *psrc2, _dst, _mask,
                (!usrdata ? type1 : std::max(depth1, CV_32F)),
                usrdata, oclop, false))*/

            Mat src1 = psrc1->getMat(), src2 = psrc2->getMat(), dst = _dst.getMat();
        Size sz = getContinuousSize2D(src1, src2, dst, src1.channels());
        tab[depth1](src1.ptr(), src1.step, src2.ptr(), src2.step, dst.ptr(), dst.step, sz.width, sz.height, usrdata);
        return;
    }

    bool haveScalar = false, swapped12 = false;

    if (dims1 != dims2 || sz1 != sz2 || cn != cn2 ||
        (kind1 == _InputArray::MATX && (sz1 == Size(1, 4) || sz1 == Size(1, 1))) ||
        (kind2 == _InputArray::MATX && (sz2 == Size(1, 4) || sz2 == Size(1, 1))))
    {
        if (checkScalar(*psrc1, type2, kind1, kind2))
        {
            // src1 is a scalar; swap it with src2
            swap(psrc1, psrc2);
            swap(sz1, sz2);
            swap(type1, type2);
            swap(depth1, depth2);
            swap(cn, cn2);
            swap(dims1, dims2);
            swapped12 = true;
            if (oclop == OCL_OP_SUB)
                oclop = OCL_OP_RSUB;
            if (oclop == OCL_OP_DIV_SCALE)
                oclop = OCL_OP_RDIV_SCALE;
        }
        else if (!checkScalar(*psrc2, type1, kind2, kind1))
            /*CV_Error(CV_StsUnmatchedSizes,
                "The operation is neither 'array op array' "
                "(where arrays have the same size and the same number of channels), "
                "nor 'array op scalar', nor 'scalar op array'");*/
        haveScalar = true;
        CV_Assert(type2 == CV_64F && (sz2.height == 1 || sz2.height == 4));

        if (!muldiv)
        {
            Mat sc = psrc2->getMat();
            depth2 = actualScalarDepth(sc.ptr<double>(), sz2 == Size(1, 1) ? cn2 : cn);
            if (depth2 == CV_64F && (depth1 < CV_32S || depth1 == CV_32F))
                depth2 = CV_32F;
        }
        else
            depth2 = CV_64F;
    }

    if (dtype < 0)
    {
        if (_dst.fixedType())
            dtype = _dst.type();
        else
        {
            if (!haveScalar && type1 != type2)
                CV_Error(CV_StsBadArg,
                    "When the input arrays in add/subtract/multiply/divide functions have different types, "
                    "the output array type must be explicitly specified");
            dtype = type1;
        }
    }
    dtype = CV_MAT_DEPTH(dtype);

    if (depth1 == depth2 && dtype == depth1)
        wtype = dtype;
    else if (!muldiv)
    {
        wtype = depth1 <= CV_8S && depth2 <= CV_8S ? CV_16S :
            depth1 <= CV_32S && depth2 <= CV_32S ? CV_32S : max(depth1, depth2);
        wtype = max(wtype, dtype);

        // when the result of addition should be converted to an integer type,
        // and just one of the input arrays is floating-point, it makes sense to convert that input to integer type before the operation,
        // instead of converting the other input to floating-point and then converting the operation result back to integers.
        if (dtype < CV_32F && (depth1 < CV_32F || depth2 < CV_32F))
            wtype = CV_32S;
    }
    else
    {
        wtype = max(depth1, max(depth2, CV_32F));
        wtype = max(wtype, dtype);
    }

    dtype = CV_MAKETYPE(dtype, cn);
    wtype = CV_MAKETYPE(wtype, cn);

    if (haveMask)
    {
        int mtype = _mask.type();
        CV_Assert((mtype == CV_8UC1 || mtype == CV_8SC1) && _mask.sameSize(*psrc1));
        reallocate = !_dst.sameSize(*psrc1) || _dst.type() != dtype;
    }

    _dst.createSameSize(*psrc1, dtype);
    if (reallocate)
        _dst.setTo(0.);

   /* CV_OCL_RUN(use_opencl,
        ocl_arithm_op(*psrc1, *psrc2, _dst, _mask, wtype,
            usrdata, oclop, haveScalar))*/

    BinaryFunc cvtsrc1 = type1 == wtype ? 0 : getConvertFunc(type1, wtype);
    BinaryFunc cvtsrc2 = type2 == type1 ? cvtsrc1 : type2 == wtype ? 0 : getConvertFunc(type2, wtype);
    BinaryFunc cvtdst = dtype == wtype ? 0 : getConvertFunc(wtype, dtype);
    cout << "cvtsrc1 : " << cvtsrc1 << endl;
    cout << "cvtsrc2 : " << cvtsrc2 << endl;
    cout << "cvtdst : " << cvtdst << endl;

    size_t esz1 = CV_ELEM_SIZE(type1), esz2 = CV_ELEM_SIZE(type2);
    size_t dsz = CV_ELEM_SIZE(dtype), wsz = CV_ELEM_SIZE(wtype);
    size_t blocksize0 = (size_t)(BLOCK_SIZE + wsz - 1) / wsz;
    BinaryFunc copymask = getCopyMaskFunc(dsz);
    Mat src1 = psrc1->getMat(), src2 = psrc2->getMat(), dst = _dst.getMat(), mask = _mask.getMat();

    AutoBuffer<uchar> _buf;
    uchar* buf, * maskbuf = 0, * buf1 = 0, * buf2 = 0, * wbuf = 0;
    size_t bufesz = (cvtsrc1 ? wsz : 0) +
        (cvtsrc2 || haveScalar ? wsz : 0) +
        (cvtdst ? wsz : 0) +
        (haveMask ? dsz : 0);
    BinaryFuncC func = tab[CV_MAT_DEPTH(wtype)];
    CV_Assert(func);

    if (!haveScalar)
    {
        const Mat* arrays[] = { &src1, &src2, &dst, &mask, 0 };
        uchar* ptrs[4] = {};

        NAryMatIterator it(arrays, ptrs);
        size_t total = it.size, blocksize = total;

        if (haveMask || cvtsrc1 || cvtsrc2 || cvtdst)
            blocksize = min(blocksize, blocksize0);

        _buf.allocate(bufesz * blocksize + 64);
        buf = _buf.data();
        if (cvtsrc1)
            buf1 = buf, buf = alignPtr(buf + blocksize * wsz, 16);
        if (cvtsrc2)
            buf2 = buf, buf = alignPtr(buf + blocksize * wsz, 16);
        wbuf = maskbuf = buf;
        if (cvtdst)
            buf = alignPtr(buf + blocksize * wsz, 16);
        if (haveMask)
            maskbuf = buf;

        for (size_t i = 0; i < it.nplanes; i++, ++it)
        {
            for (size_t j = 0; j < total; j += blocksize)
            {
                int bsz = (int)MIN(total - j, blocksize);
                Size bszn(bsz * cn, 1);
                const uchar* sptr1 = ptrs[0], * sptr2 = ptrs[1];
                uchar* dptr = ptrs[2];
                if (cvtsrc1)
                {
                    cvtsrc1(sptr1, 1, 0, 1, buf1, 1, bszn, 0);
                    sptr1 = buf1;
                }
                if (ptrs[0] == ptrs[1])
                    sptr2 = sptr1;
                else if (cvtsrc2)
                {
                    cvtsrc2(sptr2, 1, 0, 1, buf2, 1, bszn, 0);
                    sptr2 = buf2;
                }

                if (!haveMask && !cvtdst)
                    func(sptr1, 1, sptr2, 1, dptr, 1, bszn.width, bszn.height, usrdata);
                else
                {
                    func(sptr1, 1, sptr2, 1, wbuf, 0, bszn.width, bszn.height, usrdata);
                    if (!haveMask)
                        cvtdst(wbuf, 1, 0, 1, dptr, 1, bszn, 0);
                    else if (!cvtdst)
                    {
                        copymask(wbuf, 1, ptrs[3], 1, dptr, 1, Size(bsz, 1), &dsz);
                        ptrs[3] += bsz;
                    }
                    else
                    {
                        cvtdst(wbuf, 1, 0, 1, maskbuf, 1, bszn, 0);
                        copymask(maskbuf, 1, ptrs[3], 1, dptr, 1, Size(bsz, 1), &dsz);
                        ptrs[3] += bsz;
                    }
                }
                ptrs[0] += bsz * esz1; ptrs[1] += bsz * esz2; ptrs[2] += bsz * dsz;
            }
        }
    }
    else
    {
        const Mat* arrays[] = { &src1, &dst, &mask, 0 };
        uchar* ptrs[3] = {};

        NAryMatIterator it(arrays, ptrs);
        size_t total = it.size, blocksize = min(total, blocksize0);

        _buf.allocate(bufesz * blocksize + 64);
        buf = _buf.data();
        if (cvtsrc1)
            buf1 = buf, buf = alignPtr(buf + blocksize * wsz, 16);
        buf2 = buf; buf = alignPtr(buf + blocksize * wsz, 16);
        wbuf = maskbuf = buf;
        if (cvtdst)
            buf = alignPtr(buf + blocksize * wsz, 16);
        if (haveMask)
            maskbuf = buf;

        convertAndUnrollScalar(src2, wtype, buf2, blocksize);

        for (size_t i = 0; i < it.nplanes; i++, ++it)
        {
            for (size_t j = 0; j < total; j += blocksize)
            {
                int bsz = (int)MIN(total - j, blocksize);
                Size bszn(bsz * cn, 1);
                const uchar* sptr1 = ptrs[0];
                const uchar* sptr2 = buf2;
                uchar* dptr = ptrs[1];

                if (cvtsrc1)
                {
                    cvtsrc1(sptr1, 1, 0, 1, buf1, 1, bszn, 0);
                    sptr1 = buf1;
                }

                if (swapped12)
                    std::swap(sptr1, sptr2);

                if (!haveMask && !cvtdst)
                    func(sptr1, 1, sptr2, 1, dptr, 1, bszn.width, bszn.height, usrdata);
                else
                {
                    func(sptr1, 1, sptr2, 1, wbuf, 1, bszn.width, bszn.height, usrdata);
                    if (!haveMask)
                        cvtdst(wbuf, 1, 0, 1, dptr, 1, bszn, 0);
                    else if (!cvtdst)
                    {
                        copymask(wbuf, 1, ptrs[2], 1, dptr, 1, Size(bsz, 1), &dsz);
                        ptrs[2] += bsz;
                    }
                    else
                    {
                        cvtdst(wbuf, 1, 0, 1, maskbuf, 1, bszn, 0);
                        copymask(maskbuf, 1, ptrs[2], 1, dptr, 1, Size(bsz, 1), &dsz);
                        ptrs[2] += bsz;
                    }
                }
                ptrs[0] += bsz * esz1; ptrs[1] += bsz * dsz;
            }
        }
    }
}


static BinaryFuncC* getSubTab()
{
    static BinaryFuncC subTab[] =
    {
        /*(BinaryFuncC)GET_OPTIMIZED(sub8u), (BinaryFuncC)GET_OPTIMIZED(sub8s),
        (BinaryFuncC)GET_OPTIMIZED(sub16u), (BinaryFuncC)GET_OPTIMIZED(sub16s),
        (BinaryFuncC)GET_OPTIMIZED(sub32s),*/
        0,0,0,0,0,
        (BinaryFuncC)GET_OPTIMIZED(sub32f), 0,//(BinaryFuncC)sub64f,
        0
    };

    return subTab;
}

void subtract_(InputArray _src1, InputArray _src2, OutputArray _dst,
    InputArray mask, int dtype)
{
    //CV_INSTRUMENT_REGION();

    arithm_op(_src1, _src2, _dst, mask, dtype, getSubTab(), false, 0, OCL_OP_SUB);
}

CV_EXPORTS_W void subtract_(InputArray src1, InputArray src2, OutputArray dst,
    InputArray mask = noArray(), int dtype = -1);


class buildDoGPyramidComputer : public ParallelLoopBody_
{
public:
    buildDoGPyramidComputer(
        int _nOctaveLayers,
        const std::vector<Mat>& _gpyr,
        std::vector<Mat>& _dogpyr)
        : nOctaveLayers(_nOctaveLayers),
        gpyr(_gpyr),
        dogpyr(_dogpyr) { }

    void operator()(const cv::Range& range) const CV_OVERRIDE
    {
        //CV_TRACE_FUNCTION();

        const int begin = range.start;
        const int end = range.end;

        for (int a = begin; a < end; a++)
        {
            const int o = a / (nOctaveLayers + 2);
            const int i = a % (nOctaveLayers + 2);

            const Mat& src1 = gpyr[o * (nOctaveLayers + 3) + i];
            const Mat& src2 = gpyr[o * (nOctaveLayers + 3) + i + 1];
            Mat& dst = dogpyr[o * (nOctaveLayers + 2) + i];
            subtract_(src2, src1, dst, noArray(), DataType<sift_wt>::type);
        }
    }

private:
    int nOctaveLayers;
    const std::vector<Mat>& gpyr;
    std::vector<Mat>& dogpyr;
};