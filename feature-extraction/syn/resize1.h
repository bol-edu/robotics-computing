

/* ////////////////////////////////////////////////////////////////////
//
//  Geometrical transforms on images and matrices: rotation, zoom etc.
//
// */

//#include "Methods.h"
#include <algorithm>
#include <iostream>
#include <stdint.h>
#include "Mat.h"
//#include "define.h"
#include "saturate_cast.h"
#include "Range.h"
//#include "AutoBuffer__.h"
//#include "parallel_for__.h"
//#include <opencv2/opencv.hpp>
using namespace std;
//using namespace cv;
//using namespace utils;

/*#define CALL_HAL(name, fun, ...) \
{       cout<<"IN CALL HALLLLLLLLLLLLLLLLL"  ;   \
    int res = __CV_EXPAND(fun(__VA_ARGS__)); \
    if (res == CV_HAL_ERROR_OK) \
        return; \
    else if (res != CV_HAL_ERROR_NOT_IMPLEMENTED) \
        CV_Error_(cv::Error::StsInternal, \
            ("HAL implementation " CVAUX_STR(name) " ==> " CVAUX_STR(fun) " returned %d (0x%08x)", res, res)); \
}
#define CV_IPP_RUN_(condition, func, ...)

#define CV_IPP_RUN_FAST(func, ...) CV_IPP_RUN_(true, func, __VA_ARGS__)*/

//inline int hal_ni_resize(int src_type, const uchar* src_data, size_t src_step, int src_width, int src_height, uchar* dst_data, size_t dst_step, int dst_width, int dst_height, double inv_scale_x, double inv_scale_y, int interpolation) { return CV_HAL_ERROR_NOT_IMPLEMENTED; }
//#define cv_hal_resize hal_ni_resize

//using namespace cv;

//namespace
//{
struct softdouble
{
public:

    softdouble() : v(0) { }

    softdouble(const softdouble& c) { v = c.v; }

    softdouble& operator=(const softdouble& c)
    {
        if (&c != this) v = c.v;
        return *this;
    }

    static softdouble fromRaw(const uint64_t a) { softdouble x; x.v = a; return x; }


    explicit softdouble(const uint32_t);
    explicit softdouble(const uint64_t);
    explicit softdouble(const  int32_t);
    explicit softdouble(const  int64_t);

#ifdef CV_INT32_T_IS_LONG_INT
    // for platforms with int32_t = long int
    explicit softdouble(const int a) { *this = softdouble(static_cast<int32_t>(a)); }
#endif


    explicit softdouble(const double a) { Cv64suf s; s.f = a; v = s.u; }


    //operator softfloat() const;
    operator double() const { Cv64suf s; s.u = v; return s.f; }


    softdouble operator + (const softdouble&) const;
    softdouble operator - (const softdouble&) const;
    softdouble operator * (const softdouble&) const;
    softdouble operator / (const softdouble&) const;
    softdouble operator - () const { softdouble x; x.v = v ^ (1ULL << 63); return x; }


    softdouble operator % (const softdouble&) const;

    softdouble& operator += (const softdouble& a) { *this = *this + a; return *this; }
    softdouble& operator -= (const softdouble& a) { *this = *this - a; return *this; }
    softdouble& operator *= (const softdouble& a) { *this = *this * a; return *this; }
    softdouble& operator /= (const softdouble& a) { *this = *this / a; return *this; }
    softdouble& operator %= (const softdouble& a) { *this = *this % a; return *this; }


    bool operator == (const softdouble&) const;
    bool operator != (const softdouble&) const;
    bool operator >  (const softdouble&) const;
    bool operator >= (const softdouble&) const;
    bool operator <  (const softdouble&) const;
    bool operator <= (const softdouble&) const;


    inline bool isNaN() const { return (v & 0x7fffffffffffffff) > 0x7ff0000000000000; }

    inline bool isInf() const { return (v & 0x7fffffffffffffff) == 0x7ff0000000000000; }

    inline bool isSubnormal() const { return ((v >> 52) & 0x7FF) == 0; }


    inline bool getSign() const { return (v >> 63) != 0; }

    softdouble setSign(bool sign) const { softdouble x; x.v = (v & ((1ULL << 63) - 1)) | ((uint_fast64_t)(sign) << 63); return x; }

    inline int getExp() const { return ((v >> 52) & 0x7FF) - 1023; }

    inline softdouble setExp(int e) const
    {
        softdouble x;
        x.v = (v & 0x800FFFFFFFFFFFFF) | ((uint_fast64_t)((e + 1023) & 0x7FF) << 52);
        return x;
    }


    inline softdouble getFrac() const
    {
        uint_fast64_t vv = (v & 0x000FFFFFFFFFFFFF) | ((uint_fast64_t)(1023) << 52);
        return softdouble::fromRaw(vv);
    }

    inline softdouble setFrac(const softdouble& s) const
    {
        softdouble x;
        x.v = (v & 0xFFF0000000000000) | (s.v & 0x000FFFFFFFFFFFFF);
        return x;
    }


    static softdouble zero() { return softdouble::fromRaw(0); }

    static softdouble  inf() { return softdouble::fromRaw((uint_fast64_t)(0x7FF) << 52); }

    static softdouble  nan() { return softdouble::fromRaw(CV_BIG_INT(0x7FFFFFFFFFFFFFFF)); }

    static softdouble  one() { return softdouble::fromRaw((uint_fast64_t)(1023) << 52); }

    static softdouble  min_() { return softdouble::fromRaw((uint_fast64_t)(0x01) << 52); }

    static softdouble  eps() { return softdouble::fromRaw((uint_fast64_t)(1023 - 52) << 52); }

    static softdouble  max_() { return softdouble::fromRaw(((uint_fast64_t)(0x7FF) << 52) - 1); }

    static softdouble   pi() { return softdouble::fromRaw(CV_BIG_INT(0x400921FB54442D18)); }

    uint64_t v;
};

typedef softdouble float64_t;
#define signF64UI( a ) (((uint64_t) (a)>>63) != 0)
#define expF64UI( a ) ((int_fast16_t) ((a)>>52) & 0x7FF)
#define fracF64UI( a ) ((a) & UINT64_C( 0x000FFFFFFFFFFFFF ))
#define packToF64UI( sign, exp, sig ) ((uint64_t) (((uint_fast64_t) (sign)<<63) + ((uint_fast64_t) (exp)<<52) + (sig)))
#define isNaNF64UI( a ) (((~(a) & UINT64_C( 0x7FF0000000000000 )) == 0) && ((a) & UINT64_C( 0x000FFFFFFFFFFFFF )))
#define i64_fromNegOverflow  (~UINT64_C( 0x7FFFFFFFFFFFFFFF ) + 1 - 1)
#define i64_fromPosOverflow  UINT64_C( 0x7FFFFFFFFFFFFFFF )
#define i64_fromNaN          UINT64_C( 0x7FFFFFFFFFFFFFFF )

enum {
    round_near_even = 0, // round to nearest, with ties to even
    round_minMag = 1, // round to minimum magnitude (toward zero)
    round_min = 2, // round to minimum (down)
    round_max = 3, // round to maximum (up)
    round_near_maxMag = 4, // round to nearest, with ties to maximum magnitude (away from zero)
    round_odd = 5  // round to odd (jamming)
};
static const uint_fast8_t globalRoundingMode = round_near_even;

enum {
    tininess_beforeRounding = 0,
    tininess_afterRounding = 1
};
//fixed to make softfloat code stateless
static const uint_fast8_t globalDetectTininess = tininess_afterRounding;

static inline uint64_t softfloat_shiftRightJam64(uint64_t a, uint_fast32_t dist)
{
    //fixed unsigned unary minus: -x == ~x + 1
    return (dist < 63) ? a >> dist | ((uint64_t)(a << ((~dist + 1) & 63)) != 0) : (a != 0);
}

static float64_t
softfloat_roundPackToF64(bool sign, int_fast16_t exp, uint_fast64_t sig)
{
    uint_fast8_t roundingMode;
    bool roundNearEven;
    uint_fast16_t roundIncrement, roundBits;
    bool isTiny;
    uint_fast64_t uiZ;

    //------------------------------------------------------------------------
   //------------------------------------------------------------------------
    roundingMode = globalRoundingMode;
    roundNearEven = (roundingMode == round_near_even);
    roundIncrement = 0x200;
    if (!roundNearEven && (roundingMode != round_near_maxMag)) {
        roundIncrement =
            (roundingMode
                == (sign ? round_min : round_max))
            ? 0x3FF
            : 0;
    }
    roundBits = sig & 0x3FF;
    //------------------------------------------------------------------------
    //-----------------------------------------------------------------------
    if (0x7FD <= (uint16_t)exp) {
        if (exp < 0) {
            //
            isTiny =
                (globalDetectTininess == tininess_beforeRounding)
                || (exp < -1)
                || (sig + roundIncrement < UINT64_C(0x8000000000000000));
            sig = softfloat_shiftRightJam64(sig, -exp);
            exp = 0;
            roundBits = sig & 0x3FF;
            if (isTiny && roundBits) {
                //raiseFlags(flag_underflow);
            }
        }
        else if (
            (0x7FD < exp)
            || (UINT64_C(0x8000000000000000) <= sig + roundIncrement)
            ) {
            //
            //raiseFlags(
                //flag_overflow | flag_inexact);
            uiZ = packToF64UI(sign, 0x7FF, 0) - !roundIncrement;
            goto uiZ;
        }
    }
    //
    sig = (sig + roundIncrement) >> 10;
    if (roundBits) {
        //raiseFlags(flag_inexact);
        if (roundingMode == round_odd) {
            sig |= 1;
            goto packReturn;
        }
    }
    sig &= ~(uint_fast64_t)(!(roundBits ^ 0x200) & roundNearEven);
    if (!sig) exp = 0;
    //
packReturn:
    uiZ = packToF64UI(sign, exp, sig);
uiZ:
    return float64_t::fromRaw(uiZ);
}
#define softfloat_isSigNaNF64UI( uiA ) \
    ((((uiA) & UINT64_C( 0x7FF8000000000000 )) == UINT64_C( 0x7FF0000000000000 )) && \
      ((uiA) & UINT64_C( 0x0007FFFFFFFFFFFF )))
static uint_fast64_t
softfloat_propagateNaNF64UI(uint_fast64_t uiA, uint_fast64_t uiB)
{
    bool isSigNaNA;

    isSigNaNA = softfloat_isSigNaNF64UI(uiA);
    if (isSigNaNA || softfloat_isSigNaNF64UI(uiB)) {
        //raiseFlags(flag_invalid);
        if (isSigNaNA) return uiA | UINT64_C(0x0008000000000000);
    }
    return (isNaNF64UI(uiA) ? uiA : uiB) | UINT64_C(0x0008000000000000);
}

static float64_t
softfloat_addMagsF64(uint_fast64_t uiA, uint_fast64_t uiB, bool signZ)
{
    int_fast16_t expA;
    uint_fast64_t sigA;
    int_fast16_t expB;
    uint_fast64_t sigB;
    int_fast16_t expDiff;
    uint_fast64_t uiZ;
    int_fast16_t expZ;
    uint_fast64_t sigZ;

    //
    expA = expF64UI(uiA);
    sigA = fracF64UI(uiA);
    expB = expF64UI(uiB);
    sigB = fracF64UI(uiB);
    //
    expDiff = expA - expB;
    if (!expDiff) {
        //
        if (!expA) {
            uiZ = uiA + sigB;
            goto uiZ;
        }
        if (expA == 0x7FF) {
            if (sigA | sigB) goto propagateNaN;
            uiZ = uiA;
            goto uiZ;
        }
        expZ = expA;
        sigZ = UINT64_C(0x0020000000000000) + sigA + sigB;
        sigZ <<= 9;
    }
    else {
        //
        sigA <<= 9;
        sigB <<= 9;
        if (expDiff < 0) {
            if (expB == 0x7FF) {
                if (sigB) goto propagateNaN;
                uiZ = packToF64UI(signZ, 0x7FF, 0);
                goto uiZ;
            }
            expZ = expB;
            if (expA) {
                sigA += UINT64_C(0x2000000000000000);
            }
            else {
                sigA <<= 1;
            }
            sigA = softfloat_shiftRightJam64(sigA, -expDiff);
        }
        else {
            if (expA == 0x7FF) {
                if (sigA) goto propagateNaN;
                uiZ = uiA;
                goto uiZ;
            }
            expZ = expA;
            if (expB) {
                sigB += UINT64_C(0x2000000000000000);
            }
            else {
                sigB <<= 1;
            }
            sigB = softfloat_shiftRightJam64(sigB, expDiff);
        }
        sigZ = UINT64_C(0x2000000000000000) + sigA + sigB;
        if (sigZ < UINT64_C(0x4000000000000000)) {
            --expZ;
            sigZ <<= 1;
        }
    }
    return softfloat_roundPackToF64(signZ, expZ, sigZ);
    //
propagateNaN:
    uiZ = softfloat_propagateNaNF64UI(uiA, uiB);
uiZ:
    return float64_t::fromRaw(uiZ);
}

#define defaultNaNF64UI UINT64_C( 0xFFF8000000000000 )

static const uint_least8_t softfloat_countLeadingZeros8[256] = {
    8, 7, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4,
    3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static uint_fast8_t softfloat_countLeadingZeros64(uint64_t a)
{
    uint_fast8_t count;
    uint32_t a32;

    count = 0;
    a32 = a >> 32;
    if (!a32) {
        count = 32;
        a32 = (uint32_t)a; //fixed warning on type cast
    }
    //-----------------------------------------------------------------------
    //From here, result is current count + count leading zeros of `a32'.
    //----------------------------------------------------------------------
    if (a32 < 0x10000) {
        count += 16;
        a32 <<= 16;
    }
    if (a32 < 0x1000000) {
        count += 8;
        a32 <<= 8;
    }
    count += softfloat_countLeadingZeros8[a32 >> 24];
    return count;
}

static float64_t
softfloat_normRoundPackToF64(bool sign, int_fast16_t exp, uint_fast64_t sig)
{
    int_fast8_t shiftDist;

    shiftDist = softfloat_countLeadingZeros64(sig) - 1;
    exp -= shiftDist;
    if ((10 <= shiftDist) && ((unsigned int)exp < 0x7FD)) {
        return float64_t::fromRaw(packToF64UI(sign, sig ? exp : 0, sig << (shiftDist - 10)));
    }
    else {
        return softfloat_roundPackToF64(sign, exp, sig << shiftDist);
    }
}

static float64_t
softfloat_subMagsF64(uint_fast64_t uiA, uint_fast64_t uiB, bool signZ)
{
    int_fast16_t expA;
    uint_fast64_t sigA;
    int_fast16_t expB;
    uint_fast64_t sigB;
    int_fast16_t expDiff;
    uint_fast64_t uiZ;
    int_fast64_t sigDiff;
    int_fast8_t shiftDist;
    int_fast16_t expZ;
    uint_fast64_t sigZ;

    //
    expA = expF64UI(uiA);
    sigA = fracF64UI(uiA);
    expB = expF64UI(uiB);
    sigB = fracF64UI(uiB);
    //
    expDiff = expA - expB;
    if (!expDiff) {
        //
        if (expA == 0x7FF) {
            if (sigA | sigB) goto propagateNaN;
            //raiseFlags(flag_invalid);
            uiZ = defaultNaNF64UI;
            goto uiZ;
        }
        sigDiff = sigA - sigB;
        if (!sigDiff) {
            uiZ =
                packToF64UI(
                    (globalRoundingMode == round_min), 0, 0);
            goto uiZ;
        }
        if (expA) --expA;
        if (sigDiff < 0) {
            signZ = !signZ;
            sigDiff = -sigDiff;
        }
        shiftDist = softfloat_countLeadingZeros64(sigDiff) - 11;
        expZ = expA - shiftDist;
        if (expZ < 0) {
            shiftDist = (int_fast8_t)expA; //fixed type cast
            expZ = 0;
        }
        uiZ = packToF64UI(signZ, expZ, sigDiff << shiftDist);
        goto uiZ;
    }
    else {
       //
        sigA <<= 10;
        sigB <<= 10;
        if (expDiff < 0) {
            //
            signZ = !signZ;
            if (expB == 0x7FF) {
                if (sigB) goto propagateNaN;
                uiZ = packToF64UI(signZ, 0x7FF, 0);
                goto uiZ;
            }
            sigA += expA ? UINT64_C(0x4000000000000000) : sigA;
            sigA = softfloat_shiftRightJam64(sigA, -expDiff);
            sigB |= UINT64_C(0x4000000000000000);
            expZ = expB;
            sigZ = sigB - sigA;
        }
        else {
            ///
            //
            if (expA == 0x7FF) {
                if (sigA) goto propagateNaN;
                uiZ = uiA;
                goto uiZ;
            }
            sigB += expB ? UINT64_C(0x4000000000000000) : sigB;
            sigB = softfloat_shiftRightJam64(sigB, expDiff);
            sigA |= UINT64_C(0x4000000000000000);
            expZ = expA;
            sigZ = sigA - sigB;
        }
        return softfloat_normRoundPackToF64(signZ, expZ - 1, sigZ);
    }
    ///--------
    //-------
propagateNaN:
    uiZ = softfloat_propagateNaNF64UI(uiA, uiB);
uiZ:
    return float64_t::fromRaw(uiZ);
}

static float64_t f64_add(float64_t a, float64_t b)
{
    uint_fast64_t uiA;
    bool signA;
    uint_fast64_t uiB;
    bool signB;

    uiA = a.v;
    signA = signF64UI(uiA);
    uiB = b.v;
    signB = signF64UI(uiB);
    if (signA == signB) {
        return softfloat_addMagsF64(uiA, uiB, signA);
    }
    else {
        return softfloat_subMagsF64(uiA, uiB, signA);
    }
}

static float64_t f64_sub(float64_t a, float64_t b)
{
    uint_fast64_t uiA;
    bool signA;
    uint_fast64_t uiB;
    bool signB;

    uiA = a.v;
    signA = signF64UI(uiA);
    uiB = b.v;
    signB = signF64UI(uiB);

    if (signA == signB) {
        return softfloat_subMagsF64(uiA, uiB, signA);
    }
    else {
        return softfloat_addMagsF64(uiA, uiB, signA);
    }
}

struct exp16_sig64 { int_fast16_t exp; uint_fast64_t sig; };
static struct exp16_sig64 softfloat_normSubnormalF64Sig(uint_fast64_t sig)
{
    int_fast8_t shiftDist;
    struct exp16_sig64 z;

    shiftDist = softfloat_countLeadingZeros64(sig) - 11;
    z.exp = 1 - shiftDist;
    z.sig = sig << shiftDist;
    return z;
}

#ifndef WORDS_BIGENDIAN
struct uint128 { uint64_t v0, v64; };
//struct uint64_extra { uint64_t extra, v; };
//struct uint128_extra { uint64_t extra; struct uint128 v; };
#else
struct uint128 { uint64_t v64, v0; };
//struct uint64_extra { uint64_t v, extra; };
//struct uint128_extra { struct uint128 v; uint64_t extra; };
#endif


static struct uint128 softfloat_mul64To128(uint64_t a, uint64_t b)
{
    uint32_t a32, a0, b32, b0;
    struct uint128 z;
    uint64_t mid1, mid;

    a32 = a >> 32;
    a0 = (uint32_t)a; //fixed warning on type cast
    b32 = b >> 32;
    b0 = (uint32_t)b; //fixed warning on type cast
    z.v0 = (uint_fast64_t)a0 * b0;
    mid1 = (uint_fast64_t)a32 * b0;
    mid = mid1 + (uint_fast64_t)a0 * b32;
    z.v64 = (uint_fast64_t)a32 * b32;
    z.v64 += (uint_fast64_t)(mid < mid1) << 32 | mid >> 32;
    mid <<= 32;
    z.v0 += mid;
    z.v64 += (z.v0 < mid);
    return z;
}

static float64_t f64_mul(float64_t a, float64_t b)
{
    uint_fast64_t uiA;
    bool signA;
    int_fast16_t expA;
    uint_fast64_t sigA;
    uint_fast64_t uiB;
    bool signB;
    int_fast16_t expB;
    uint_fast64_t sigB;
    bool signZ;
    uint_fast64_t magBits;
    struct exp16_sig64 normExpSig;
    int_fast16_t expZ;
    struct uint128 sig128Z;
    uint_fast64_t sigZ, uiZ;

    ///--------
    //------
    uiA = a.v;
    signA = signF64UI(uiA);
    expA = expF64UI(uiA);
    sigA = fracF64UI(uiA);
    uiB = b.v;
    signB = signF64UI(uiB);
    expB = expF64UI(uiB);
    sigB = fracF64UI(uiB);
    signZ = signA ^ signB;
    //-------------------------------------------------------------------
    //-----------------------------------------------------------------
    if (expA == 0x7FF) {
        if (sigA || ((expB == 0x7FF) && sigB)) goto propagateNaN;
        magBits = expB | sigB;
        goto infArg;
    }
    if (expB == 0x7FF) {
        if (sigB) goto propagateNaN;
        magBits = expA | sigA;
        goto infArg;
    }
    //--------------------------------------------------------------------
    //
    if (!expA) {
        if (!sigA) goto zero;
        normExpSig = softfloat_normSubnormalF64Sig(sigA);
        expA = normExpSig.exp;
        sigA = normExpSig.sig;
    }
    if (!expB) {
        if (!sigB) goto zero;
        normExpSig = softfloat_normSubnormalF64Sig(sigB);
        expB = normExpSig.exp;
        sigB = normExpSig.sig;
    }
    //--------------------------------------------------------------------
    //
    expZ = expA + expB - 0x3FF;
    sigA = (sigA | UINT64_C(0x0010000000000000)) << 10;
    sigB = (sigB | UINT64_C(0x0010000000000000)) << 11;
    sig128Z = softfloat_mul64To128(sigA, sigB);
    sigZ = sig128Z.v64 | (sig128Z.v0 != 0);

    if (sigZ < UINT64_C(0x4000000000000000)) {
        --expZ;
        sigZ <<= 1;
    }
    return softfloat_roundPackToF64(signZ, expZ, sigZ);
    //--------------------------------------------------------------------
    //
propagateNaN:
    uiZ = softfloat_propagateNaNF64UI(uiA, uiB);
    goto uiZ;
    //--------------------------------------------------------------------
    //
infArg:
    if (!magBits) {
        //raiseFlags(flag_invalid);
        uiZ = defaultNaNF64UI;
    }
    else {
        uiZ = packToF64UI(signZ, 0x7FF, 0);
    }
    goto uiZ;
    //--------------------------------------------------------------------
    //
zero:
    uiZ = packToF64UI(signZ, 0, 0);
uiZ:
    return float64_t::fromRaw(uiZ);
}

#define softfloat_approxRecip32_1( a ) ((uint32_t) (UINT64_C( 0x7FFFFFFFFFFFFFFF ) / (uint32_t) (a)))

static float64_t f64_div(float64_t a, float64_t b)
{
    uint_fast64_t uiA;
    bool signA;
    int_fast16_t expA;
    uint_fast64_t sigA;
    uint_fast64_t uiB;
    bool signB;
    int_fast16_t expB;
    uint_fast64_t sigB;
    bool signZ;
    struct exp16_sig64 normExpSig;
    int_fast16_t expZ;
    uint32_t recip32, sig32Z, doubleTerm;
    uint_fast64_t rem;
    uint32_t q;
    uint_fast64_t sigZ;
    uint_fast64_t uiZ;

    //--------------------------------------------------------------------
    //
    uiA = a.v;
    signA = signF64UI(uiA);
    expA = expF64UI(uiA);
    sigA = fracF64UI(uiA);
    uiB = b.v;
    signB = signF64UI(uiB);
    expB = expF64UI(uiB);
    sigB = fracF64UI(uiB);
    signZ = signA ^ signB;
    //--------------------------------------------------------------------
    //
    if (expA == 0x7FF) {
        if (sigA) goto propagateNaN;
        if (expB == 0x7FF) {
            if (sigB) goto propagateNaN;
            goto invalid;
        }
        goto infinity;
    }
    if (expB == 0x7FF) {
        if (sigB) goto propagateNaN;
        goto zero;
    }
    //--------------------------------------------------------------------
    //
    if (!expB) {
        if (!sigB) {
            if (!(expA | sigA)) goto invalid;
            //raiseFlags(flag_infinite);
            goto infinity;
        }
        normExpSig = softfloat_normSubnormalF64Sig(sigB);
        expB = normExpSig.exp;
        sigB = normExpSig.sig;
    }
    if (!expA) {
        if (!sigA) goto zero;
        normExpSig = softfloat_normSubnormalF64Sig(sigA);
        expA = normExpSig.exp;
        sigA = normExpSig.sig;
    }
    //--------------------------------------------------------------------
    //
    expZ = expA - expB + 0x3FE;
    sigA |= UINT64_C(0x0010000000000000);
    sigB |= UINT64_C(0x0010000000000000);
    if (sigA < sigB) {
        --expZ;
        sigA <<= 11;
    }
    else {
        sigA <<= 10;
    }
    sigB <<= 11;
    recip32 = softfloat_approxRecip32_1(sigB >> 32) - 2;
    sig32Z = ((uint32_t)(sigA >> 32) * (uint_fast64_t)recip32) >> 32;
    doubleTerm = sig32Z << 1;
    rem =
        ((sigA - (uint_fast64_t)doubleTerm * (uint32_t)(sigB >> 32)) << 28)
        - (uint_fast64_t)doubleTerm * ((uint32_t)sigB >> 4);
    q = (((uint32_t)(rem >> 32) * (uint_fast64_t)recip32) >> 32) + 4;
    sigZ = ((uint_fast64_t)sig32Z << 32) + ((uint_fast64_t)q << 4);
    //--------------------------------------------------------------------
    //
    if ((sigZ & 0x1FF) < 4 << 4) {
        q &= ~7;
        sigZ &= ~(uint_fast64_t)0x7F;
        doubleTerm = q << 1;
        rem =
            ((rem - (uint_fast64_t)doubleTerm * (uint32_t)(sigB >> 32)) << 28)
            - (uint_fast64_t)doubleTerm * ((uint32_t)sigB >> 4);
        if (rem & UINT64_C(0x8000000000000000)) {
            sigZ -= 1 << 7;
        }
        else {
            if (rem) sigZ |= 1;
        }
    }
    return softfloat_roundPackToF64(signZ, expZ, sigZ);
    //--------------------------------------------------------------------
    //
propagateNaN:
    uiZ = softfloat_propagateNaNF64UI(uiA, uiB);
    goto uiZ;
    //--------------------------------------------------------------------
    //
invalid:
    //raiseFlags(flag_invalid);
    uiZ = defaultNaNF64UI;
    goto uiZ;
    //--------------------------------------------------------------------
    //
infinity:
    uiZ = packToF64UI(signZ, 0x7FF, 0);
    goto uiZ;
    //--------------------------------------------------------------------
    //
zero:
    uiZ = packToF64UI(signZ, 0, 0);
uiZ:
    return float64_t::fromRaw(uiZ);
}

static bool f64_eq(float64_t a, float64_t b)
{
    uint_fast64_t uiA;
    uint_fast64_t uiB;

    uiA = a.v;
    uiB = b.v;
    if (isNaNF64UI(uiA) || isNaNF64UI(uiB))
    {
        if (softfloat_isSigNaNF64UI(uiA) || softfloat_isSigNaNF64UI(uiB))
            //raiseFlags(flag_invalid);
        return false;
    }
    return (uiA == uiB) || !((uiA | uiB) & UINT64_C(0x7FFFFFFFFFFFFFFF));
}


static inline uint_fast8_t softfloat_countLeadingZeros32(uint32_t a)
{
    uint_fast8_t count = 0;
    if (a < 0x10000) {
        count = 16;
        a <<= 16;
    }
    if (a < 0x1000000) {
        count += 8;
        a <<= 8;
    }
    count += softfloat_countLeadingZeros8[a >> 24];
    return count;
}

static float64_t i32_to_f64(int32_t a)
{
    uint_fast64_t uiZ;
    bool sign;
    uint_fast32_t absA;
    int_fast8_t shiftDist;

    if (!a) {
        uiZ = 0;
    }
    else {
        sign = (a < 0);
        //fixed unsigned unary minus: -x == ~x + 1
        absA = sign ? (~(uint_fast32_t)a + 1) : (uint_fast32_t)a;
        shiftDist = softfloat_countLeadingZeros32(absA) + 21;
        uiZ =
            packToF64UI(
                sign, 0x432 - shiftDist, (uint_fast64_t)absA << shiftDist);
    }
    return float64_t::fromRaw(uiZ);
}

static float64_t i64_to_f64(int64_t a)
{
    bool sign;
    uint_fast64_t absA;

    sign = (a < 0);
    if (!(a & UINT64_C(0x7FFFFFFFFFFFFFFF))) {
        return float64_t::fromRaw(sign ? packToF64UI(1, 0x43E, 0) : 0);
    }
    //fixed unsigned unary minus: -x == ~x + 1
    absA = sign ? (~(uint_fast64_t)a + 1) : (uint_fast64_t)a;
    return softfloat_normRoundPackToF64(sign, 0x43C, absA);
}


softdouble softdouble::operator + (const softdouble& a) const { return f64_add(*this, a); }
softdouble softdouble::operator - (const softdouble& a) const { return f64_sub(*this, a); }
softdouble softdouble::operator * (const softdouble& a) const { return f64_mul(*this, a); }
softdouble softdouble::operator / (const softdouble& a) const { return f64_div(*this, a); }
//softdouble softdouble::operator % (const softdouble& a) const { return f64_rem(*this, a); }
bool softdouble::operator == (const softdouble& a) const { return  f64_eq(*this, a); }

softdouble::softdouble(const  int32_t a) { *this = i32_to_f64(a); }
softdouble::softdouble(const  int64_t a) { *this = i64_to_f64(a); }





static int_fast64_t
softfloat_roundToI64(
    bool sign, uint_fast64_t sig, uint_fast64_t sigExtra, uint_fast8_t roundingMode, bool exact)
{
    bool roundNearEven, doIncrement;
    union { uint64_t ui; int64_t i; } uZ;
    int_fast64_t z;

    //--------------------------------------------------------------------
    //
    roundNearEven = (roundingMode == round_near_even);
    doIncrement = (UINT64_C(0x8000000000000000) <= sigExtra);
    if (!roundNearEven && (roundingMode != round_near_maxMag)) {
        doIncrement =
            (roundingMode
                == (sign ? round_min : round_max))
            && sigExtra;
    }
    if (doIncrement) {
        ++sig;
        if (!sig) goto invalid;
        sig &=
            ~(uint_fast64_t)
            (!(sigExtra & UINT64_C(0x7FFFFFFFFFFFFFFF))
                & roundNearEven);
    }
    uZ.ui = sign ? (~sig + 1) : sig;
    z = uZ.i;
    if (z && ((z < 0) ^ sign)) goto invalid;
    if (exact && sigExtra) {
        // raiseFlags(flag_inexact);
    }
    return z;
    //--------------------------------------------------------------------
    //
invalid:
    //raiseFlags(flag_invalid);
    return sign ? i64_fromNegOverflow : i64_fromPosOverflow;
}

static int_fast64_t f64_to_i64(float64_t a, uint_fast8_t roundingMode, bool exact)
{
    uint_fast64_t uiA;
    bool sign;
    int_fast16_t exp;
    uint_fast64_t sig;
    int_fast16_t shiftDist;

    //--------------------------------------------------------------------
    //
    uiA = a.v;
    sign = signF64UI(uiA);
    exp = expF64UI(uiA);
    sig = fracF64UI(uiA);
    //--------------------------------------------------------------------
    //
#if (i64_fromNaN != i64_fromPosOverflow) || (i64_fromNaN != i64_fromNegOverflow)
    if ((exp == 0x7FF) && sig) {
#if (i64_fromNaN == i64_fromPosOverflow)
        sign = 0;
#elif (i64_fromNaN == i64_fromNegOverflow)
        sign = 1;
#else
        //raiseFlags(flag_invalid);
        return i64_fromNaN;
#endif
    }
#endif
    //--------------------------------------------------------------------
    //
    if (exp) sig |= UINT64_C(0x0010000000000000);
    shiftDist = 0x433 - exp;
    if (shiftDist <= 0) {
        bool isValid = shiftDist >= -11;
        if (isValid)
        {
            uint_fast64_t z = sig << -shiftDist;
            if (0 == (z & UINT64_C(0x8000000000000000)))
            {
                return sign ? -(int_fast64_t)z : (int_fast64_t)z;
            }
        }
        //raiseFlags(flag_invalid);
        return sign ? i64_fromNegOverflow : i64_fromPosOverflow;
    }
    else {
        if (shiftDist < 64)
            return
            softfloat_roundToI64(
                sign, sig >> shiftDist, sig << (-shiftDist & 63), roundingMode, exact);
        else
            return
            softfloat_roundToI64(
                sign, 0, (shiftDist == 64) ? sig : (sig != 0), roundingMode, exact);
    }
}

int64_t cvRound64_(const softdouble& a) { return f64_to_i64(a, round_near_even, false); }

class fixedpoint64
{
private:
    int64_t val;
    fixedpoint64(int64_t _val) : val(_val) {}
    static CV_ALWAYS_INLINE uint64_t fixedround(const uint64_t& _val) { return (_val + ((1LL << fixedShift) >> 1)); }
public:
    static const int fixedShift = 32;

    typedef fixedpoint64 WT;
    typedef int64_t raw_t;
    CV_ALWAYS_INLINE fixedpoint64() { val = 0; }
    CV_ALWAYS_INLINE fixedpoint64(const fixedpoint64& v) { val = v.val; }
    CV_ALWAYS_INLINE fixedpoint64(const int8_t& _val) { val = ((int64_t)_val) << fixedShift; }
    CV_ALWAYS_INLINE fixedpoint64(const uint8_t& _val) { val = ((int64_t)_val) << fixedShift; }
    CV_ALWAYS_INLINE fixedpoint64(const int16_t& _val) { val = ((int64_t)_val) << fixedShift; }
    CV_ALWAYS_INLINE fixedpoint64(const uint16_t& _val) { val = ((int64_t)_val) << fixedShift; }
    CV_ALWAYS_INLINE fixedpoint64(const int32_t& _val) { val = ((int64_t)_val) << fixedShift; }
    CV_ALWAYS_INLINE fixedpoint64(const softdouble& _val) { val = cvRound64_(_val * softdouble((int64_t)(1LL << fixedShift))); }
    CV_ALWAYS_INLINE fixedpoint64& operator = (const int8_t& _val) { val = ((int64_t)_val) << fixedShift; return *this; }
    CV_ALWAYS_INLINE fixedpoint64& operator = (const uint8_t& _val) { val = ((int64_t)_val) << fixedShift; return *this; }
    CV_ALWAYS_INLINE fixedpoint64& operator = (const int16_t& _val) { val = ((int64_t)_val) << fixedShift; return *this; }
    CV_ALWAYS_INLINE fixedpoint64& operator = (const uint16_t& _val) { val = ((int64_t)_val) << fixedShift; return *this; }
    CV_ALWAYS_INLINE fixedpoint64& operator = (const int32_t& _val) { val = ((int64_t)_val) << fixedShift; return *this; }
    CV_ALWAYS_INLINE fixedpoint64& operator = (const softdouble& _val) { val = cvRound64_(_val * softdouble((int64_t)(1LL << fixedShift))); return *this; }
    CV_ALWAYS_INLINE fixedpoint64& operator = (const fixedpoint64& _val) { val = _val.val; return *this; }
    CV_ALWAYS_INLINE fixedpoint64 operator * (const int8_t& val2) const { return operator *(fixedpoint64(val2)); }
    CV_ALWAYS_INLINE fixedpoint64 operator * (const uint8_t& val2) const { return operator *(fixedpoint64(val2)); }
    CV_ALWAYS_INLINE fixedpoint64 operator * (const int16_t& val2) const { return operator *(fixedpoint64(val2)); }
    CV_ALWAYS_INLINE fixedpoint64 operator * (const uint16_t& val2) const { return operator *(fixedpoint64(val2)); }
    CV_ALWAYS_INLINE fixedpoint64 operator * (const int32_t& val2) const { return operator *(fixedpoint64(val2)); }
    CV_ALWAYS_INLINE fixedpoint64 operator * (const fixedpoint64& val2) const
    {
        bool sign_val = val < 0;
        bool sign_mul = val2.val < 0;
        uint64_t uval = sign_val ? (uint64_t)(-val) : (uint64_t)val;
        uint64_t umul = sign_mul ? (uint64_t)(-val2.val) : (uint64_t)val2.val;
        bool ressign = sign_val ^ sign_mul;

        uint64_t sh0 = fixedround((uval & 0xFFFFFFFF) * (umul & 0xFFFFFFFF));
        uint64_t sh1_0 = (uval >> 32) * (umul & 0xFFFFFFFF);
        uint64_t sh1_1 = (uval & 0xFFFFFFFF) * (umul >> 32);
        uint64_t sh2 = (uval >> 32) * (umul >> 32);
        uint64_t val0_l = (sh1_0 & 0xFFFFFFFF) + (sh1_1 & 0xFFFFFFFF) + (sh0 >> 32);
        uint64_t val0_h = (sh2 & 0xFFFFFFFF) + (sh1_0 >> 32) + (sh1_1 >> 32) + (val0_l >> 32);
        val0_l &= 0xFFFFFFFF;

        if (sh2 > CV_BIG_INT(0x7FFFFFFF) || val0_h > CV_BIG_INT(0x7FFFFFFF))
            return (int64_t)(ressign ? CV_BIG_UINT(0x8000000000000000) : CV_BIG_INT(0x7FFFFFFFFFFFFFFF));

        if (ressign)
        {
            return -(int64_t)(val0_h << 32 | val0_l);
        }
        return (int64_t)(val0_h << 32 | val0_l);
    }
    CV_ALWAYS_INLINE fixedpoint64 operator + (const fixedpoint64& val2) const
    {
        int64_t res = val + val2.val;
        return (int64_t)(((val ^ res) & (val2.val ^ res)) < 0 ? ~(res & CV_BIG_UINT(0x8000000000000000)) : res);
    }
    CV_ALWAYS_INLINE fixedpoint64 operator - (const fixedpoint64& val2) const
    {
        int64_t res = val - val2.val;
        return (int64_t)(((val ^ val2.val) & (val ^ res)) < 0 ? ~(res & CV_BIG_UINT(0x8000000000000000)) : res);
    }
    CV_ALWAYS_INLINE fixedpoint64 operator >> (int n) const { return fixedpoint64(val >> n); }
    CV_ALWAYS_INLINE fixedpoint64 operator << (int n) const { return fixedpoint64(val << n); }
    CV_ALWAYS_INLINE bool operator == (const fixedpoint64& val2) const { return val == val2.val; }
    template <typename ET>
    CV_ALWAYS_INLINE ET saturate_cast_() const { return saturate_cast<ET>((int64_t)fixedround((uint64_t)val) >> fixedShift); }
    CV_ALWAYS_INLINE operator double() const { return (double)val / (1LL << fixedShift); }
    CV_ALWAYS_INLINE operator float() const { return (float)val / (1LL << fixedShift); }
    CV_ALWAYS_INLINE operator uint8_t() const { return saturate_cast_<uint8_t>(); }
    CV_ALWAYS_INLINE operator int8_t() const { return saturate_cast_<int8_t>(); }
    CV_ALWAYS_INLINE operator uint16_t() const { return saturate_cast_<uint16_t>(); }
    CV_ALWAYS_INLINE operator int16_t() const { return saturate_cast_<int16_t>(); }
    CV_ALWAYS_INLINE operator int32_t() const { return saturate_cast_<int32_t>(); }
    CV_ALWAYS_INLINE bool isZero() { return val == 0; }
    static CV_ALWAYS_INLINE fixedpoint64 zero() { return fixedpoint64(); }
    static CV_ALWAYS_INLINE fixedpoint64 one() { return fixedpoint64((int64_t)(1LL << fixedShift)); }
    friend class fixedpoint32;
};
class ufixedpoint64
{
private:
    uint64_t val;
    ufixedpoint64(uint64_t _val) : val(_val) {}
    static CV_ALWAYS_INLINE uint64_t fixedround(const uint64_t& _val) { return (_val + ((1LL << fixedShift) >> 1)); }
public:
    static const int fixedShift = 32;

    typedef ufixedpoint64 WT;
    typedef uint64_t raw_t;
    CV_ALWAYS_INLINE ufixedpoint64() { val = 0; }
    CV_ALWAYS_INLINE ufixedpoint64(const ufixedpoint64& v) { val = v.val; }
    CV_ALWAYS_INLINE ufixedpoint64(const uint8_t& _val) { val = ((uint64_t)_val) << fixedShift; }
    CV_ALWAYS_INLINE ufixedpoint64(const uint16_t& _val) { val = ((uint64_t)_val) << fixedShift; }
    CV_ALWAYS_INLINE ufixedpoint64(const uint32_t& _val) { val = ((uint64_t)_val) << fixedShift; }
    CV_ALWAYS_INLINE ufixedpoint64(const softdouble& _val) { val = _val.getSign() ? 0 : (uint64_t)cvRound64_(_val * softdouble((int64_t)(1LL << fixedShift))); }
    CV_ALWAYS_INLINE ufixedpoint64& operator = (const uint8_t& _val) { val = ((uint64_t)_val) << fixedShift; return *this; }
    CV_ALWAYS_INLINE ufixedpoint64& operator = (const uint16_t& _val) { val = ((uint64_t)_val) << fixedShift; return *this; }
    CV_ALWAYS_INLINE ufixedpoint64& operator = (const uint32_t& _val) { val = ((uint64_t)_val) << fixedShift; return *this; }
    CV_ALWAYS_INLINE ufixedpoint64& operator = (const softdouble& _val) { val = _val.getSign() ? 0 : (uint64_t)cvRound64_(_val * softdouble((int64_t)(1LL << fixedShift))); return *this; }
    CV_ALWAYS_INLINE ufixedpoint64& operator = (const ufixedpoint64& _val) { val = _val.val; return *this; }
    CV_ALWAYS_INLINE ufixedpoint64 operator * (const uint8_t& val2) const { return operator *(ufixedpoint64(val2)); }
    CV_ALWAYS_INLINE ufixedpoint64 operator * (const uint16_t& val2) const { return operator *(ufixedpoint64(val2)); }
    CV_ALWAYS_INLINE ufixedpoint64 operator * (const uint32_t& val2) const { return operator *(ufixedpoint64(val2)); }
    CV_ALWAYS_INLINE ufixedpoint64 operator * (const ufixedpoint64& val2) const
    {
        uint64_t sh0 = fixedround((val & 0xFFFFFFFF) * (val2.val & 0xFFFFFFFF));
        uint64_t sh1_0 = (val >> 32) * (val2.val & 0xFFFFFFFF);
        uint64_t sh1_1 = (val & 0xFFFFFFFF) * (val2.val >> 32);
        uint64_t sh2 = (val >> 32) * (val2.val >> 32);
        uint64_t val0_l = (sh1_0 & 0xFFFFFFFF) + (sh1_1 & 0xFFFFFFFF) + (sh0 >> 32);
        uint64_t val0_h = (sh2 & 0xFFFFFFFF) + (sh1_0 >> 32) + (sh1_1 >> 32) + (val0_l >> 32);
        val0_l &= 0xFFFFFFFF;

        if (sh2 > CV_BIG_INT(0xFFFFFFFF) || val0_h > CV_BIG_INT(0xFFFFFFFF))
            return (uint64_t)CV_BIG_UINT(0xFFFFFFFFFFFFFFFF);

        return (val0_h << 32 | val0_l);
    }
    CV_ALWAYS_INLINE ufixedpoint64 operator + (const ufixedpoint64& val2) const
    {
        uint64_t res = val + val2.val;
        return (uint64_t)((val > res) ? CV_BIG_UINT(0xFFFFFFFFFFFFFFFF) : res);
    }
    CV_ALWAYS_INLINE ufixedpoint64 operator - (const ufixedpoint64& val2) const
    {
        return val > val2.val ? (val - val2.val) : 0;
    }
    CV_ALWAYS_INLINE ufixedpoint64 operator >> (int n) const { return ufixedpoint64(val >> n); }
    CV_ALWAYS_INLINE ufixedpoint64 operator << (int n) const { return ufixedpoint64(val << n); }
    CV_ALWAYS_INLINE bool operator == (const ufixedpoint64& val2) const { return val == val2.val; }
    template <typename ET>
    CV_ALWAYS_INLINE ET saturate_cast_() const { return saturate_cast<ET>(fixedround(val) >> fixedShift); }
    CV_ALWAYS_INLINE operator double() const { return (double)val / (1LL << fixedShift); }
    CV_ALWAYS_INLINE operator float() const { return (float)val / (1LL << fixedShift); }
    CV_ALWAYS_INLINE operator uint8_t() const { return saturate_cast_<uint8_t>(); }
    CV_ALWAYS_INLINE operator int8_t() const { return saturate_cast_<int8_t>(); }
    CV_ALWAYS_INLINE operator uint16_t() const { return saturate_cast_<uint16_t>(); }
    CV_ALWAYS_INLINE operator int16_t() const { return saturate_cast_<int16_t>(); }
    CV_ALWAYS_INLINE operator int32_t() const { return saturate_cast_<int32_t>(); }

    CV_ALWAYS_INLINE bool isZero() { return val == 0; }
    static CV_ALWAYS_INLINE ufixedpoint64 zero() { return ufixedpoint64(); }
    static CV_ALWAYS_INLINE ufixedpoint64 one() { return ufixedpoint64((uint64_t)(1ULL << fixedShift)); }

    static CV_ALWAYS_INLINE ufixedpoint64 fromRaw(uint64_t v) { return ufixedpoint64(v); }
    CV_ALWAYS_INLINE uint64_t raw() { return val; }
   // CV_ALWAYS_INLINE uint32_t cvFloor() const { return saturate_cast<uint32_t>(val >> fixedShift); }
    friend class ufixedpoint32;
};

class fixedpoint32
{
private:
    int32_t val;
    fixedpoint32(int32_t _val) : val(_val) {}
    static CV_ALWAYS_INLINE uint32_t fixedround(const uint32_t& _val) { return (_val + ((1 << fixedShift) >> 1)); }
public:
    static const int fixedShift = 16;

    typedef fixedpoint64 WT;
    typedef int32_t raw_t;
    CV_ALWAYS_INLINE fixedpoint32() { val = 0; }
    CV_ALWAYS_INLINE fixedpoint32(const fixedpoint32& v) { val = v.val; }
    CV_ALWAYS_INLINE fixedpoint32(const int8_t& _val) { val = ((int32_t)_val) << fixedShift; }
    CV_ALWAYS_INLINE fixedpoint32(const uint8_t& _val) { val = ((int32_t)_val) << fixedShift; }
    CV_ALWAYS_INLINE fixedpoint32(const int16_t& _val) { val = ((int32_t)_val) << fixedShift; }
    CV_ALWAYS_INLINE fixedpoint32(const softdouble& _val) { val = (int32_t)cvRound(_val * softdouble((1 << fixedShift))); }
    CV_ALWAYS_INLINE fixedpoint32& operator = (const int8_t& _val) { val = ((int32_t)_val) << fixedShift; return *this; }
    CV_ALWAYS_INLINE fixedpoint32& operator = (const uint8_t& _val) { val = ((int32_t)_val) << fixedShift; return *this; }
    CV_ALWAYS_INLINE fixedpoint32& operator = (const int16_t& _val) { val = ((int32_t)_val) << fixedShift; return *this; }
    CV_ALWAYS_INLINE fixedpoint32& operator = (const softdouble& _val) { val = (int32_t)cvRound(_val * softdouble((1 << fixedShift))); return *this; }
    CV_ALWAYS_INLINE fixedpoint32& operator = (const fixedpoint32& _val) { val = _val.val; return *this; }
    //CV_ALWAYS_INLINE fixedpoint32 operator * (const int8_t& val2) const { return saturate_cast<int32_t>((int64_t)val * val2); }
    CV_ALWAYS_INLINE fixedpoint32 operator * (const uint8_t& val2) const { return saturate_cast<int32_t>((int64_t)val * val2); }
    //CV_ALWAYS_INLINE fixedpoint32 operator * (const int16_t& val2) const { return saturate_cast<int32_t>((int64_t)val * val2); }
    CV_ALWAYS_INLINE fixedpoint64 operator * (const fixedpoint32& val2) const { return (int64_t)val * (int64_t)(val2.val); }
    CV_ALWAYS_INLINE fixedpoint32 operator + (const fixedpoint32& val2) const
    {
        int32_t res = val + val2.val;
        return (int64_t)((val ^ res) & (val2.val ^ res)) >> 31 ? ~(res & ~0x7FFFFFFF) : res;
    }
    CV_ALWAYS_INLINE fixedpoint32 operator - (const fixedpoint32& val2) const
    {
        int32_t res = val - val2.val;
        return (int64_t)((val ^ val2.val) & (val ^ res)) >> 31 ? ~(res & ~0x7FFFFFFF) : res;
    }
    CV_ALWAYS_INLINE fixedpoint32 operator >> (int n) const { return fixedpoint32(val >> n); }
    CV_ALWAYS_INLINE fixedpoint32 operator << (int n) const { return fixedpoint32(val << n); }
    CV_ALWAYS_INLINE bool operator == (const fixedpoint32& val2) const { return val == val2.val; }
    template <typename ET>
    CV_ALWAYS_INLINE ET saturate_cast_() const { return saturate_cast<ET>((int32_t)fixedround((uint32_t)val) >> fixedShift); }
    CV_ALWAYS_INLINE operator fixedpoint64() const { return (int64_t)val << (fixedpoint64::fixedShift - fixedShift); }
    CV_ALWAYS_INLINE operator double() const { return (double)val / (1 << fixedShift); }
    CV_ALWAYS_INLINE operator float() const { return (float)val / (1 << fixedShift); }
    CV_ALWAYS_INLINE operator uint8_t() const { return saturate_cast_<uint8_t>(); }
    CV_ALWAYS_INLINE operator int8_t() const { return saturate_cast_<int8_t>(); }
    CV_ALWAYS_INLINE operator uint16_t() const { return saturate_cast_<uint16_t>(); }
    CV_ALWAYS_INLINE operator int16_t() const { return saturate_cast_<int16_t>(); }
    CV_ALWAYS_INLINE operator int32_t() const { return saturate_cast_<int32_t>(); }
    CV_ALWAYS_INLINE bool isZero() { return val == 0; }
    static CV_ALWAYS_INLINE fixedpoint32 zero() { return fixedpoint32(); }
    static CV_ALWAYS_INLINE fixedpoint32 one() { return fixedpoint32((1 << fixedShift)); }
    friend class fixedpoint16;
};

class ufixedpoint32
{
private:
    uint32_t val;
    ufixedpoint32(uint32_t _val) : val(_val) {}
    static CV_ALWAYS_INLINE uint32_t fixedround(const uint32_t& _val) { return (_val + ((1 << fixedShift) >> 1)); }
public:
    static const int fixedShift = 16;

    typedef ufixedpoint64 WT;
    typedef uint32_t raw_t;
    CV_ALWAYS_INLINE ufixedpoint32() { val = 0; }
    CV_ALWAYS_INLINE ufixedpoint32(const ufixedpoint32& v) { val = v.val; }
    CV_ALWAYS_INLINE ufixedpoint32(const uint8_t& _val) { val = ((uint32_t)_val) << fixedShift; }
    CV_ALWAYS_INLINE ufixedpoint32(const uint16_t& _val) { val = ((uint32_t)_val) << fixedShift; }
    CV_ALWAYS_INLINE ufixedpoint32(const softdouble& _val) { val = _val.getSign() ? 0 : (uint32_t)cvRound(_val * softdouble((1 << fixedShift))); }
    CV_ALWAYS_INLINE ufixedpoint32& operator = (const uint8_t& _val) { val = ((uint32_t)_val) << fixedShift; return *this; }
    CV_ALWAYS_INLINE ufixedpoint32& operator = (const uint16_t& _val) { val = ((uint32_t)_val) << fixedShift; return *this; }
    CV_ALWAYS_INLINE ufixedpoint32& operator = (const softdouble& _val) { val = _val.getSign() ? 0 : (uint32_t)cvRound(_val * softdouble((1 << fixedShift))); return *this; }
    CV_ALWAYS_INLINE ufixedpoint32& operator = (const ufixedpoint32& _val) { val = _val.val; return *this; }
    CV_ALWAYS_INLINE ufixedpoint32 operator * (const uint8_t& val2) const { return saturate_cast<uint32_t>((uint64_t)val * val2); }
    //CV_ALWAYS_INLINE ufixedpoint32 operator * (const uint16_t& val2) const { return saturate_cast<uint32_t>((uint64_t)val * val2); }
    CV_ALWAYS_INLINE ufixedpoint64 operator * (const ufixedpoint32& val2) const { return (uint64_t)val * (uint64_t)(val2.val); }
    CV_ALWAYS_INLINE ufixedpoint32 operator + (const ufixedpoint32& val2) const
    {
        uint32_t res = val + val2.val;
        return (val > res) ? 0xFFFFFFFF : res;
    }
    CV_ALWAYS_INLINE ufixedpoint32 operator - (const ufixedpoint32& val2) const
    {
        return val > val2.val ? (val - val2.val) : 0;
    }
    CV_ALWAYS_INLINE ufixedpoint32 operator >> (int n) const { return ufixedpoint32(val >> n); }
    CV_ALWAYS_INLINE ufixedpoint32 operator << (int n) const { return ufixedpoint32(val << n); }
    CV_ALWAYS_INLINE bool operator == (const ufixedpoint32& val2) const { return val == val2.val; }
    template <typename ET>
    CV_ALWAYS_INLINE ET saturate_cast_() const { return saturate_cast<ET>(fixedround(val) >> fixedShift); }
    CV_ALWAYS_INLINE operator ufixedpoint64() const { return (uint64_t)val << (ufixedpoint64::fixedShift - fixedShift); }
    CV_ALWAYS_INLINE operator double() const { return (double)val / (1 << fixedShift); }
    CV_ALWAYS_INLINE operator float() const { return (float)val / (1 << fixedShift); }
    CV_ALWAYS_INLINE operator uint8_t() const { return saturate_cast_<uint8_t>(); }
    CV_ALWAYS_INLINE operator int8_t() const { return saturate_cast_<int8_t>(); }
    CV_ALWAYS_INLINE operator uint16_t() const { return saturate_cast_<uint16_t>(); }
    CV_ALWAYS_INLINE operator int16_t() const { return saturate_cast_<int16_t>(); }
    CV_ALWAYS_INLINE operator int32_t() const { return saturate_cast_<int32_t>(); }
    CV_ALWAYS_INLINE bool isZero() { return val == 0; }
    static CV_ALWAYS_INLINE ufixedpoint32 zero() { return ufixedpoint32(); }
    static CV_ALWAYS_INLINE ufixedpoint32 one() { return ufixedpoint32((1U << fixedShift)); }

    static CV_ALWAYS_INLINE ufixedpoint32 fromRaw(uint32_t v) { return ufixedpoint32(v); }
    CV_ALWAYS_INLINE uint32_t raw() { return val; }
    friend class ufixedpoint16;
};

class ufixedpoint16
{
private:
    uint16_t val;
    ufixedpoint16(uint16_t _val) : val(_val) {}
    static CV_ALWAYS_INLINE uint16_t fixedround(const uint16_t& _val) { return (_val + ((1 << fixedShift) >> 1)); }
public:
    static const int fixedShift = 8;

    typedef ufixedpoint32 WT;
    typedef uint16_t raw_t;
    CV_ALWAYS_INLINE ufixedpoint16() { val = 0; }
    CV_ALWAYS_INLINE ufixedpoint16(const ufixedpoint16& v) { val = v.val; }
    CV_ALWAYS_INLINE ufixedpoint16(const uint8_t& _val) { val = ((uint16_t)_val) << fixedShift; }
    CV_ALWAYS_INLINE ufixedpoint16(const softdouble& _val) { val = _val.getSign() ? 0 : (uint16_t)cvRound(_val * softdouble((int32_t)(1 << fixedShift))); }
    CV_ALWAYS_INLINE ufixedpoint16& operator = (const uint8_t& _val) { val = ((uint16_t)_val) << fixedShift; return *this; }
    CV_ALWAYS_INLINE ufixedpoint16& operator = (const softdouble& _val) { val = _val.getSign() ? 0 : (uint16_t)cvRound(_val * softdouble((int32_t)(1 << fixedShift))); return *this; }
    CV_ALWAYS_INLINE ufixedpoint16& operator = (const ufixedpoint16& _val) { val = _val.val; return *this; }
    CV_ALWAYS_INLINE ufixedpoint16 operator * (const uint8_t& val2) const { return saturate_cast<uint16_t>((uint32_t)val * val2); }
    CV_ALWAYS_INLINE ufixedpoint32 operator * (const ufixedpoint16& val2) const { return ((uint32_t)val * (uint32_t)(val2.val)); }
    CV_ALWAYS_INLINE ufixedpoint16 operator + (const ufixedpoint16& val2) const
    {
        uint16_t res = val + val2.val;
        return (val > res) ? (uint16_t)0xFFFF : res;
    }
    CV_ALWAYS_INLINE ufixedpoint16 operator - (const ufixedpoint16& val2) const
    {
        return val > val2.val ? (uint16_t)(val - val2.val) : (uint16_t)0;
    }
    CV_ALWAYS_INLINE ufixedpoint16 operator >> (int n) const { return ufixedpoint16((uint16_t)(val >> n)); }
    CV_ALWAYS_INLINE ufixedpoint16 operator << (int n) const { return ufixedpoint16((uint16_t)(val << n)); }
    CV_ALWAYS_INLINE bool operator == (const ufixedpoint16& val2) const { return val == val2.val; }
    template <typename ET>
    CV_ALWAYS_INLINE ET saturate_cast_() const { return saturate_cast<ET>(fixedround(val) >> fixedShift); }
    CV_ALWAYS_INLINE operator ufixedpoint32() const { return (uint32_t)val << (ufixedpoint32::fixedShift - fixedShift); }
    CV_ALWAYS_INLINE operator double() const { return (double)val / (1 << fixedShift); }
    CV_ALWAYS_INLINE operator float() const { return (float)val / (1 << fixedShift); }
    CV_ALWAYS_INLINE operator uint8_t() const { return saturate_cast_<uint8_t>(); }
    CV_ALWAYS_INLINE operator int8_t() const { return saturate_cast_<int8_t>(); }
    CV_ALWAYS_INLINE operator uint16_t() const { return saturate_cast_<uint16_t>(); }
    CV_ALWAYS_INLINE operator int16_t() const { return saturate_cast_<int16_t>(); }
    CV_ALWAYS_INLINE operator int32_t() const { return saturate_cast_<int32_t>(); }
    CV_ALWAYS_INLINE bool isZero() { return val == 0; }
    static CV_ALWAYS_INLINE ufixedpoint16 zero() { return ufixedpoint16(); }
    static CV_ALWAYS_INLINE ufixedpoint16 one() { return ufixedpoint16((uint16_t)(1 << fixedShift)); }

    static CV_ALWAYS_INLINE ufixedpoint16 fromRaw(uint16_t v) { return ufixedpoint16(v); }
    CV_ALWAYS_INLINE uint16_t raw() { return val; }
};



    template <typename ET, bool needsign> struct fixedtype { typedef fixedpoint64 type; };
    template <> struct fixedtype<uint32_t, false> { typedef ufixedpoint64 type; };

    template <bool needsign> struct fixedtype<int8_t, needsign> { typedef fixedpoint32 type; };
    template <> struct fixedtype<uint8_t, false> { typedef ufixedpoint16 type; };
   

    //FT is fixedtype<ET, needsign>::type
    template <typename ET, typename FT, int n, bool mulall>
    static void hlineResize(ET* src, int cn, int* ofst, FT* m, FT* dst, int dst_min, int dst_max, int dst_width)
    {
        int i = 0;
        for (; i < dst_min; i++, m += n) // Points that fall left from src image so became equal to leftmost src point
        {
            for (int j = 0; j < cn; j++, dst++)
            {
                *dst = src[j];
            }
        }
        for (; i < dst_max; i++, m += n)
        {
            ET* src_ofst = src + cn * ofst[i];
            for (int j = 0; j < cn; j++, dst++)
            {
                *dst = (mulall || !m[0].isZero()) ? m[0] * src_ofst[j] : FT::zero();
                for (int k = 1; k < n; k++)
                {
                    *dst = *dst + ((mulall || !m[k].isZero()) ? m[k] * src_ofst[j + k * cn] : FT::zero());
                }
            }
        }
        ET* src_last = src + cn * ofst[dst_width - 1];
        for (; i < dst_width; i++) // Points that fall right from src image so became equal to rightmost src point
        {
            for (int j = 0; j < cn; j++, dst++)
            {
                *dst = src_last[j];
            }
        }
    }
    template <typename ET, typename FT, int n, bool mulall, int cncnt> struct hline
    {
        static void ResizeCn(ET* src, int cn, int* ofst, FT* m, FT* dst, int dst_min, int dst_max, int dst_width)
        {
            hlineResize<ET, FT, n, mulall>(src, cn, ofst, m, dst, dst_min, dst_max, dst_width);
        }
    };
    template <typename ET, typename FT> struct hline<ET, FT, 2, true, 1>
    {
        static void ResizeCn(ET* src, int, int* ofst, FT* m, FT* dst, int dst_min, int dst_max, int dst_width)
        {
            int i = 0;
            FT src0(src[0]);
            for (; i < dst_min; i++, m += 2) // Points that fall left from src image so became equal to leftmost src point
            {
                *(dst++) = src0;
            }
            for (; i < dst_max; i++, m += 2)
            {
                ET* px = src + ofst[i];
                *(dst++) = m[0] * px[0] + m[1] * px[1];
            }
            src0 = (src + ofst[dst_width - 1])[0];
            for (; i < dst_width; i++) // Points that fall right from src image so became equal to rightmost src point
            {
                *(dst++) = src0;
            }
        }
    };
    /*
    template <typename ET, typename FT> struct hline<ET, FT, 2, true, 2>
    {
        static void ResizeCn(ET* src, int, int* ofst, FT* m, FT* dst, int dst_min, int dst_max, int dst_width)
        {
            int i = 0;
            FT src0(src[0]), src1(src[1]);
            for (; i < dst_min; i++, m += 2) // Points that fall left from src image so became equal to leftmost src point
            {
                *(dst++) = src0;
                *(dst++) = src1;
            }
            for (; i < dst_max; i++, m += 2)
            {
                ET* px = src + 2 * ofst[i];
                *(dst++) = m[0] * px[0] + m[1] * px[2];
                *(dst++) = m[0] * px[1] + m[1] * px[3];
            }
            src0 = (src + 2 * ofst[dst_width - 1])[0];
            src1 = (src + 2 * ofst[dst_width - 1])[1];
            for (; i < dst_width; i++) // Points that fall right from src image so became equal to rightmost src point
            {
                *(dst++) = src0;
                *(dst++) = src1;
            }
        }
    };
    template <typename ET, typename FT> struct hline<ET, FT, 2, true, 3>
    {
        static void ResizeCn(ET* src, int, int* ofst, FT* m, FT* dst, int dst_min, int dst_max, int dst_width)
        {
            int i = 0;
            FT src0(src[0]), src1(src[1]), src2(src[2]);
            for (; i < dst_min; i++, m += 2) // Points that fall left from src image so became equal to leftmost src point
            {
                *(dst++) = src0;
                *(dst++) = src1;
                *(dst++) = src2;
            }
            for (; i < dst_max; i++, m += 2)
            {
                ET* px = src + 3 * ofst[i];
                *(dst++) = m[0] * px[0] + m[1] * px[3];
                *(dst++) = m[0] * px[1] + m[1] * px[4];
                *(dst++) = m[0] * px[2] + m[1] * px[5];
            }
            src0 = (src + 3 * ofst[dst_width - 1])[0];
            src1 = (src + 3 * ofst[dst_width - 1])[1];
            src2 = (src + 3 * ofst[dst_width - 1])[2];
            for (; i < dst_width; i++) // Points that fall right from src image so became equal to rightmost src point
            {
                *(dst++) = src0;
                *(dst++) = src1;
                *(dst++) = src2;
            }
        }
    };
    template <typename ET, typename FT> struct hline<ET, FT, 2, true, 4>
    {
        static void ResizeCn(ET* src, int, int* ofst, FT* m, FT* dst, int dst_min, int dst_max, int dst_width)
        {
            int i = 0;
            FT src0(src[0]), src1(src[1]), src2(src[2]), src3(src[3]);
            for (; i < dst_min; i++, m += 2) // Points that fall left from src image so became equal to leftmost src point
            {
                *(dst++) = src0;
                *(dst++) = src1;
                *(dst++) = src2;
                *(dst++) = src3;
            }
            for (; i < dst_max; i++, m += 2)
            {
                ET* px = src + 4 * ofst[i];
                *(dst++) = m[0] * px[0] + m[1] * px[4];
                *(dst++) = m[0] * px[1] + m[1] * px[5];
                *(dst++) = m[0] * px[2] + m[1] * px[6];
                *(dst++) = m[0] * px[3] + m[1] * px[7];
            }
            src0 = (src + 4 * ofst[dst_width - 1])[0];
            src1 = (src + 4 * ofst[dst_width - 1])[1];
            src2 = (src + 4 * ofst[dst_width - 1])[2];
            src3 = (src + 4 * ofst[dst_width - 1])[3];
            for (; i < dst_width; i++) // Points that fall right from src image so became equal to rightmost src point
            {
                *(dst++) = src0;
                *(dst++) = src1;
                *(dst++) = src2;
                *(dst++) = src3;
            }
        }
    };
    template <typename ET, typename FT> struct hline<ET, FT, 4, true, 1>
    {
        static void ResizeCn(ET* src, int, int* ofst, FT* m, FT* dst, int dst_min, int dst_max, int dst_width)
        {
            int i = 0;
            FT src0(src[0]);
            for (; i < dst_min; i++, m += 4) // Points that fall left from src image so became equal to leftmost src point
            {
                *(dst++) = src0;
            }
            for (; i < dst_max; i++, m += 4)
            {
                ET* px = src + ofst[i];
                *(dst++) = m[0] * src[0] + m[1] * src[1] + m[2] * src[2] + m[3] * src[3];
            }
            src0 = (src + ofst[dst_width - 1])[0];
            for (; i < dst_width; i++) // Points that fall right from src image so became equal to rightmost src point
            {
                *(dst++) = src0;
            }
        }
    };
    template <typename ET, typename FT> struct hline<ET, FT, 4, true, 2>
    {
        static void ResizeCn(ET* src, int, int* ofst, FT* m, FT* dst, int dst_min, int dst_max, int dst_width)
        {
            int i = 0;
            FT src0(src[0]), src1(src[1]);
            for (; i < dst_min; i++, m += 4) // Points that fall left from src image so became equal to leftmost src point
            {
                *(dst++) = src0;
                *(dst++) = src1;
            }
            for (; i < dst_max; i++, m += 4)
            {
                ET* px = src + 2 * ofst[i];
                *(dst++) = m[0] * src[0] + m[1] * src[2] + m[2] * src[4] + m[3] * src[6];
                *(dst++) = m[0] * src[1] + m[1] * src[3] + m[2] * src[5] + m[3] * src[7];
            }
            src0 = (src + 2 * ofst[dst_width - 1])[0];
            src1 = (src + 2 * ofst[dst_width - 1])[1];
            for (; i < dst_width; i++) // Points that fall right from src image so became equal to rightmost src point
            {
                *(dst++) = src0;
                *(dst++) = src1;
            }
        }
    };
    template <typename ET, typename FT> struct hline<ET, FT, 4, true, 3>
    {
        static void ResizeCn(ET* src, int, int* ofst, FT* m, FT* dst, int dst_min, int dst_max, int dst_width)
        {
            int i = 0;
            FT src0(src[0]), src1(src[1]), src2(src[2]);
            for (; i < dst_min; i++, m += 4) // Points that fall left from src image so became equal to leftmost src point
            {
                *(dst++) = src0;
                *(dst++) = src1;
                *(dst++) = src2;
            }
            for (; i < dst_max; i++, m += 4)
            {
                ET* px = src + 3 * ofst[i];
                *(dst++) = m[0] * src[0] + m[1] * src[3] + m[2] * src[6] + m[3] * src[9];
                *(dst++) = m[0] * src[1] + m[1] * src[4] + m[2] * src[7] + m[3] * src[10];
                *(dst++) = m[0] * src[2] + m[1] * src[5] + m[2] * src[8] + m[3] * src[11];
            }
            src0 = (src + 3 * ofst[dst_width - 1])[0];
            src1 = (src + 3 * ofst[dst_width - 1])[1];
            src2 = (src + 3 * ofst[dst_width - 1])[2];
            for (; i < dst_width; i++) // Points that fall right from src image so became equal to rightmost src point
            {
                *(dst++) = src0;
                *(dst++) = src1;
                *(dst++) = src2;
            }
        }
    };
    template <typename ET, typename FT> struct hline<ET, FT, 4, true, 4>
    {
        static void ResizeCn(ET* src, int, int* ofst, FT* m, FT* dst, int dst_min, int dst_max, int dst_width)
        {
            int i = 0;
            FT src0(src[0]), src1(src[1]), src2(src[2]), src3(src[3]);
            for (; i < dst_min; i++, m += 4) // Points that fall left from src image so became equal to leftmost src point
            {
                *(dst++) = src0;
                *(dst++) = src1;
                *(dst++) = src2;
                *(dst++) = src3;
            }
            for (; i < dst_max; i++, m += 4)
            {
                ET* px = src + 4 * ofst[i];
                *(dst++) = m[0] * src[0] + m[1] * src[4] + m[2] * src[8] + m[3] * src[12];
                *(dst++) = m[0] * src[1] + m[1] * src[5] + m[2] * src[9] + m[3] * src[13];
                *(dst++) = m[0] * src[2] + m[1] * src[6] + m[2] * src[10] + m[3] * src[14];
                *(dst++) = m[0] * src[3] + m[1] * src[7] + m[2] * src[11] + m[3] * src[15];
            }
            src0 = (src + 4 * ofst[dst_width - 1])[0];
            src1 = (src + 4 * ofst[dst_width - 1])[1];
            src2 = (src + 4 * ofst[dst_width - 1])[2];
            src3 = (src + 4 * ofst[dst_width - 1])[3];
            for (; i < dst_width; i++) // Points that fall right from src image so became equal to rightmost src point
            {
                *(dst++) = src0;
                *(dst++) = src1;
                *(dst++) = src2;
                *(dst++) = src3;
            }
        }
    };*/
    /*template <typename ET, typename FT, int n, bool mulall, int cncnt>
    static void hlineResizeCn(ET* src, int cn, int* ofst, FT* m, FT* dst, int dst_min, int dst_max, int dst_width)
    {
        hline<ET, FT, n, mulall, cncnt>::ResizeCn(src, cn, ofst, m, dst, dst_min, dst_max, dst_width);
    };

    template <><uint8_t, ufixedpoint16, 2, true, 1>*/
    void hlineResizeCn(uint8_t* src, int nouse, int* ofst, ufixedpoint16* m, ufixedpoint16* dst, int dst_min, int dst_max, int dst_width)
    {
        int i = 0;
       ufixedpoint16 src_0(src[0]);
       /*uchar* srcof = src + ofst[dst_width - 1];
                //uchar srcdata = srcof[0];
                ufixedpoint16 src_1(srcof[0]);*/




       // int counter = dst_min + dst_max;
                //dst[i] = src_1;
        for (; i < dst_min; i++, m += 2)
        {
            dst[i] = src_0;
        }

         for (; i < dst_max; i += 1, m += 2)
        {
            uint8_t* px = src + ofst[i];
            dst[i] = m[0] * px[0] + m[1] * px[1];

        }

         uchar* srcof = src + ofst[dst_width - 1];
                 src_0 = srcof[0];


         for (; i < dst_width; i++)
        {
        	dst[i] = src_0;


        }
    }



    template <typename ET, typename FT>
    void vlineSet(FT* src, ET* dst, int dst_width)
    {
        for (int i = 0; i < dst_width; i++)
            dst[i] = src[i];
    }
    /*template <>
    void vlineSet<uint8_t, ufixedpoint16>(ufixedpoint16* src, uint8_t* dst, int dst_width)
    {
        int i = 0;

        for (; i < dst_width; i++)
        	dst[i] = src[i];
    }*/

void vlineResize(ufixedpoint16* src, size_t src_step, ufixedpoint16* m, uint8_t* dst, int dst_width)
    {
        // cout << "vlineResize" << endl;
        for (int i = 0; i < dst_width; i++)
        {
            typename ufixedpoint16::WT res = src[i] * m[0];
            for (int k = 1; k < 2; k++)
                res = res + src[i + k * src_step] * m[k];
            dst[i] = res;
        }
    }


   /* template <typename ET, typename FT, int n>
    void vlineResize(FT* src, size_t src_step, FT* m, ET* dst, int dst_width)
    {
        for (int i = 0; i < dst_width; i++)
        {
            typename FT::WT res = src[i] * m[0];
            for (int k = 1; k < n; k++)
                res = res + src[i + k * src_step] * m[k];
            dst[i] = res;
        }
    }
    template <>
    void vlineResize<uint8_t, ufixedpoint16, 2>(ufixedpoint16* src, size_t src_step, ufixedpoint16* m, uint8_t* dst, int dst_width)
    {
        int i = 0;
        ufixedpoint16* src1 = src + src_step;

        for (; i < dst_width; i++)
        {
            *(dst++) = (uint8_t)(*(src++) * m[0] + *(src1++) * m[1]);
        }
    }*/

    //template <typename ET>
    class interpolationLinear
    {
    public:
        static const int len = 2;
        static const bool needsign = false;
        interpolationLinear(double inv_scale, int srcsize, int dstsize) : scale(softdouble::one() / softdouble(inv_scale)), maxsize(srcsize), minofst(0), maxofst(dstsize) {}
        void getCoeffs(int val, int* offset, ufixedpoint16* coeffs)
        {
            //typedef typename fixedtype<ET, needsign>::type fixedpoint;
            softdouble fval = scale * (softdouble(val) + softdouble(0.5)) - softdouble(0.5);
            int ival = cvFloor(fval);
            if (ival >= 0 && maxsize > 1)
            {
                if (ival < maxsize - 1)
                {
                    *offset = ival;
                    coeffs[1] = fval - softdouble(ival);
                    coeffs[0] = ufixedpoint16::one() - coeffs[1];
                }
                else
                {
                    *offset = maxsize - 1;
                    maxofst = min(maxofst, val);
                }
            }
            else
            {
                minofst = max(minofst, val + 1);
            }
        }
        void getMinMax(int& min, int& max)
        {
            min = minofst;
            max = maxofst;
        }
    protected:
        softdouble scale;
        int maxsize;
        int minofst, maxofst;
    };

   /* template <typename ET, typename FT, int interp_y_len>
    class resize_bitExactInvoker :
        public ParallelLoopBody_
    {
    public:
        typedef FT fixedpoint;
        typedef void(*hResizeFunc)(ET* src, int cn, int* ofst, fixedpoint* m, fixedpoint* dst, int dst_min, int dst_max, int dst_width);
        resize_bitExactInvoker(const uchar* _src, size_t _src_step, int _src_width, int _src_height,
            uchar* _dst, size_t _dst_step, int _dst_width, int _dst_height,
            int _cn, int* _xoffsets, int* _yoffsets, fixedpoint* _xcoeffs, fixedpoint* _ycoeffs,
            int _min_x, int _max_x, int _min_y, int _max_y, hResizeFunc _hResize) : ParallelLoopBody_(),
            src(_src), src_step(_src_step), src_width(_src_width), src_height(_src_height),
            dst(_dst), dst_step(_dst_step), dst_width(_dst_width), dst_height(_dst_height),
            cn(_cn), xoffsets(_xoffsets), yoffsets(_yoffsets), xcoeffs(_xcoeffs), ycoeffs(_ycoeffs),
            min_x(_min_x), max_x(_max_x), min_y(_min_y), max_y(_max_y), hResize(_hResize) {}

        virtual void operator() (const Range& range) const CV_OVERRIDE
        {
            AutoBuffer__<fixedpoint> linebuf(interp_y_len * dst_width * cn);
            int last_eval = -interp_y_len;
            int evalbuf_start = 0;
            int rmin_y = max(min_y, range.start);
            int rmax_y = min(max_y, range.end);
            if (range.start < min_y)
            {
                last_eval = 1 - interp_y_len;
                evalbuf_start = 1;
                hResize((ET*)src, cn, xoffsets, xcoeffs, linebuf.data(), min_x, max_x, dst_width);
            }
            int dy = range.start;
            for (; dy < rmin_y; dy++)
                vlineSet<ET, FT>(linebuf.data(), (ET*)(dst + dst_step * dy), dst_width * cn);
            for (; dy < rmax_y; dy++)
            {
                int& iy = yoffsets[dy];

                int i;
                for (i = max(iy, last_eval + interp_y_len); i < min(iy + interp_y_len, src_height); i++, evalbuf_start = (evalbuf_start + 1) % interp_y_len)
                    hResize((ET*)(src + i * src_step), cn, xoffsets, xcoeffs, linebuf.data() + evalbuf_start * (dst_width * cn), min_x, max_x, dst_width);
                evalbuf_start = (evalbuf_start + max(iy, src_height - interp_y_len) - max(last_eval, src_height - interp_y_len)) % interp_y_len;
                last_eval = iy;

                fixedpoint curcoeffs[interp_y_len];
                for (i = 0; i < evalbuf_start; i++)
                    curcoeffs[i] = ycoeffs[dy * interp_y_len - evalbuf_start + interp_y_len + i];
                for (; i < interp_y_len; i++)
                    curcoeffs[i] = ycoeffs[dy * interp_y_len - evalbuf_start + i];

                vlineResize<ET, FT, interp_y_len>(linebuf.data(), dst_width * cn, curcoeffs, (ET*)(dst + dst_step * dy), dst_width * cn);
            }
            fixedpoint* endline = linebuf.data();
            if (last_eval + interp_y_len > src_height)
                endline += dst_width * cn * ((evalbuf_start + src_height - 1 - last_eval) % interp_y_len);
            else
                hResize((ET*)(src + (src_height - 1) * src_step), cn, xoffsets, xcoeffs, endline, min_x, max_x, dst_width);
            for (; dy < range.end; dy++)
                vlineSet<ET, FT>(endline, (ET*)(dst + dst_step * dy), dst_width * cn);

        }

    private:
        const uchar* src;
        size_t src_step;
        int src_width, src_height;
        uchar* dst;
        size_t dst_step;
        int dst_width, dst_height, cn;
        int* xoffsets, * yoffsets;
        fixedpoint* xcoeffs, * ycoeffs;
        int min_x, max_x, min_y, max_y;
        hResizeFunc hResize;

        resize_bitExactInvoker(const resize_bitExactInvoker&);
        resize_bitExactInvoker& operator=(const resize_bitExactInvoker&);
    };*/

    //template <typename ET, typename interpolation>
    void resize_bitExact(const uchar* src, size_t src_step, int src_width, int src_height,
        uchar* dst, size_t dst_step, int dst_width, int dst_height,
        int cn, double inv_scale_x, double inv_scale_y)
    {
        //typedef typename fixedtype<ET, false>::type fixedpoint;
        /*void(*hResize)(ET * src, int cn, int* ofst, fixedpoint * m, fixedpoint * dst, int dst_min, int dst_max, int dst_width);
        //cout << "src_width : " << src_width<<endl;fixedShift
       // cout << "fixed : " << fixedpoint.fixedShift << endl;
        switch (cn)
        {
        case  1: hResize = src_width > 2 ? hlineResizeCn<ET, fixedpoint, 2, true, 1> : hlineResizeCn<ET, fixedpoint, 2, false, 1>; break;
        case  2: hResize = src_width > 2 ? hlineResizeCn<ET, fixedpoint, 2, true, 2> : hlineResizeCn<ET, fixedpoint, 2, false, 2>; break;
        case  3: hResize = src_width > 2 ? hlineResizeCn<ET, fixedpoint, 2, true, 3> : hlineResizeCn<ET, fixedpoint, 2, false, 3>; break;
        case  4: hResize = src_width > 2 ? hlineResizeCn<ET, fixedpoint, 2, true, 4> : hlineResizeCn<ET, fixedpoint, 2, false, 4>; break;
        default: hResize = src_width > 2 ? hlineResize<ET, fixedpoint, 2, true> : hlineResize<ET, fixedpoint, 2, false>; break;
        }*/
        //cout << "cn: " << cn << ", src_width: " << src_width << endl;

        interpolationLinear interp_x(inv_scale_x, src_width, dst_width);
        interpolationLinear interp_y(inv_scale_y, src_height, dst_height);



        static unsigned char buf[12776];

        int* xoffsets = (int*)buf;
        int* yoffsets = xoffsets + dst_width;
        ufixedpoint16* xcoeffs = (ufixedpoint16*)(yoffsets + dst_height);
        ufixedpoint16* ycoeffs = xcoeffs + dst_width * interp_x.len;

        int min_x, max_x, min_y, max_y;
        for (int dx = 0; dx < dst_width; dx++)
            interp_x.getCoeffs(dx, xoffsets + dx, xcoeffs + dx * interp_x.len);
        interp_x.getMinMax(min_x, max_x);
        for (int dy = 0; dy < dst_height; dy++)
            interp_y.getCoeffs(dy, yoffsets + dy, ycoeffs + dy * interp_y.len);
        interp_y.getMinMax(min_y, max_y);





       // resize_bitExactInvoker<ET, fixedpoint, interpolation::len> invoker(src, src_step, src_width, src_height, dst, dst_step, dst_width, dst_height, cn,
        //    xoffsets, yoffsets, xcoeffs, ycoeffs, min_x, max_x, min_y, max_y, hResize);
        Range range(0, dst_height);

        static const int interp_y_len = 2;
                //   cout << "Using" << endl;
                   //AutoBuffer__<fixedpoint> linebuf(interp_y_len * dst_width * cn);
                   //cout << " size: " << interp_y_len * dst_width * cn << endl;
         ufixedpoint16 LB[2068];
        ufixedpoint16* linebuf;
                linebuf = LB;

          /*      unsigned short LBN[2068];
                unsigned short* lbn = LBN;

                for (int idx = 0; idx < 2068; idx++){
                	#pragma HLS PIPELINE
                	linebuf[idx] = 0;
                }*/

                //linebuf[0] = 0;
                //lbn[1] = 0;

                //ufixedpoint32 temp = linebuf[0] ;
        // unsigned int temp3 =(temp.raw() + ((1 << 16) >> 1)) >> 16;
    // unsigned int temp2 = min ( temp3, (unsigned int)UCHAR_MAX);
           //     uchar* srch1 = (uchar*)src;
          //   dst[0]  = srch1[0];

                   int last_eval = -interp_y_len;
                   int evalbuf_start = 0;
                   int rmin_y = max(min_y, range.start);   //cout << "rmin_y: " << rmin_y << endl; 0
                   int rmax_y = min(max_y, range.end);     //cout << "max_y: " << max_y << endl; //change

                  // uchar* srch1 = (uchar*)src;
              /*     uchar tes = 0;
                  ufixedpoint16 src_0t(tes);
                   for (int i = 0; i < min_x; i++)
                     {

                             linebuf[i] = src_0t;

                     }*/

                   if (range.start < min_y)
                   {
                       last_eval = 1 - interp_y_len;
                       evalbuf_start = 1;
                       hlineResizeCn((uchar*)src, cn, xoffsets, xcoeffs, linebuf, min_x, max_x, dst_width);
                       /*{
                                           uchar* srch = (uchar*)src;
                                           int i = 0;
                                           ufixedpoint16 src_0(srch[0]);
                                           ufixedpoint16* m = xcoeffs;

                                           //cout << "dst_min: " << dst_min << endl;
											//#pragma HLS UNROLL factor=4
                                           for (; i < min_x; i++, m += 2)
                                           {

                                               linebuf[i] = src_0;

                                           }

                                           for (; i < max_x; i += 1, m += 2)
                                           {
                                               uint8_t* px = srch + xoffsets[i];
                                               linebuf[i] = m[0] * px[0] + m[1] * px[1];
                                           }
                                            uchar* srcof = srch + xoffsets[dst_width - 1];
                                           src_0 = srcof[0];

                                          for (; i < dst_width; i++)
                                           {
                                               linebuf[i] = src_0;
                                           }
                         }*/

                   }


                   int dy = range.start;
                  // unsigned char* dstnew = (dst + dst_step * dy);
                  for (; dy < rmin_y; dy++)
                	  cout<<"Error: In resize1.h, use vlineSet"<<endl;
                       //vlineSet<unsigned char, fixedpoint>(linebuf, dstnew, dst_width * cn);
                   //cout << "dy1: " << dy << endl;
                  for (; dy < rmax_y; dy++)
                   {
#pragma HLS PIPELINE
                       int& iy = yoffsets[dy];

                       int i;
                       for (i = max(iy, last_eval + interp_y_len); i < min(iy + interp_y_len, src_height); i++, evalbuf_start = (evalbuf_start + 1) % interp_y_len)
                    	   hlineResizeCn((uchar*)(src + i * src_step), cn, xoffsets, xcoeffs, linebuf + evalbuf_start * (dst_width * cn), min_x, max_x, dst_width);
                    	   /*{

                                           uchar* srch = (uchar*)(src + i * src_step);
                                           int counter = 0;
                                           ufixedpoint16 src_0(srch[0]);
                                           ufixedpoint16* m = xcoeffs;
                                           ufixedpoint16* linebufh = linebuf + evalbuf_start * (dst_width * cn);

                                           //cout << "dst_min: " << dst_min << endl;

                                           for (; counter < min_x; counter++, m += 2)
                                           {
                                               linebufh[counter] = src_0;

                                           }

                                           for (; counter < max_x; counter += 1, m += 2)
                                           {
                                               uint8_t* px = srch + xoffsets[counter];
                                               linebufh[counter] = m[0] * px[0] + m[1] * px[1];
                                           }
                                           uchar* srcof = srch + xoffsets[dst_width - 1];
                                           src_0 = srcof[0];


                                           for (; counter < dst_width; counter++)
                                           {
                                               linebufh[counter] = src_0;
                                           }
                       }*/






                       evalbuf_start = (evalbuf_start + max(iy, src_height - interp_y_len) - max(last_eval, src_height - interp_y_len)) % interp_y_len;
                       last_eval = iy;

                       ufixedpoint16 curcoeffs[interp_y_len];
                       for (i = 0; i < evalbuf_start; i++)
                           curcoeffs[i] = ycoeffs[dy * interp_y_len - evalbuf_start + interp_y_len + i];
                       for (; i < interp_y_len; i++)
                           curcoeffs[i] = ycoeffs[dy * interp_y_len - evalbuf_start + i];

                       //vlineResize<ET, ufixedpoint16, interp_y_len>(linebuf, dst_width * cn, curcoeffs, (ET*)(dst + dst_step * dy), dst_width * cn);
                     // vlineResize(linebuf, dst_width * cn, curcoeffs, (dst + dst_step * dy), dst_width * cn);

                       unsigned char* dstV = (dst + dst_step * dy);

                       size_t src_step = (size_t)(dst_width * cn);
                       unsigned char dstVbuf[1034];
                                      for (int i = 0; i < dst_width; i++)
                                      {

                                    	  ufixedpoint32 res = linebuf[i] * curcoeffs[0];
                                          for (int k = 1; k < 2; k++)
                                          {

                                              res = res + linebuf[i + k * src_step] * curcoeffs[k];
                                          }

                                          //uchar dstVbuf = saturate_cast<uchar>((res.raw() + ((1 << 16) >> 1)) >> 16);
                                          //dstV[i] = dstVbuf;//
                                          unsigned int ibuf = min(((res.raw() + ((1 << 16) >> 1)) >> 16), (unsigned)UCHAR_MAX);
                                          dstV[i] = res;

                                      }
                                     // for (int i = 0; i < dst_width; i++)
                                      //      dstV[i] = dstVbuf[i];

                   }
                   ufixedpoint16* endline = linebuf;
                   //cout << "dy2: " << dy << endl;
                   if (last_eval + interp_y_len > src_height)
                       endline += dst_width * cn * ((evalbuf_start + src_height - 1 - last_eval) % interp_y_len);
                   else
                	   hlineResizeCn((uchar*)(src + (src_height - 1) * src_step), cn, xoffsets, xcoeffs, endline, min_x, max_x, dst_width);
                  // for (; dy < range.end; dy++)
                   //    vlineSet<ET, ufixedpoint16>(endline, (ET*)(dst + dst_step * dy), dst_width * cn);




        //parallel_for__(range, invoker, dst_width * dst_height / (double)(1 << 16));*/
    }



//}

//namespace cv
//{

    /************** interpolation formulas and tables ***************/

  

    template<typename ST, typename DT> struct Cast
    {
        typedef ST type1;
        typedef DT rtype;

        DT operator()(ST val) const { return saturate_cast<DT>(val); }
    };

    /*template<typename ST, typename DT, int bits> struct FixedPtCast
    {
        typedef ST type1;
        typedef DT rtype;
        enum { SHIFT = bits, DELTA = 1 << (bits - 1) };

        DT operator()(ST val) const { return saturate_cast<DT>((val + DELTA) >> SHIFT); }
    };*/

    /****************************************************************************************\
    *                                         Resize                                         *
    \****************************************************************************************/

 

    /*typedef void (*ResizeFunc)(const Mat& src, Mat& dst,
        const int* xofs, const void* alpha,
        const int* yofs, const void* beta,
        int xmin, int xmax, int ksize);*/

    /*typedef void (*ResizeAreaFastFunc)(const Mat& src, Mat& dst,
        const int* ofs, const int* xofs,
        int scale_x, int scale_y);

    typedef void (*ResizeAreaFunc)(const Mat& src, Mat& dst,
        const DecimateAlpha* xtab, int xtab_size,
        const DecimateAlpha* ytab, int ytab_size,
        const int* yofs);*/


   

    //==================================================================================================
        enum {
            
            CV_StsBadArg = -5,  /**< function arg/param is bad       */
        };

    //namespace hal {

        void resize_(int src_type,
            const uchar* src_data, size_t src_step, int src_width, int src_height,
            uchar* dst_data, size_t dst_step, int dst_width, int dst_height,
            double inv_scale_x, double inv_scale_y, int interpolation)
        {
            //CV_INSTRUMENT_REGION();

           // CV_Assert((dst_width > 0 && dst_height > 0) || (inv_scale_x > 0 && inv_scale_y > 0));
            /*if (inv_scale_x < DBL_EPSILON || inv_scale_y < DBL_EPSILON)
            {
                inv_scale_x = static_cast<double>(dst_width) / src_width;
                inv_scale_y = static_cast<double>(dst_height) / src_height;
            }
            cout << " Test: " << (inv_scale_x < DBL_EPSILON || inv_scale_y < DBL_EPSILON) << endl;*/

            //CALL_HAL(resize_, cv_hal_resize, src_type, src_data, src_step, src_width, src_height, dst_data, dst_step, dst_width, dst_height, inv_scale_x, inv_scale_y, interpolation);

           // int  depth = CV_MAT_DEPTH(src_type), cn = CV_MAT_CN(src_type);
            int  depth = ((src_type) & ((1 << 3) - 1)), cn = ((((src_type) & (511 << 3)) >> 3) + 1) ; //cout << "cn : " << cn << endl; d:0
          //  Size dsize = Size(saturate_cast<int>(src_width * inv_scale_x),
           //     saturate_cast<int>(src_height * inv_scale_y));
           
            //dst_data[0] = 0;



           // double scale_x = 1. / inv_scale_x, scale_y = 1. / inv_scale_y;

          //  int iscale_x = saturate_cast<int>(scale_x);
           // int iscale_y = saturate_cast<int>(scale_y);

          //  bool is_area_fast = std::abs(scale_x - iscale_x) < DBL_EPSILON &&
          //      std::abs(scale_y - iscale_y) < DBL_EPSILON;

           // Mat src(Size(src_width, src_height), src_type, const_cast<uchar*>(src_data), src_step);
            //Mat dst(dsize, src_type, dst_data, dst_step);

            //if (interpolation == INTER_LINEAR_EXACT)
            //{
               // cout << "INTER_LINEAR_EXACT" << endl;
                // in case of inv_scale_x && inv_scale_y is equal to 0.5
                // INTER_AREA (fast) is equal to bit exact INTER_LINEAR
                //if (is_area_fast && iscale_x == 2 && iscale_y == 2 && cn != 2)//Area resize implementation for 2-channel images isn't bit-exact
             //   {
                    //cout << "INTER_AREA" << endl;
                //    interpolation = INTER_AREA;
              //  }
               // else
                {
                    /*be_resize_func func = linear_exact_tab[depth];
                    //CV_Assert(func != 0);
                    func(src_data, src_step, src_width, src_height,
                        dst_data, dst_step, dst_width, dst_height,
                        cn, inv_scale_x, inv_scale_y);*/

                   resize_bitExact(src_data, src_step, src_width, src_height,
                                            dst_data, dst_step, dst_width, dst_height,
                                            cn, inv_scale_x, inv_scale_y);
                   //uchar test = dst_data[0];
                   
                    return;
                }
           // }
            cout << "here!!" << endl;
            /*
            if (interpolation == INTER_NEAREST)
            {
                
                resizeNN(src, dst, inv_scale_x, inv_scale_y);
                return;
            }

            if (interpolation == INTER_NEAREST_EXACT)
            {
                resizeNN_bitexact(src, dst, inv_scale_x, inv_scale_y);
                return;
            }

           
            
            int k, sx, sy, dx, dy;


            {
                // in case of scale_x && scale_y is equal to 2
                // INTER_AREA (fast) also is equal to INTER_LINEAR
                if (interpolation == INTER_LINEAR && is_area_fast && iscale_x == 2 && iscale_y == 2)
                    interpolation = INTER_AREA;

                // true "area" interpolation is only implemented for the case (scale_x >= 1 && scale_y >= 1).
                // In other cases it is emulated using some variant of bilinear interpolation
                if (interpolation == INTER_AREA && scale_x >= 1 && scale_y >= 1)
                {
                    cout << "INTER_AREA" << endl;
                    if (is_area_fast)
                    {
                        int area = iscale_x * iscale_y;
                        size_t srcstep = src_step / src.elemSize1();
                        AutoBuffer<int> _ofs(area + dsize.width * cn);
                        int* ofs = _ofs.data();
                        int* xofs = ofs + area;
                        ResizeAreaFastFunc func = areafast_tab[depth];
                        CV_Assert(func != 0);

                        for (sy = 0, k = 0; sy < iscale_y; sy++)
                            for (sx = 0; sx < iscale_x; sx++)
                                ofs[k++] = (int)(sy * srcstep + sx * cn);

                        for (dx = 0; dx < dsize.width; dx++)
                        {
                            int j = dx * cn;
                            sx = iscale_x * j;
                            for (k = 0; k < cn; k++)
                                xofs[j + k] = sx + k;
                        }

                        func(src, dst, ofs, xofs, iscale_x, iscale_y);
                        return;
                    }

                    ResizeAreaFunc func = area_tab[depth];
                    CV_Assert(func != 0 && cn <= 4);

                    AutoBuffer<DecimateAlpha> _xytab((src_width + src_height) * 2);
                    DecimateAlpha* xtab = _xytab.data(), * ytab = xtab + src_width * 2;

                    int xtab_size = computeResizeAreaTab(src_width, dsize.width, cn, scale_x, xtab);
                    int ytab_size = computeResizeAreaTab(src_height, dsize.height, 1, scale_y, ytab);

                    AutoBuffer<int> _tabofs(dsize.height + 1);
                    int* tabofs = _tabofs.data();
                    for (k = 0, dy = 0; k < ytab_size; k++)
                    {
                        if (k == 0 || ytab[k].di != ytab[k - 1].di)
                        {
                            assert(ytab[k].di == dy);
                            tabofs[dy++] = k;
                        }
                    }
                    tabofs[dy] = ytab_size;

                    func(src, dst, xtab, xtab_size, ytab, ytab_size, tabofs);
                    return;
                }
            }

            int xmin = 0, xmax = dsize.width, width = dsize.width * cn;
            bool area_mode = interpolation == INTER_AREA;
            bool fixpt = depth == CV_8U;
            float fx, fy;
            ResizeFunc func = 0;
            int ksize = 0, ksize2;
           // cout << "interpolation : " << interpolation << endl;
            if (interpolation == INTER_CUBIC)
                ksize = 4, func = cubic_tab[depth];
            else if (interpolation == INTER_LANCZOS4)
                ksize = 8, func = lanczos4_tab[depth];
            else if (interpolation == INTER_LINEAR || interpolation == INTER_AREA)*/
               // ksize = 2, func = linear_tab[depth];
           /* else
                CV_Error(CV_StsBadArg, "Unknown interpolation method");*/
           /* ksize2 = ksize / 2;

            CV_Assert(func != 0);

            AutoBuffer<uchar> _buffer((width + dsize.height) * (sizeof(int) + sizeof(float) * ksize));
            int* xofs = (int*)_buffer.data();
            int* yofs = xofs + width;
            float* alpha = (float*)(yofs + dsize.height);
            short* ialpha = (short*)alpha;
            float* beta = alpha + width * ksize;
            short* ibeta = ialpha + width * ksize;
            float cbuf[MAX_ESIZE] = { 0 };

            for (dx = 0; dx < dsize.width; dx++)
            {
                if (!area_mode)
                {
                    fx = (float)((dx + 0.5) * scale_x - 0.5);
                    sx = cvFloor(fx);
                    fx -= sx;
                }
                else
                {
                    sx = cvFloor(dx * scale_x);
                    fx = (float)((dx + 1) - (sx + 1) * inv_scale_x);
                    fx = fx <= 0 ? 0.f : fx - cvFloor(fx);
                }

                if (sx < ksize2 - 1)
                {
                    xmin = dx + 1;
                    if (sx < 0 && (interpolation != INTER_CUBIC && interpolation != INTER_LANCZOS4))
                        fx = 0, sx = 0;
                }

                if (sx + ksize2 >= src_width)
                {
                    xmax = min(xmax, dx);
                    if (sx >= src_width - 1 && (interpolation != INTER_CUBIC && interpolation != INTER_LANCZOS4))
                        fx = 0, sx = src_width - 1;
                }

                for (k = 0, sx *= cn; k < cn; k++)
                    xofs[dx * cn + k] = sx + k;
               // cout << "interpolation : " << interpolation << endl;
                if (interpolation == INTER_CUBIC)
                    interpolateCubic(fx, cbuf);
                else if (interpolation == INTER_LANCZOS4)
                    interpolateLanczos4(fx, cbuf);
                else
                {
                    cbuf[0] = 1.f - fx;
                    cbuf[1] = fx;
                }
                if (fixpt)
                {
                    for (k = 0; k < ksize; k++)
                        ialpha[dx * cn * ksize + k] = saturate_cast<short>(cbuf[k] * INTER_RESIZE_COEF_SCALE);
                    for (; k < cn * ksize; k++)
                        ialpha[dx * cn * ksize + k] = ialpha[dx * cn * ksize + k - ksize];
                }
                else
                {
                    for (k = 0; k < ksize; k++)
                        alpha[dx * cn * ksize + k] = cbuf[k];
                    for (; k < cn * ksize; k++)
                        alpha[dx * cn * ksize + k] = alpha[dx * cn * ksize + k - ksize];
                }
            }

            for (dy = 0; dy < dsize.height; dy++)
            {
                if (!area_mode)
                {
                    fy = (float)((dy + 0.5) * scale_y - 0.5);
                    sy = cvFloor(fy);
                    fy -= sy;
                }
                else
                {
                    sy = cvFloor(dy * scale_y);
                    fy = (float)((dy + 1) - (sy + 1) * inv_scale_y);
                    fy = fy <= 0 ? 0.f : fy - cvFloor(fy);
                }

                yofs[dy] = sy;
                if (interpolation == INTER_CUBIC)
                    interpolateCubic(fy, cbuf);
                else if (interpolation == INTER_LANCZOS4)
                    interpolateLanczos4(fy, cbuf);
                else
                {
                    cbuf[0] = 1.f - fy;
                    cbuf[1] = fy;
                }

                if (fixpt)
                {
                    for (k = 0; k < ksize; k++)
                        ibeta[dy * ksize + k] = saturate_cast<short>(cbuf[k] * INTER_RESIZE_COEF_SCALE);
                }
                else
                {
                    for (k = 0; k < ksize; k++)
                        beta[dy * ksize + k] = cbuf[k];
                }
            }

            func(src, dst, xofs, fixpt ? (void*)ialpha : (void*)alpha, yofs,
                fixpt ? (void*)ibeta : (void*)beta, xmin, xmax, ksize);

            */
        }

    //} // cv::hal::
//} // cv::

//==================================================================================================

void resize__(Mat& src, Mat& dst, Size dsize,
    double inv_scale_x, double inv_scale_y, int interpolation)
{

	dst.data[0] = 0;
	Size src_size(src.cols, src.rows);
    Size ssize = src_size;


    if (dsize.empty())
    {

        dsize = Size(saturate_cast<int>(ssize.width * inv_scale_x),
            saturate_cast<int>(ssize.height * inv_scale_y));

    }
    else
    {
        inv_scale_x = (double)dsize.width / ssize.width;
        inv_scale_y = (double)dsize.height / ssize.height;

    }
    //cout << "empty: " << (dsize.empty()) << endl;

    if (interpolation == INTER_LINEAR_EXACT && (src.depth() == CV_32F || src.depth() == CV_64F))
        interpolation = INTER_LINEAR; // If depth isn't supported fallback to generic resize
   // cout << "interpolation: " << interpolation << endl;

    

   
    resize_(src.type(), src.data, src.step[0], src.cols, src.rows, dst.data, dst.step[0], dst.cols, dst.rows, inv_scale_x, inv_scale_y, interpolation);

}


/*CV_IMPL void
cvResize(const CvArr* srcarr, CvArr* dstarr, int method)
{
    cv::Mat src = cv::cvarrToMat(srcarr), dst = cv::cvarrToMat(dstarr);
    CV_Assert(src.type() == dst.type());
    cv::resize(src, dst, dst.size(), (double)dst.cols / src.cols,
        (double)dst.rows / src.rows, method);
}*/

/* End of file. */
