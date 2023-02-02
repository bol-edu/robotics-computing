#pragma once
#include <opencv2/opencv.hpp>
#include "sepFilter2D__.h"
#include "AutoBuffer__.h"

using namespace std;

#define DBL_EPSILON      2.2204460492503131e-016 // smallest such that 1.0+DBL_EPSILON != 1.0

/*enum {
    softfloat_mulAdd_subC = 1,
    softfloat_mulAdd_subProd = 2
};*/


#define EXPTAB_SCALE 6
#define EXPTAB_MASK  ((1 << EXPTAB_SCALE) - 1)

CV_EXPORTS_W void GaussianBlur__(InputArray src, OutputArray dst, Size ksize,
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
/*
static inline struct uint128 softfloat_add128(uint64_t a64, uint64_t a0, uint64_t b64, uint64_t b0)
{
    struct uint128 z;
    z.v0 = a0 + b0;
    z.v64 = a64 + b64 + (z.v0 < a0);
    return z;
}

static struct exp16_sig64 softfloat_normSubnormalF64Sig(uint_fast64_t sig)
{
    int_fast8_t shiftDist;
    struct exp16_sig64 z;

    shiftDist = softfloat_countLeadingZeros64(sig) - 11;
    z.exp = 1 - shiftDist;
    z.sig = sig << shiftDist;
    return z;
}*/
/*
static inline struct uint128 softfloat_shortShiftRightJam128(uint64_t a64, uint64_t a0, uint_fast8_t dist)
{
    uint_fast8_t negDist = -dist;
    struct uint128 z;
    z.v64 = a64 >> dist;
    z.v0 =
        a64 << (negDist & 63) | a0 >> dist
        | ((uint64_t)(a0 << (negDist & 63)) != 0);
    return z;
}

static struct uint128
softfloat_shiftRightJam128(uint64_t a64, uint64_t a0, uint_fast32_t dist)
{
    uint_fast8_t u8NegDist;
    struct uint128 z;

    if (dist < 64) {
        //fixed unsigned unary minus: -x == ~x + 1 , fixed type cast
        u8NegDist = (uint_fast8_t)(~dist + 1);
        z.v64 = a64 >> dist;
        z.v0 =
            a64 << (u8NegDist & 63) | a0 >> dist
            | ((uint64_t)(a0 << (u8NegDist & 63)) != 0);
    }
    else {
        z.v64 = 0;
        z.v0 =
            (dist < 127)
            ? a64 >> (dist & 63)
            | (((a64 & (((uint_fast64_t)1 << (dist & 63)) - 1)) | a0)
                != 0)
            : ((a64 | a0) != 0);
    }
    return z;
}
/*
static inline struct uint128 softfloat_sub128(uint64_t a64, uint64_t a0, uint64_t b64, uint64_t b0)
{
    struct uint128 z;
    z.v0 = a0 - b0;
    z.v64 = a64 - b64;
    z.v64 -= (a0 < b0);
    return z;
}

static inline uint64_t softfloat_shortShiftRightJam64(uint64_t a, uint_fast8_t dist)
{
    return a >> dist | ((a & (((uint_fast64_t)1 << dist) - 1)) != 0);
}

static inline struct uint128 softfloat_shortShiftLeft128(uint64_t a64, uint64_t a0, uint_fast8_t dist)
{
    struct uint128 z;
    z.v64 = a64 << dist | a0 >> (-dist & 63);
    z.v0 = a0 << dist;
    return z;
}*/

/*static inline void raiseFlags(uint_fast8_t /* flags )
{
    //exceptionFlags |= flags;
}

enum {
    /*flag_inexact = 1,
    flag_underflow = 2,
    flag_overflow = 4,
    flag_infinite = 8,
    flag_invalid = 16
};*/
/*
static float64_t
softfloat_mulAddF64(
    uint_fast64_t uiA, uint_fast64_t uiB, uint_fast64_t uiC, uint_fast8_t op)
{
    bool signA;
    int_fast16_t expA;
    uint_fast64_t sigA;
    bool signB;
    int_fast16_t expB;
    uint_fast64_t sigB;
    bool signC;
    int_fast16_t expC;
    uint_fast64_t sigC;
    bool signZ;
    uint_fast64_t magBits, uiZ;
    struct exp16_sig64 normExpSig;
    int_fast16_t expZ;
    struct uint128 sig128Z;
    uint_fast64_t sigZ;
    int_fast16_t expDiff;
    struct uint128 sig128C;
    int_fast8_t shiftDist;

    /*------------------------------------------------------------------------
    /*------------------------------------------------------------------------
    signA = signF64UI(uiA);
    expA = expF64UI(uiA);
    sigA = fracF64UI(uiA);
    signB = signF64UI(uiB);
    expB = expF64UI(uiB);
    sigB = fracF64UI(uiB);
    signC = signF64UI(uiC) ^ (op == softfloat_mulAdd_subC);
    expC = expF64UI(uiC);
    sigC = fracF64UI(uiC);
    signZ = signA ^ signB ^ (op == softfloat_mulAdd_subProd);
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*
    if (expA == 0x7FF) {
        if (sigA || ((expB == 0x7FF) && sigB)) goto propagateNaN_ABC;
        magBits = expB | sigB;
        goto infProdArg;
    }
    if (expB == 0x7FF) {
        if (sigB) goto propagateNaN_ABC;
        magBits = expA | sigA;
        goto infProdArg;
    }
    if (expC == 0x7FF) {
        if (sigC) {
            uiZ = 0;
            goto propagateNaN_ZC;
        }
        uiZ = uiC;
        goto uiZ;
    }
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*
    if (!expA) {
        if (!sigA) goto zeroProd;
        normExpSig = softfloat_normSubnormalF64Sig(sigA);
        expA = normExpSig.exp;
        sigA = normExpSig.sig;
    }
    if (!expB) {
        if (!sigB) goto zeroProd;
        normExpSig = softfloat_normSubnormalF64Sig(sigB);
        expB = normExpSig.exp;
        sigB = normExpSig.sig;
    }
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*
    expZ = expA + expB - 0x3FE;
    sigA = (sigA | UINT64_C(0x0010000000000000)) << 10;
    sigB = (sigB | UINT64_C(0x0010000000000000)) << 10;
    sig128Z = softfloat_mul64To128(sigA, sigB);
    if (sig128Z.v64 < UINT64_C(0x2000000000000000)) {
        --expZ;
        sig128Z =
            softfloat_add128(
                sig128Z.v64, sig128Z.v0, sig128Z.v64, sig128Z.v0);
    }
    if (!expC) {
        if (!sigC) {
            --expZ;
            sigZ = sig128Z.v64 << 1 | (sig128Z.v0 != 0);
            goto roundPack;
        }
        normExpSig = softfloat_normSubnormalF64Sig(sigC);
        expC = normExpSig.exp;
        sigC = normExpSig.sig;
    }
    sigC = (sigC | UINT64_C(0x0010000000000000)) << 9;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*
    //fixed initialization
    sig128C.v0 = sig128C.v64 = 0;
    expDiff = expZ - expC;
    if (expDiff < 0) {
        expZ = expC;
        if ((signZ == signC) || (expDiff < -1)) {
            sig128Z.v64 = softfloat_shiftRightJam64(sig128Z.v64, -expDiff);
        }
        else {
            sig128Z =
                softfloat_shortShiftRightJam128(sig128Z.v64, sig128Z.v0, 1);
        }
    }
    else if (expDiff) {
        sig128C = softfloat_shiftRightJam128(sigC, 0, expDiff);
    }
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*
    if (signZ == signC) {
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*
        if (expDiff <= 0) {
            sigZ = (sigC + sig128Z.v64) | (sig128Z.v0 != 0);
        }
        else {
            sig128Z =
                softfloat_add128(
                    sig128Z.v64, sig128Z.v0, sig128C.v64, sig128C.v0);
            sigZ = sig128Z.v64 | (sig128Z.v0 != 0);
        }
        if (sigZ < UINT64_C(0x4000000000000000)) {
            --expZ;
            sigZ <<= 1;
        }
    }
    else {
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*
        if (expDiff < 0) {
            signZ = signC;
            sig128Z = softfloat_sub128(sigC, 0, sig128Z.v64, sig128Z.v0);
        }
        else if (!expDiff) {
            sig128Z.v64 = sig128Z.v64 - sigC;
            if (!(sig128Z.v64 | sig128Z.v0)) goto completeCancellation;
            if (sig128Z.v64 & UINT64_C(0x8000000000000000)) {
                signZ = !signZ;
                sig128Z = softfloat_sub128(0, 0, sig128Z.v64, sig128Z.v0);
            }
        }
        else {
            sig128Z =
                softfloat_sub128(
                    sig128Z.v64, sig128Z.v0, sig128C.v64, sig128C.v0);
        }
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*
        if (!sig128Z.v64) {
            expZ -= 64;
            sig128Z.v64 = sig128Z.v0;
            sig128Z.v0 = 0;
        }
        shiftDist = softfloat_countLeadingZeros64(sig128Z.v64) - 1;
        expZ -= shiftDist;
        if (shiftDist < 0) {
            sigZ = softfloat_shortShiftRightJam64(sig128Z.v64, -shiftDist);
        }
        else {
            sig128Z =
                softfloat_shortShiftLeft128(
                    sig128Z.v64, sig128Z.v0, shiftDist);
            sigZ = sig128Z.v64;
        }
        sigZ |= (sig128Z.v0 != 0);
    }
roundPack:
    return softfloat_roundPackToF64(signZ, expZ, sigZ);
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*
propagateNaN_ABC:
    uiZ = softfloat_propagateNaNF64UI(uiA, uiB);
    goto propagateNaN_ZC;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*
infProdArg:
    if (magBits) {
        uiZ = packToF64UI(signZ, 0x7FF, 0);
        if (expC != 0x7FF) goto uiZ;
        if (sigC) goto propagateNaN_ZC;
        if (signZ == signC) goto uiZ;
    }
    //raiseFlags(flag_invalid);
    uiZ = defaultNaNF64UI;
propagateNaN_ZC:
    uiZ = softfloat_propagateNaNF64UI(uiZ, uiC);
    goto uiZ;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*
zeroProd:
    uiZ = uiC;
    if (!(expC | sigC) && (signZ != signC)) {
    completeCancellation:
        uiZ =
            packToF64UI((globalRoundingMode == round_min), 0, 0);
    }
uiZ:
    return float64_t::fromRaw(uiZ);
}*/

/*
static float64_t f64_mulAdd(float64_t a, float64_t b, float64_t c)
{
    uint_fast64_t uiA;
    uint_fast64_t uiB;
    uint_fast64_t uiC;

    uiA = a.v;
    uiB = b.v;
    uiC = c.v;
    return softfloat_mulAddF64(uiA, uiB, uiC, 0);
}*/

//softdouble mulAdd(const softdouble& a, const softdouble& b, const softdouble& c) { return f64_mulAdd(a, b, c); }


static
softdouble getGaussianKernelBitExact(std::vector<softdouble>& result, int n, double sigma)
{
    //CV_Assert(n > 0);
    //TODO: incorrect SURF implementation requests kernel with n = 20 (PATCH_SZ): https://github.com/opencv/opencv/issues/15856
    //CV_Assert((n & 1) == 1);  // odd

    cout << "sigma: " << sigma << endl;
    /*if (sigma <= 0)
    {
        if (n == 1)
        {
            result = std::vector<softdouble>(1, softdouble::one());
            return softdouble::one();
        }
        else if (n == 3)
        {
            softdouble v3[] = {
                softdouble::fromRaw(0x3fd0000000000000),  // 0.25
                softdouble::fromRaw(0x3fe0000000000000),  // 0.5
                softdouble::fromRaw(0x3fd0000000000000)   // 0.25
            };
            result.assign(v3, v3 + 3);
            return softdouble::one();
        }
        else if (n == 5)
        {
            softdouble v5[] = {
                softdouble::fromRaw(0x3fb0000000000000),  // 0.0625
                softdouble::fromRaw(0x3fd0000000000000),  // 0.25
                softdouble::fromRaw(0x3fd8000000000000),  // 0.375
                softdouble::fromRaw(0x3fd0000000000000),  // 0.25
                softdouble::fromRaw(0x3fb0000000000000)   // 0.0625
            };
            result.assign(v5, v5 + 5);
            return softdouble::one();
        }
        else if (n == 7)
        {
            softdouble v7[] = {
                softdouble::fromRaw(0x3fa0000000000000),  // 0.03125
                softdouble::fromRaw(0x3fbc000000000000),  // 0.109375
                softdouble::fromRaw(0x3fcc000000000000),  // 0.21875
                softdouble::fromRaw(0x3fd2000000000000),  // 0.28125
                softdouble::fromRaw(0x3fcc000000000000),  // 0.21875
                softdouble::fromRaw(0x3fbc000000000000),  // 0.109375
                softdouble::fromRaw(0x3fa0000000000000)   // 0.03125
            };
            result.assign(v7, v7 + 7);
            return softdouble::one();
        }
        else if (n == 9)
        {
            softdouble v9[] = {
                softdouble::fromRaw(0x3f90000000000000),  // 4  / 256
                softdouble::fromRaw(0x3faa000000000000),  // 13 / 256
                softdouble::fromRaw(0x3fbe000000000000),  // 30 / 256
                softdouble::fromRaw(0x3fc9800000000000),  // 51 / 256
                softdouble::fromRaw(0x3fce000000000000),  // 60 / 256
                softdouble::fromRaw(0x3fc9800000000000),  // 51 / 256
                softdouble::fromRaw(0x3fbe000000000000),  // 30 / 256
                softdouble::fromRaw(0x3faa000000000000),  // 13 / 256
                softdouble::fromRaw(0x3f90000000000000)   // 4  / 256
            };
            result.assign(v9, v9 + 9);
            return softdouble::one();
        }
    }*/

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

Mat getGaussianKernel_(int n, double sigma, int ktype)
{
   // CV_CheckDepth(ktype, ktype == CV_32F || ktype == CV_64F, "");
    Mat kernel(n, 1, ktype);

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

static void getGaussianKernel_(int n, double sigma, int ktype, Mat& res) { res = getGaussianKernel_(n, sigma, ktype); }


template <typename T>
static void createGaussianKernels(T& kx, T& ky, int type, Size& ksize,
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

    getGaussianKernel_(ksize.width, sigma1, max(depth, CV_32F), kx);
    if (ksize.height == ksize.width && std::abs(sigma1 - sigma2) < DBL_EPSILON)
        ky = kx;
    else
        getGaussianKernel_(ksize.height, sigma2, max(depth, CV_32F), ky);
}

// in copyMakeBorder__.h
 /*enum BorderTypes_ {
    BORDER_CONSTANT_ = 0, //!< `iiiiii|abcdefgh|iiiiiii`  with some specified `i`
   BORDER_REPLICATE = 1, //!< `aaaaaa|abcdefgh|hhhhhhh`
    BORDER_REFLECT = 2, //!< `fedcba|abcdefgh|hgfedcb`
    BORDER_WRAP = 3, //!< `cdefgh|abcdefgh|abcdefg`
    BORDER_REFLECT_101 = 4, //!< `gfedcb|abcdefgh|gfedcba`
    BORDER_TRANSPARENT = 5, //!< `uvwxyz|abcdefgh|ijklmno`

    BORDER_REFLECT101 = BORDER_REFLECT_101, //!< same as BORDER_REFLECT_101
    BORDER_DEFAULT = BORDER_REFLECT_101, //!< same as BORDER_REFLECT_101
    BORDER_ISOLATED_ = 16 //!< do not look outside of ROI
};*/


void GaussianBlur__(InputArray _src, OutputArray _dst, Size ksize,
    double sigma1, double sigma2,
    int borderType)
{
    //CV_INSTRUMENT_REGION();

   // CV_Assert(!_src.empty());

    int type = _src.type();
    Size size = _src.size();
    _dst.create(size, type);

    if ((borderType & ~BORDER_ISOLATED_) != BORDER_CONSTANT_ &&
        ((borderType & BORDER_ISOLATED_) != 0 || !_src.getMat().isSubmatrix()))
    {
        if (size.height == 1)
            ksize.height = 1;
        if (size.width == 1)
            ksize.width = 1;
    }

    if (ksize.width == 1 && ksize.height == 1)
    {
        //cout << "copyTo" << endl;
        _src.copyTo(_dst);
        return;
    }

    /*bool useOpenCL = ocl::isOpenCLActivated() && _dst.isUMat() && _src.dims() <= 2 &&
        _src.rows() >= ksize.height && _src.cols() >= ksize.width &&
        ksize.width > 1 && ksize.height > 1;
    CV_UNUSED(useOpenCL);*/

    int sdepth = CV_MAT_DEPTH(type), cn = CV_MAT_CN(type);

    Mat kx, ky;
    createGaussianKernels(kx, ky, type, ksize, sigma1, sigma2);

    /*CV_OCL_RUN(useOpenCL && sdepth == CV_8U &&
        ((ksize.width == 3 && ksize.height == 3) ||
            (ksize.width == 5 && ksize.height == 5)),
        ocl_GaussianBlur_8UC1(_src, _dst, ksize, CV_MAT_DEPTH(type), kx, ky, borderType)
    );*/

    

    /*if (sdepth == CV_8U && ((borderType & BORDER_ISOLATED) || !_src.isSubmatrix()))
    {
        std::vector<ufixedpoint16> fkx, fky;
        createGaussianKernels(fkx, fky, type, ksize, sigma1, sigma2);

        static bool param_check_gaussian_blur_bitexact_kernels = utils::getConfigurationParameterBool("OPENCV_GAUSSIANBLUR_CHECK_BITEXACT_KERNELS", false);
        if (param_check_gaussian_blur_bitexact_kernels && !validateGaussianBlurKernel(fkx))
        {
            CV_LOG_INFO(NULL, "GaussianBlur: bit-exact fx kernel can't be applied: ksize=" << ksize << " sigma=" << Size2d(sigma1, sigma2));
        }
        else if (param_check_gaussian_blur_bitexact_kernels && !validateGaussianBlurKernel(fky))
        {
            CV_LOG_INFO(NULL, "GaussianBlur: bit-exact fy kernel can't be applied: ksize=" << ksize << " sigma=" << Size2d(sigma1, sigma2));
        }
        else
        {
            CV_OCL_RUN(useOpenCL,
                ocl_sepFilter2D_BitExact(_src, _dst, sdepth,
                    ksize,
                    (const uint16_t*)&fkx[0], (const uint16_t*)&fky[0],
                    Point(-1, -1), 0, borderType,
                    8//shift_bits)
            );

            Mat src = _src.getMat();
            Mat dst = _dst.getMat();

            if (src.data == dst.data)
                src = src.clone();
            CV_CPU_DISPATCH(GaussianBlurFixedPoint, (src, dst, (const uint16_t*)&fkx[0], (int)fkx.size(), (const uint16_t*)&fky[0], (int)fky.size(), borderType),
                CV_CPU_DISPATCH_MODES_ALL);
            return;
        }
    }*/
   /* if (sdepth == CV_16U && ((borderType & BORDER_ISOLATED) || !_src.isSubmatrix()))
    {
        CV_LOG_INFO(NULL, "GaussianBlur: running bit-exact version...");

        std::vector<ufixedpoint32> fkx, fky;
        createGaussianKernels(fkx, fky, type, ksize, sigma1, sigma2);

        static bool param_check_gaussian_blur_bitexact_kernels = utils::getConfigurationParameterBool("OPENCV_GAUSSIANBLUR_CHECK_BITEXACT_KERNELS", false);
        if (param_check_gaussian_blur_bitexact_kernels && !validateGaussianBlurKernel(fkx))
        {
            CV_LOG_INFO(NULL, "GaussianBlur: bit-exact fx kernel can't be applied: ksize=" << ksize << " sigma=" << Size2d(sigma1, sigma2));
        }
        else if (param_check_gaussian_blur_bitexact_kernels && !validateGaussianBlurKernel(fky))
        {
            CV_LOG_INFO(NULL, "GaussianBlur: bit-exact fy kernel can't be applied: ksize=" << ksize << " sigma=" << Size2d(sigma1, sigma2));
        }
        else
        {
            // TODO: implement ocl_sepFilter2D_BitExact -- how to deal with bdepth?
            // CV_OCL_RUN(useOpenCL,
            //         ocl_sepFilter2D_BitExact(_src, _dst, sdepth,
            //                 ksize,
            //                 (const uint32_t*)&fkx[0], (const uint32_t*)&fky[0],
            //                 Point(-1, -1), 0, borderType,
            //                 16//shift_bits)
            // );

            Mat src = _src.getMat();
            Mat dst = _dst.getMat();

            if (src.data == dst.data)
                src = src.clone();
            CV_CPU_DISPATCH(GaussianBlurFixedPoint, (src, dst, (const uint32_t*)&fkx[0], (int)fkx.size(), (const uint32_t*)&fky[0], (int)fky.size(), borderType),
                CV_CPU_DISPATCH_MODES_ALL);
            return;
        }
    }*/
/*
#ifdef HAVE_OPENCL
    if (useOpenCL)
    {
        sepFilter2D(_src, _dst, sdepth, kx, ky, Point(-1, -1), 0, borderType);
        return;
    }
#endif*/

    Mat src = _src.getMat();
    Mat dst = _dst.getMat();

    Point ofs;
    Size wsz(src.cols, src.rows);
    if (!(borderType & BORDER_ISOLATED_))
        src.locateROI(wsz, ofs);

    /*CALL_HAL(gaussianBlur, cv_hal_gaussianBlur, src.ptr(), src.step, dst.ptr(), dst.step, src.cols, src.rows, sdepth, cn,
        ofs.x, ofs.y, wsz.width - src.cols - ofs.x, wsz.height - src.rows - ofs.y, ksize.width, ksize.height,
        sigma1, sigma2, borderType & ~BORDER_ISOLATED);

    CV_OVX_RUN(true,
        openvx_gaussianBlur(src, dst, ksize, sigma1, sigma2, borderType))*/

#if defined ENABLE_IPP_GAUSSIAN_BLUR
        // IPP is not bit-exact to OpenCV implementation
        CV_IPP_RUN_FAST(ipp_GaussianBlur(src, dst, ksize, sigma1, sigma2, borderType));
#endif

    sepFilter2D__(src, dst, sdepth, kx, ky, Point(-1, -1), 0, borderType);
}
