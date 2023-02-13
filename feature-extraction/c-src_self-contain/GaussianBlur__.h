#pragma once
//#include <opencv2/opencv.hpp>
#include "sepFilter2D__.h"
#include "AutoBuffer__.h"
#include "define.h"
//#include "resize1.h"
#include "Point__.h"
#include "Size.h"
#include "Mat.h"
#include <algorithm>

using namespace std;

#define DBL_EPSILON      2.2204460492503131e-016 // smallest such that 1.0+DBL_EPSILON != 1.0

/*enum {
    softfloat_mulAdd_subC = 1,
    softfloat_mulAdd_subProd = 2
};*/


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
