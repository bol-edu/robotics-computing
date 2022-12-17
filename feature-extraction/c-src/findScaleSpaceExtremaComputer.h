#pragma once
//#include <opencv2/opencv.hpp>
#include "TLSDataAccumulator.h"
#include "Vec3f__.h"
//#include "BufferArea__.h"
using namespace std;
#ifdef __GNUC__
#  define CV_DECL_ALIGNED(x) __attribute__ ((aligned (x)))
#elif defined _MSC_VER
#  define CV_DECL_ALIGNED(x) __declspec(align(x))
#else
#  define CV_DECL_ALIGNED(x)
#endif

#define CV_SIMD_WIDTH 16
#define CV_PI   3.1415926535897932384626433832795

static const int SIFT_IMG_BORDER = 5;
static const int SIFT_ORI_HIST_BINS = 36;
static const int SIFT_MAX_INTERP_STEPS = 5;
static const float SIFT_ORI_SIG_FCTR = 1.5f;
static const float SIFT_ORI_RADIUS = 4.5f; // 3 * SIFT_ORI_SIG_FCTR;
static const float SIFT_ORI_PEAK_RATIO = 0.8f;

static const float atan2_p1_ = 0.9997878412794807f * (float)(180 / CV_PI);
static const float atan2_p3_ = -0.3258083974640975f * (float)(180 / CV_PI);
static const float atan2_p5_ = 0.1555786518463281f * (float)(180 / CV_PI);
static const float atan2_p7_ = -0.04432655554792128f * (float)(180 / CV_PI);


void exp32f(const float* src, float* dst, int n)
{
    //CV_INSTRUMENT_REGION();

    for (int i = 0; i < n; i++)
    {
        dst[i] = std::exp(src[i]);
    }
}


static inline float atan_f32(float y, float x)
{
    float ax = std::abs(x), ay = std::abs(y);
    float a, c, c2;
    if (ax >= ay)
    {
        c = ay / (ax + (float)DBL_EPSILON);
        c2 = c * c;
        a = (((atan2_p7_ * c2 + atan2_p5_) * c2 + atan2_p3_) * c2 + atan2_p1_) * c;
    }
    else
    {
        c = ax / (ay + (float)DBL_EPSILON);
        c2 = c * c;
        a = 90.f - (((atan2_p7_ * c2 + atan2_p5_) * c2 + atan2_p3_) * c2 + atan2_p1_) * c;
    }
    if (x < 0)
        a = 180.f - a;
    if (y < 0)
        a = 360.f - a;
    return a;
}

static void fastAtan32f_(const float* Y, const float* X, float* angle, int len, bool angleInDegrees)
{
    float scale = angleInDegrees ? 1.f : (float)(CV_PI / 180);
    int i = 0;
#if CV_SIMD
    const int VECSZ = v_float32::nlanes;
    v_atan_f32 v(scale);

    for (; i < len; i += VECSZ * 2)
    {
        if (i + VECSZ * 2 > len)
        {
            // if it's inplace operation, we cannot repeatedly process
            // the tail for the second time, so we have to use the
            // scalar code
            if (i == 0 || angle == X || angle == Y)
                break;
            i = len - VECSZ * 2;
        }

        v_float32 y0 = vx_load(Y + i);
        v_float32 x0 = vx_load(X + i);
        v_float32 y1 = vx_load(Y + i + VECSZ);
        v_float32 x1 = vx_load(X + i + VECSZ);

        v_float32 r0 = v.compute(y0, x0);
        v_float32 r1 = v.compute(y1, x1);

        v_store(angle + i, r0);
        v_store(angle + i + VECSZ, r1);
    }
    vx_cleanup();
#endif

    for (; i < len; i++)
        angle[i] = atan_f32(Y[i], X[i]) * scale;
}


void fastAtan32f(const float* Y, const float* X, float* angle, int len, bool angleInDegrees)
{
    //CV_INSTRUMENT_REGION();
    fastAtan32f_(Y, X, angle, len, angleInDegrees);
}

void fastAtan2(const float* Y, const float* X, float* angle, int len, bool angleInDegrees)
{
    //CV_INSTRUMENT_REGION();
    fastAtan32f(Y, X, angle, len, angleInDegrees);
}



void magnitude32f(const float* x, const float* y, float* mag, int len)
{
    //CV_INSTRUMENT_REGION();

    int i = 0;

#if CV_SIMD
    const int VECSZ = v_float32::nlanes;
    for (; i < len; i += VECSZ * 2)
    {
        if (i + VECSZ * 2 > len)
        {
            if (i == 0 || mag == x || mag == y)
                break;
            i = len - VECSZ * 2;
        }
        v_float32 x0 = vx_load(x + i), x1 = vx_load(x + i + VECSZ);
        v_float32 y0 = vx_load(y + i), y1 = vx_load(y + i + VECSZ);
        x0 = v_sqrt(v_muladd(x0, x0, y0 * y0));
        x1 = v_sqrt(v_muladd(x1, x1, y1 * y1));
        v_store(mag + i, x0);
        v_store(mag + i + VECSZ, x1);
    }
    vx_cleanup();
#endif

    for (; i < len; i++)
    {
        float x0 = x[i], y0 = y[i];
        mag[i] = std::sqrt(x0 * x0 + y0 * y0);
    }
}




static
float calcOrientationHist(
    const Mat& img, Point pt, int radius,
    float sigma, float* hist, int n
)
{
    //CV_TRACE_FUNCTION();

    int i, j, k, len = (radius * 2 + 1) * (radius * 2 + 1);

    float expf_scale = -1.f / (2.f * sigma * sigma);

    BufferArea__ area;
    float* X = 0, * Y = 0, * Mag, * Ori = 0, * W = 0, * temphist = 0;
    area.allocate(X, len, CV_SIMD_WIDTH);
    area.allocate(Y, len, CV_SIMD_WIDTH);
    area.allocate(Ori, len, CV_SIMD_WIDTH);
    area.allocate(W, len, CV_SIMD_WIDTH);
    area.allocate(temphist, n + 4, CV_SIMD_WIDTH);
    area.commit();
    temphist += 2;
    Mag = X;

    for (i = 0; i < n; i++)
        temphist[i] = 0.f;

    for (i = -radius, k = 0; i <= radius; i++)
    {
        int y = pt.y + i;
        if (y <= 0 || y >= img.rows - 1)
            continue;
        for (j = -radius; j <= radius; j++)
        {
            int x = pt.x + j;
            if (x <= 0 || x >= img.cols - 1)
                continue;

            float dx = (float)(img.at<sift_wt>(y, x + 1) - img.at<sift_wt>(y, x - 1));
            float dy = (float)(img.at<sift_wt>(y - 1, x) - img.at<sift_wt>(y + 1, x));

            X[k] = dx; Y[k] = dy; W[k] = (i * i + j * j) * expf_scale;
            k++;
        }
    }

    len = k;

    // compute gradient values, orientations and the weights over the pixel neighborhood
    exp32f(W, W, len);
    fastAtan2(Y, X, Ori, len, true);
    magnitude32f(X, Y, Mag, len);

    k = 0;
#if CV_SIMD
    const int vecsize = v_float32::nlanes;
    v_float32 nd360 = vx_setall_f32(n / 360.f);
    v_int32 __n = vx_setall_s32(n);
    int CV_DECL_ALIGNED(CV_SIMD_WIDTH) bin_buf[vecsize];
    float CV_DECL_ALIGNED(CV_SIMD_WIDTH) w_mul_mag_buf[vecsize];

    for (; k <= len - vecsize; k += vecsize)
    {
        v_float32 w = vx_load_aligned(W + k);
        v_float32 mag = vx_load_aligned(Mag + k);
        v_float32 ori = vx_load_aligned(Ori + k);
        v_int32 bin = v_round(nd360 * ori);

        bin = v_select(bin >= __n, bin - __n, bin);
        bin = v_select(bin < vx_setzero_s32(), bin + __n, bin);

        w = w * mag;
        v_store_aligned(bin_buf, bin);
        v_store_aligned(w_mul_mag_buf, w);
        for (int vi = 0; vi < vecsize; vi++)
        {
            temphist[bin_buf[vi]] += w_mul_mag_buf[vi];
        }
    }
#endif
    for (; k < len; k++)
    {
        int bin = cvRound((n / 360.f) * Ori[k]);
        if (bin >= n)
            bin -= n;
        if (bin < 0)
            bin += n;
        temphist[bin] += W[k] * Mag[k];
    }

    // smooth the histogram
    temphist[-1] = temphist[n - 1];
    temphist[-2] = temphist[n - 2];
    temphist[n] = temphist[0];
    temphist[n + 1] = temphist[1];

    i = 0;
#if CV_SIMD
    v_float32 d_1_16 = vx_setall_f32(1.f / 16.f);
    v_float32 d_4_16 = vx_setall_f32(4.f / 16.f);
    v_float32 d_6_16 = vx_setall_f32(6.f / 16.f);
    for (; i <= n - v_float32::nlanes; i += v_float32::nlanes)
    {
        v_float32 tn2 = vx_load_aligned(temphist + i - 2);
        v_float32 tn1 = vx_load(temphist + i - 1);
        v_float32 t0 = vx_load(temphist + i);
        v_float32 t1 = vx_load(temphist + i + 1);
        v_float32 t2 = vx_load(temphist + i + 2);
        v_float32 _hist = v_fma(tn2 + t2, d_1_16,
            v_fma(tn1 + t1, d_4_16, t0 * d_6_16));
        v_store(hist + i, _hist);
    }
#endif
    for (; i < n; i++)
    {
        hist[i] = (temphist[i - 2] + temphist[i + 2]) * (1.f / 16.f) +
            (temphist[i - 1] + temphist[i + 1]) * (4.f / 16.f) +
            temphist[i] * (6.f / 16.f);
    }

    float maxval = hist[0];
    for (i = 1; i < n; i++)
        maxval = max(maxval, hist[i]);

    return maxval;
}

static
bool adjustLocalExtrema(
    const std::vector<Mat>& dog_pyr, KeyPoint& kpt, int octv,
    int& layer, int& r, int& c, int nOctaveLayers,
    float contrastThreshold, float edgeThreshold, float sigma
)
{
    //CV_TRACE_FUNCTION();

    const float img_scale = 1.f / (255 * SIFT_FIXPT_SCALE);
    const float deriv_scale = img_scale * 0.5f;
    const float second_deriv_scale = img_scale;
    const float cross_deriv_scale = img_scale * 0.25f;

    float xi = 0, xr = 0, xc = 0, contr = 0;
    int i = 0;

    for (; i < SIFT_MAX_INTERP_STEPS; i++)
    {
        int idx = octv * (nOctaveLayers + 2) + layer;
        const Mat& img = dog_pyr[idx];
        const Mat& prev = dog_pyr[idx - 1];
        const Mat& next = dog_pyr[idx + 1];

        Vec3f dD((img.at<sift_wt>(r, c + 1) - img.at<sift_wt>(r, c - 1)) * deriv_scale,
            (img.at<sift_wt>(r + 1, c) - img.at<sift_wt>(r - 1, c)) * deriv_scale,
            (next.at<sift_wt>(r, c) - prev.at<sift_wt>(r, c)) * deriv_scale);

        float v2 = (float)img.at<sift_wt>(r, c) * 2;
        float dxx = (img.at<sift_wt>(r, c + 1) + img.at<sift_wt>(r, c - 1) - v2) * second_deriv_scale;
        float dyy = (img.at<sift_wt>(r + 1, c) + img.at<sift_wt>(r - 1, c) - v2) * second_deriv_scale;
        float dss = (next.at<sift_wt>(r, c) + prev.at<sift_wt>(r, c) - v2) * second_deriv_scale;
        float dxy = (img.at<sift_wt>(r + 1, c + 1) - img.at<sift_wt>(r + 1, c - 1) -
            img.at<sift_wt>(r - 1, c + 1) + img.at<sift_wt>(r - 1, c - 1)) * cross_deriv_scale;
        float dxs = (next.at<sift_wt>(r, c + 1) - next.at<sift_wt>(r, c - 1) -
            prev.at<sift_wt>(r, c + 1) + prev.at<sift_wt>(r, c - 1)) * cross_deriv_scale;
        float dys = (next.at<sift_wt>(r + 1, c) - next.at<sift_wt>(r - 1, c) -
            prev.at<sift_wt>(r + 1, c) + prev.at<sift_wt>(r - 1, c)) * cross_deriv_scale;

        Matx33f H(dxx, dxy, dxs,
            dxy, dyy, dys,
            dxs, dys, dss);

        Vec3f X = H.solve(dD, DECOMP_LU);

        xi = -X[2];
        xr = -X[1];
        xc = -X[0];

        if (std::abs(xi) < 0.5f && std::abs(xr) < 0.5f && std::abs(xc) < 0.5f)
            break;

        if (std::abs(xi) > (float)(INT_MAX / 3) ||
            std::abs(xr) > (float)(INT_MAX / 3) ||
            std::abs(xc) > (float)(INT_MAX / 3))
            return false;

        c += cvRound(xc);
        r += cvRound(xr);
        layer += cvRound(xi);

        if (layer < 1 || layer > nOctaveLayers ||
            c < SIFT_IMG_BORDER || c >= img.cols - SIFT_IMG_BORDER ||
            r < SIFT_IMG_BORDER || r >= img.rows - SIFT_IMG_BORDER)
            return false;
    }

    // ensure convergence of interpolation
    if (i >= SIFT_MAX_INTERP_STEPS)
        return false;

    {
        int idx = octv * (nOctaveLayers + 2) + layer;
        const Mat& img = dog_pyr[idx];
        const Mat& prev = dog_pyr[idx - 1];
        const Mat& next = dog_pyr[idx + 1];
        Matx31f dD((img.at<sift_wt>(r, c + 1) - img.at<sift_wt>(r, c - 1)) * deriv_scale,
            (img.at<sift_wt>(r + 1, c) - img.at<sift_wt>(r - 1, c)) * deriv_scale,
            (next.at<sift_wt>(r, c) - prev.at<sift_wt>(r, c)) * deriv_scale);
        float t = dD.dot(Matx31f(xc, xr, xi));

        contr = img.at<sift_wt>(r, c) * img_scale + t * 0.5f;
        if (std::abs(contr) * nOctaveLayers < contrastThreshold)
            return false;

        // principal curvatures are computed using the trace and det of Hessian
        float v2 = img.at<sift_wt>(r, c) * 2.f;
        float dxx = (img.at<sift_wt>(r, c + 1) + img.at<sift_wt>(r, c - 1) - v2) * second_deriv_scale;
        float dyy = (img.at<sift_wt>(r + 1, c) + img.at<sift_wt>(r - 1, c) - v2) * second_deriv_scale;
        float dxy = (img.at<sift_wt>(r + 1, c + 1) - img.at<sift_wt>(r + 1, c - 1) -
            img.at<sift_wt>(r - 1, c + 1) + img.at<sift_wt>(r - 1, c - 1)) * cross_deriv_scale;
        float tr = dxx + dyy;
        float det = dxx * dyy - dxy * dxy;

        if (det <= 0 || tr * tr * edgeThreshold >= (edgeThreshold + 1) * (edgeThreshold + 1) * det)
            return false;
    }

    kpt.pt.x = (c + xc) * (1 << octv);
    kpt.pt.y = (r + xr) * (1 << octv);
    kpt.octave = octv + (layer << 8) + (cvRound((xi + 0.5) * 255) << 16);
    kpt.size = sigma * powf(2.f, (layer + xi) / nOctaveLayers) * (1 << octv) * 2;
    kpt.response = std::abs(contr);

    return true;
}


class findScaleSpaceExtremaT
{
public:
    findScaleSpaceExtremaT(
        int _o,
        int _i,
        int _threshold,
        int _idx,
        int _step,
        int _cols,
        int _nOctaveLayers,
        double _contrastThreshold,
        double _edgeThreshold,
        double _sigma,
        const std::vector<Mat>& _gauss_pyr,
        const std::vector<Mat>& _dog_pyr,
        std::vector<KeyPoint>& kpts)

        : o(_o),
        i(_i),
        threshold(_threshold),
        idx(_idx),
        step(_step),
        cols(_cols),
        nOctaveLayers(_nOctaveLayers),
        contrastThreshold(_contrastThreshold),
        edgeThreshold(_edgeThreshold),
        sigma(_sigma),
        gauss_pyr(_gauss_pyr),
        dog_pyr(_dog_pyr),
        kpts_(kpts)
    {
        // nothing
    }
    void process(const cv::Range& range)
    {
        //CV_TRACE_FUNCTION();

        const int begin = range.start;
        const int end = range.end;

        static const int n = SIFT_ORI_HIST_BINS;
        float CV_DECL_ALIGNED(CV_SIMD_WIDTH) hist[n];

        const Mat& img = dog_pyr[idx];
        const Mat& prev = dog_pyr[idx - 1];
        const Mat& next = dog_pyr[idx + 1];

        for (int r = begin; r < end; r++)
        {
            const sift_wt* currptr = img.ptr<sift_wt>(r);
            const sift_wt* prevptr = prev.ptr<sift_wt>(r);
            const sift_wt* nextptr = next.ptr<sift_wt>(r);

            for (int c = SIFT_IMG_BORDER; c < cols - SIFT_IMG_BORDER; c++)
            {
                sift_wt val = currptr[c];

                // find local extrema with pixel accuracy
                if (std::abs(val) > threshold &&
                    ((val > 0 && val >= currptr[c - 1] && val >= currptr[c + 1] &&
                        val >= currptr[c - step - 1] && val >= currptr[c - step] && val >= currptr[c - step + 1] &&
                        val >= currptr[c + step - 1] && val >= currptr[c + step] && val >= currptr[c + step + 1] &&
                        val >= nextptr[c] && val >= nextptr[c - 1] && val >= nextptr[c + 1] &&
                        val >= nextptr[c - step - 1] && val >= nextptr[c - step] && val >= nextptr[c - step + 1] &&
                        val >= nextptr[c + step - 1] && val >= nextptr[c + step] && val >= nextptr[c + step + 1] &&
                        val >= prevptr[c] && val >= prevptr[c - 1] && val >= prevptr[c + 1] &&
                        val >= prevptr[c - step - 1] && val >= prevptr[c - step] && val >= prevptr[c - step + 1] &&
                        val >= prevptr[c + step - 1] && val >= prevptr[c + step] && val >= prevptr[c + step + 1]) ||
                        (val < 0 && val <= currptr[c - 1] && val <= currptr[c + 1] &&
                            val <= currptr[c - step - 1] && val <= currptr[c - step] && val <= currptr[c - step + 1] &&
                            val <= currptr[c + step - 1] && val <= currptr[c + step] && val <= currptr[c + step + 1] &&
                            val <= nextptr[c] && val <= nextptr[c - 1] && val <= nextptr[c + 1] &&
                            val <= nextptr[c - step - 1] && val <= nextptr[c - step] && val <= nextptr[c - step + 1] &&
                            val <= nextptr[c + step - 1] && val <= nextptr[c + step] && val <= nextptr[c + step + 1] &&
                            val <= prevptr[c] && val <= prevptr[c - 1] && val <= prevptr[c + 1] &&
                            val <= prevptr[c - step - 1] && val <= prevptr[c - step] && val <= prevptr[c - step + 1] &&
                            val <= prevptr[c + step - 1] && val <= prevptr[c + step] && val <= prevptr[c + step + 1])))
                {
                    //CV_TRACE_REGION("pixel_candidate");
                    //cout << "adjustLocalExtrema" << endl;

                    KeyPoint kpt;
                    int r1 = r, c1 = c, layer = i;
                    /*bool a = adjustLocalExtrema(dog_pyr, kpt, o, layer, r1, c1, nOctaveLayers, (float)contrastThreshold, (float)edgeThreshold, (float)sigma);
                   cout << "a : " << a << endl;*/
                    if (!adjustLocalExtrema(dog_pyr, kpt, o, layer, r1, c1,
                        nOctaveLayers, (float)contrastThreshold,
                        (float)edgeThreshold, (float)sigma))
                        continue;
                   // cout << "continue" << endl; 
                    float scl_octv = kpt.size * 0.5f / (1 << o);
                    float omax = calcOrientationHist(gauss_pyr[o * (nOctaveLayers + 3) + layer],
                        Point(c1, r1),
                        cvRound(SIFT_ORI_RADIUS * scl_octv),
                        SIFT_ORI_SIG_FCTR * scl_octv,
                        hist, n);
                    float mag_thr = (float)(omax * SIFT_ORI_PEAK_RATIO);
                    for (int j = 0; j < n; j++)
                    {
                        int l = j > 0 ? j - 1 : n - 1;
                        int r2 = j < n - 1 ? j + 1 : 0;

                        if (hist[j] > hist[l] && hist[j] > hist[r2] && hist[j] >= mag_thr)
                        {
                            float bin = j + 0.5f * (hist[l] - hist[r2]) / (hist[l] - 2 * hist[j] + hist[r2]);
                            bin = bin < 0 ? n + bin : bin >= n ? bin - n : bin;
                            kpt.angle = 360.f - (float)((360.f / n) * bin);
                            if (std::abs(kpt.angle - 360.f) < FLT_EPSILON)
                                kpt.angle = 0.f;

                            kpts_.push_back(kpt);
                        }
                    }
                }
            }
        }
    }
private:
    int o, i;
    int threshold;
    int idx, step, cols;
    int nOctaveLayers;
    double contrastThreshold;
    double edgeThreshold;
    double sigma;
    const std::vector<Mat>& gauss_pyr;
    const std::vector<Mat>& dog_pyr;
    std::vector<KeyPoint>& kpts_;
};



void findScaleSpaceExtrema( int octave,
    int layer,
    int threshold,
    int idx,
    int step,
    int cols,
    int nOctaveLayers,
    double contrastThreshold,
    double edgeThreshold,
    double sigma,
    const std::vector<Mat>& gauss_pyr,
    const std::vector<Mat>& dog_pyr,
    std::vector<KeyPoint>& kpts,
    const cv::Range& range)
{
    //CV_TRACE_FUNCTION();

    findScaleSpaceExtremaT(octave, layer, threshold, idx,
        step, cols,
        nOctaveLayers, contrastThreshold, edgeThreshold, sigma,
        gauss_pyr, dog_pyr,
        kpts)
        .process(range);
}

class findScaleSpaceExtremaComputer : public ParallelLoopBody_
{
public:
    findScaleSpaceExtremaComputer(
        int _o,
        int _i,
        int _threshold,
        int _idx,
        int _step,
        int _cols,
        int _nOctaveLayers,
        double _contrastThreshold,
        double _edgeThreshold,
        double _sigma,
        const std::vector<Mat>& _gauss_pyr,
        const std::vector<Mat>& _dog_pyr,
        TLSData<std::vector<KeyPoint> >& _tls_kpts_struct)

        : o(_o),
        i(_i),
        threshold(_threshold),
        idx(_idx),
        step(_step),
        cols(_cols),
        nOctaveLayers(_nOctaveLayers),
        contrastThreshold(_contrastThreshold),
        edgeThreshold(_edgeThreshold),
        sigma(_sigma),
        gauss_pyr(_gauss_pyr),
        dog_pyr(_dog_pyr),
        tls_kpts_struct(_tls_kpts_struct) { }
    void operator()(const cv::Range& range) const CV_OVERRIDE
    {
        //CV_TRACE_FUNCTION();

        std::vector<KeyPoint>& kpts = tls_kpts_struct.getRef();
        
        findScaleSpaceExtrema(o, i, threshold, idx, step, cols, nOctaveLayers, contrastThreshold, edgeThreshold, sigma, gauss_pyr, dog_pyr, kpts, range);
        
        //CV_CPU_DISPATCH(findScaleSpaceExtrema, (o, i, threshold, idx, step, cols, nOctaveLayers, contrastThreshold, edgeThreshold, sigma, gauss_pyr, dog_pyr, kpts, range),
        //    CV_CPU_DISPATCH_MODES_ALL);
    }
private:
    int o, i;
    int threshold;
    int idx, step, cols;
    int nOctaveLayers;
    double contrastThreshold;
    double edgeThreshold;
    double sigma;
    const std::vector<Mat>& gauss_pyr;
    const std::vector<Mat>& dog_pyr;
    TLSData<std::vector<KeyPoint> >& tls_kpts_struct;
};