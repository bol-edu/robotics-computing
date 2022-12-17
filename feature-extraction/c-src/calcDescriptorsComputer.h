#pragma once
//#include <opencv2/opencv.hpp>
//#include "BufferArea__.h"
using namespace std;

static const int SIFT_DESCR_WIDTH = 4;
static const int SIFT_DESCR_HIST_BINS = 8;
static const float SIFT_DESCR_SCL_FCTR = 3.f;
static const float SIFT_DESCR_MAG_THR = 0.2f;
static const float SIFT_INT_DESCR_FCTR = 512.f;


void calcSIFTDescriptor(
    const Mat& img, Point2f ptf, float ori, float scl,
    int d, int n, Mat& dstMat, int row
)
{
    //CV_TRACE_FUNCTION();

    Point pt(cvRound(ptf.x), cvRound(ptf.y));
    float cos_t = cosf(ori * (float)(CV_PI / 180));
    float sin_t = sinf(ori * (float)(CV_PI / 180));
    float bins_per_rad = n / 360.f;
    float exp_scale = -1.f / (d * d * 0.5f);
    float hist_width = SIFT_DESCR_SCL_FCTR * scl;
    int radius = cvRound(hist_width * 1.4142135623730951f * (d + 1) * 0.5f);
    // Clip the radius to the diagonal of the image to avoid autobuffer too large exception
    radius = min(radius, (int)std::sqrt(((double)img.cols) * img.cols + ((double)img.rows) * img.rows));
    cos_t /= hist_width;
    sin_t /= hist_width;

    int i, j, k, len = (radius * 2 + 1) * (radius * 2 + 1), histlen = (d + 2) * (d + 2) * (n + 2);
    int rows = img.rows, cols = img.cols;

    BufferArea__ area;
    float* X = 0, * Y = 0, * Mag, * Ori = 0, * W = 0, * RBin = 0, * CBin = 0, * hist = 0, * rawDst = 0;
    area.allocate(X, len, CV_SIMD_WIDTH);
    area.allocate(Y, len, CV_SIMD_WIDTH);
    area.allocate(Ori, len, CV_SIMD_WIDTH);
    area.allocate(W, len, CV_SIMD_WIDTH);
    area.allocate(RBin, len, CV_SIMD_WIDTH);
    area.allocate(CBin, len, CV_SIMD_WIDTH);
    area.allocate(hist, histlen, CV_SIMD_WIDTH);
    area.allocate(rawDst, len, CV_SIMD_WIDTH);
    area.commit();
    Mag = Y;

    for (i = 0; i < d + 2; i++)
    {
        for (j = 0; j < d + 2; j++)
            for (k = 0; k < n + 2; k++)
                hist[(i * (d + 2) + j) * (n + 2) + k] = 0.;
    }

    for (i = -radius, k = 0; i <= radius; i++)
        for (j = -radius; j <= radius; j++)
        {
            // Calculate sample's histogram array coords rotated relative to ori.
            // Subtract 0.5 so samples that fall e.g. in the center of row 1 (i.e.
            // r_rot = 1.5) have full weight placed in row 1 after interpolation.
            float c_rot = j * cos_t - i * sin_t;
            float r_rot = j * sin_t + i * cos_t;
            float rbin = r_rot + d / 2 - 0.5f;
            float cbin = c_rot + d / 2 - 0.5f;
            int r = pt.y + i, c = pt.x + j;

            if (rbin > -1 && rbin < d && cbin > -1 && cbin < d &&
                r > 0 && r < rows - 1 && c > 0 && c < cols - 1)
            {
                float dx = (float)(img.at<sift_wt>(r, c + 1) - img.at<sift_wt>(r, c - 1));
                float dy = (float)(img.at<sift_wt>(r - 1, c) - img.at<sift_wt>(r + 1, c));
                X[k] = dx; Y[k] = dy; RBin[k] = rbin; CBin[k] = cbin;
                W[k] = (c_rot * c_rot + r_rot * r_rot) * exp_scale;
                k++;
            }
        }

    len = k;
    fastAtan2(Y, X, Ori, len, true);
    magnitude32f(X, Y, Mag, len);
    exp32f(W, W, len);

    k = 0;
#if CV_SIMD
    {
        const int vecsize = v_float32::nlanes;
        int CV_DECL_ALIGNED(CV_SIMD_WIDTH) idx_buf[vecsize];
        float CV_DECL_ALIGNED(CV_SIMD_WIDTH) rco_buf[8 * vecsize];
        const v_float32 __ori = vx_setall_f32(ori);
        const v_float32 __bins_per_rad = vx_setall_f32(bins_per_rad);
        const v_int32 __n = vx_setall_s32(n);
        const v_int32 __1 = vx_setall_s32(1);
        const v_int32 __d_plus_2 = vx_setall_s32(d + 2);
        const v_int32 __n_plus_2 = vx_setall_s32(n + 2);
        for (; k <= len - vecsize; k += vecsize)
        {
            v_float32 rbin = vx_load_aligned(RBin + k);
            v_float32 cbin = vx_load_aligned(CBin + k);
            v_float32 obin = (vx_load_aligned(Ori + k) - __ori) * __bins_per_rad;
            v_float32 mag = vx_load_aligned(Mag + k) * vx_load_aligned(W + k);

            v_int32 r0 = v_floor(rbin);
            v_int32 c0 = v_floor(cbin);
            v_int32 o0 = v_floor(obin);
            rbin -= v_cvt_f32(r0);
            cbin -= v_cvt_f32(c0);
            obin -= v_cvt_f32(o0);

            o0 = v_select(o0 < vx_setzero_s32(), o0 + __n, o0);
            o0 = v_select(o0 >= __n, o0 - __n, o0);

            v_float32 v_r1 = mag * rbin, v_r0 = mag - v_r1;
            v_float32 v_rc11 = v_r1 * cbin, v_rc10 = v_r1 - v_rc11;
            v_float32 v_rc01 = v_r0 * cbin, v_rc00 = v_r0 - v_rc01;
            v_float32 v_rco111 = v_rc11 * obin, v_rco110 = v_rc11 - v_rco111;
            v_float32 v_rco101 = v_rc10 * obin, v_rco100 = v_rc10 - v_rco101;
            v_float32 v_rco011 = v_rc01 * obin, v_rco010 = v_rc01 - v_rco011;
            v_float32 v_rco001 = v_rc00 * obin, v_rco000 = v_rc00 - v_rco001;

            v_int32 idx = v_muladd(v_muladd(r0 + __1, __d_plus_2, c0 + __1), __n_plus_2, o0);
            v_store_aligned(idx_buf, idx);

            v_store_aligned(rco_buf, v_rco000);
            v_store_aligned(rco_buf + vecsize, v_rco001);
            v_store_aligned(rco_buf + vecsize * 2, v_rco010);
            v_store_aligned(rco_buf + vecsize * 3, v_rco011);
            v_store_aligned(rco_buf + vecsize * 4, v_rco100);
            v_store_aligned(rco_buf + vecsize * 5, v_rco101);
            v_store_aligned(rco_buf + vecsize * 6, v_rco110);
            v_store_aligned(rco_buf + vecsize * 7, v_rco111);

            for (int id = 0; id < vecsize; id++)
            {
                hist[idx_buf[id]] += rco_buf[id];
                hist[idx_buf[id] + 1] += rco_buf[vecsize + id];
                hist[idx_buf[id] + (n + 2)] += rco_buf[2 * vecsize + id];
                hist[idx_buf[id] + (n + 3)] += rco_buf[3 * vecsize + id];
                hist[idx_buf[id] + (d + 2) * (n + 2)] += rco_buf[4 * vecsize + id];
                hist[idx_buf[id] + (d + 2) * (n + 2) + 1] += rco_buf[5 * vecsize + id];
                hist[idx_buf[id] + (d + 3) * (n + 2)] += rco_buf[6 * vecsize + id];
                hist[idx_buf[id] + (d + 3) * (n + 2) + 1] += rco_buf[7 * vecsize + id];
            }
        }
    }
#endif
    for (; k < len; k++)
    {
        float rbin = RBin[k], cbin = CBin[k];
        float obin = (Ori[k] - ori) * bins_per_rad;
        float mag = Mag[k] * W[k];

        int r0 = cvFloor(rbin);
        int c0 = cvFloor(cbin);
        int o0 = cvFloor(obin);
        rbin -= r0;
        cbin -= c0;
        obin -= o0;

        if (o0 < 0)
            o0 += n;
        if (o0 >= n)
            o0 -= n;

        // histogram update using tri-linear interpolation
        float v_r1 = mag * rbin, v_r0 = mag - v_r1;
        float v_rc11 = v_r1 * cbin, v_rc10 = v_r1 - v_rc11;
        float v_rc01 = v_r0 * cbin, v_rc00 = v_r0 - v_rc01;
        float v_rco111 = v_rc11 * obin, v_rco110 = v_rc11 - v_rco111;
        float v_rco101 = v_rc10 * obin, v_rco100 = v_rc10 - v_rco101;
        float v_rco011 = v_rc01 * obin, v_rco010 = v_rc01 - v_rco011;
        float v_rco001 = v_rc00 * obin, v_rco000 = v_rc00 - v_rco001;

        int idx = ((r0 + 1) * (d + 2) + c0 + 1) * (n + 2) + o0;
        hist[idx] += v_rco000;
        hist[idx + 1] += v_rco001;
        hist[idx + (n + 2)] += v_rco010;
        hist[idx + (n + 3)] += v_rco011;
        hist[idx + (d + 2) * (n + 2)] += v_rco100;
        hist[idx + (d + 2) * (n + 2) + 1] += v_rco101;
        hist[idx + (d + 3) * (n + 2)] += v_rco110;
        hist[idx + (d + 3) * (n + 2) + 1] += v_rco111;
    }

    // finalize histogram, since the orientation histograms are circular
    for (i = 0; i < d; i++)
        for (j = 0; j < d; j++)
        {
            int idx = ((i + 1) * (d + 2) + (j + 1)) * (n + 2);
            hist[idx] += hist[idx + n];
            hist[idx + 1] += hist[idx + n + 1];
            for (k = 0; k < n; k++)
                rawDst[(i * d + j) * n + k] = hist[idx + k];
        }
    // copy histogram to the descriptor,
    // apply hysteresis thresholding
    // and scale the result, so that it can be easily converted
    // to byte array
    float nrm2 = 0;
    len = d * d * n;
    k = 0;
#if CV_SIMD
    {
        v_float32 __nrm2 = vx_setzero_f32();
        v_float32 __rawDst;
        for (; k <= len - v_float32::nlanes; k += v_float32::nlanes)
        {
            __rawDst = vx_load_aligned(rawDst + k);
            __nrm2 = v_fma(__rawDst, __rawDst, __nrm2);
        }
        nrm2 = (float)v_reduce_sum(__nrm2);
    }
#endif
    for (; k < len; k++)
        nrm2 += rawDst[k] * rawDst[k];

    float thr = std::sqrt(nrm2) * SIFT_DESCR_MAG_THR;

    i = 0, nrm2 = 0;
#if 0 //CV_AVX2
    // This code cannot be enabled because it sums nrm2 in a different order,
    // thus producing slightly different results
    {
        float CV_DECL_ALIGNED(CV_SIMD_WIDTH) nrm2_buf[8];
        __m256 __dst;
        __m256 __nrm2 = _mm256_setzero_ps();
        __m256 __thr = _mm256_set1_ps(thr);
        for (; i <= len - 8; i += 8)
        {
            __dst = _mm256_loadu_ps(&rawDst[i]);
            __dst = _mm256_min_ps(__dst, __thr);
            _mm256_storeu_ps(&rawDst[i], __dst);
#if CV_FMA3
            __nrm2 = _mm256_fmadd_ps(__dst, __dst, __nrm2);
#else
            __nrm2 = _mm256_add_ps(__nrm2, _mm256_mul_ps(__dst, __dst));
#endif
        }
        _mm256_store_ps(nrm2_buf, __nrm2);
        nrm2 = nrm2_buf[0] + nrm2_buf[1] + nrm2_buf[2] + nrm2_buf[3] +
            nrm2_buf[4] + nrm2_buf[5] + nrm2_buf[6] + nrm2_buf[7];
    }
#endif
    for (; i < len; i++)
    {
        float val = min(rawDst[i], thr);
        rawDst[i] = val;
        nrm2 += val * val;
    }
    nrm2 = SIFT_INT_DESCR_FCTR / max(std::sqrt(nrm2), FLT_EPSILON);

#if 1
    k = 0;
    if (dstMat.type() == CV_32F)
    {
        float* dst = dstMat.ptr<float>(row);
#if CV_SIMD
        v_float32 __dst;
        v_float32 __min = vx_setzero_f32();
        v_float32 __max = vx_setall_f32(255.0f); // max of uchar
        v_float32 __nrm2 = vx_setall_f32(nrm2);
        for (k = 0; k <= len - v_float32::nlanes; k += v_float32::nlanes)
        {
            __dst = vx_load_aligned(rawDst + k);
            __dst = v_min(v_max(v_cvt_f32(v_round(__dst * __nrm2)), __min), __max);
            v_store(dst + k, __dst);
        }
#endif
        for (; k < len; k++)
        {
            dst[k] = saturate_cast<uchar>(rawDst[k] * nrm2);
        }
    }
    else // CV_8U
    {
        uint8_t* dst = dstMat.ptr<uint8_t>(row);
#if CV_SIMD
        v_float32 __dst0, __dst1;
        v_uint16 __pack01;
        v_float32 __nrm2 = vx_setall_f32(nrm2);
        for (k = 0; k <= len - v_float32::nlanes * 2; k += v_float32::nlanes * 2)
        {
            __dst0 = vx_load_aligned(rawDst + k);
            __dst1 = vx_load_aligned(rawDst + k + v_float32::nlanes);

            __pack01 = v_pack_u(v_round(__dst0 * __nrm2), v_round(__dst1 * __nrm2));
            v_pack_store(dst + k, __pack01);
        }
#endif
        for (; k < len; k++)
        {
            dst[k] = saturate_cast<uchar>(rawDst[k] * nrm2);
        }
    }
#else
    float* dst = dstMat.ptr<float>(row);
    float nrm1 = 0;
    for (k = 0; k < len; k++)
    {
        rawDst[k] *= nrm2;
        nrm1 += rawDst[k];
    }
    nrm1 = 1.f / std::max(nrm1, FLT_EPSILON);
    if (dstMat.type() == CV_32F)
    {
        for (k = 0; k < len; k++)
        {
            dst[k] = std::sqrt(rawDst[k] * nrm1);
        }
    }
    else // CV_8U
    {
        for (k = 0; k < len; k++)
        {
            dst[k] = saturate_cast<uchar>(std::sqrt(rawDst[k] * nrm1) * SIFT_INT_DESCR_FCTR);
        }
    }
#endif
}

/*static
void calcSIFTDescriptor(
    const Mat& img, Point2f ptf, float ori, float scl,
    int d, int n, Mat& dst, int row
)
{
    //CV_TRACE_FUNCTION();
    calcSIFTDescriptor(img, ptf, ori, scl, d, n, dst, row);
    //CV_CPU_DISPATCH(calcSIFTDescriptor, (img, ptf, ori, scl, d, n, dst, row),
     //  CV_CPU_DISPATCH_MODES_ALL);
}*/

static inline void
unpackOctave_(const KeyPoint& kpt, int& octave, int& layer, float& scale)
{
    octave = kpt.octave & 255;
    layer = (kpt.octave >> 8) & 255;
    octave = octave < 128 ? octave : (-128 | octave);
    scale = octave >= 0 ? 1.f / (1 << octave) : (float)(1 << -octave);
}

class calcDescriptorsComputer : public ParallelLoopBody_
{
public:
    calcDescriptorsComputer(const std::vector<Mat>& _gpyr,
        const std::vector<KeyPoint>& _keypoints,
        Mat& _descriptors,
        int _nOctaveLayers,
        int _firstOctave)
        : gpyr(_gpyr),
        keypoints(_keypoints),
        descriptors(_descriptors),
        nOctaveLayers(_nOctaveLayers),
        firstOctave(_firstOctave) { }

    void operator()(const cv::Range& range) const CV_OVERRIDE
    {
        //CV_TRACE_FUNCTION();

        const int begin = range.start;
        const int end = range.end;

        static const int d = SIFT_DESCR_WIDTH, n = SIFT_DESCR_HIST_BINS;

        for (int i = begin; i < end; i++)
        {
            KeyPoint kpt = keypoints[i];
            int octave, layer;
            float scale;
            unpackOctave_(kpt, octave, layer, scale);
            CV_Assert(octave >= firstOctave && layer <= nOctaveLayers + 2);
            float size = kpt.size * scale;
            Point2f ptf(kpt.pt.x * scale, kpt.pt.y * scale);
            const Mat& img = gpyr[(octave - firstOctave) * (nOctaveLayers + 3) + layer];

            float angle = 360.f - kpt.angle;
            if (std::abs(angle - 360.f) < FLT_EPSILON)
                angle = 0.f;
            calcSIFTDescriptor(img, ptf, angle, size * 0.5f, d, n, descriptors, i);
        }
    }
private:
    const std::vector<Mat>& gpyr;
    const std::vector<KeyPoint>& keypoints;
    Mat& descriptors;
    int nOctaveLayers;
    int firstOctave;
};










