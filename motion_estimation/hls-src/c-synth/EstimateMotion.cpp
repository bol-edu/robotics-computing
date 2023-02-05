#include "EstimateMotion.h"
#include <stdio.h>

#define DBL_MIN 2.2250738585072014e-308

int32_t *FIND(int32_t *first, int32_t *last, int32_t val)
{
    for (int32_t *i = first; i != last; ++i)
    {
        if (*i == val)
        {
            return i;
        }
    }
    return last;
}

/*void estimate_motion(Matrix<int32_t, MAX_KEYPOINT_NUM, 2> &match,
                     Matrix<FLOAT, MAX_KEYPOINT_NUM, 2> &kp0,
                     Matrix<FLOAT, MAX_KEYPOINT_NUM, 2> &kp1,
                     Matrix<FLOAT, 3, 3> &k,
                     Matrix<FLOAT, IMAGE_HEIGTH, IMAGE_WIDTH> &depth,
                     Matrix<FLOAT, 3, 3> &_rmat,
                     Matrix<FLOAT, 3, 1> &_tvec)*/
void estimate_motion(volatile int32_t *match_val, unsigned int match_m,
                     volatile FLOAT *kp0_val, unsigned int kp0_m,
                     volatile FLOAT *kp1_val, unsigned int kp1_m,
                     volatile FLOAT *k_val, volatile FLOAT *depth_val,
                     volatile FLOAT *rmat, volatile FLOAT *tvec)
{
#pragma HLS INTERFACE mode=m_axi depth=128 port=match_val offset=slave
#pragma HLS INTERFACE mode=m_axi depth=128 port=kp0_val offset=slave
#pragma HLS INTERFACE mode=m_axi depth=128 port=kp1_val offset=slave
#pragma HLS INTERFACE mode=m_axi depth=16 port=k_val offset=slave
#pragma HLS INTERFACE mode=m_axi depth=128 port=depth_val offset=slave
#pragma HLS INTERFACE mode=m_axi depth=16 port=rmat offset=slave
#pragma HLS INTERFACE mode=m_axi depth=4 port=tvec offset=slave
#pragma HLS INTERFACE mode=s_axilite port=match_val
#pragma HLS INTERFACE mode=s_axilite port=kp0_val
#pragma HLS INTERFACE mode=s_axilite port=kp1_val
#pragma HLS INTERFACE mode=s_axilite port=k_val
#pragma HLS INTERFACE mode=s_axilite port=depth_val
#pragma HLS INTERFACE mode=s_axilite port=rmat
#pragma HLS INTERFACE mode=s_axilite port=tvec
#pragma HLS INTERFACE mode=s_axilite port=match_m
#pragma HLS INTERFACE mode=s_axilite port=kp0_m
#pragma HLS INTERFACE mode=s_axilite port=kp1_m
#pragma HLS INTERFACE mode = s_axilite port = return

    for (int i = 0; i < IMAGE_HEIGTH; i++)
        memset(tmpArray[i], 0, IMAGE_WIDTH);

    Matrix<int32_t, MAX_KEYPOINT_NUM, 2> match = Matrix<int32_t, MAX_KEYPOINT_NUM, 2>();
    Matrix<FLOAT, MAX_KEYPOINT_NUM, 2> kp0 = Matrix<FLOAT, MAX_KEYPOINT_NUM, 2>();
    Matrix<FLOAT, MAX_KEYPOINT_NUM, 2> kp1 = Matrix<FLOAT, MAX_KEYPOINT_NUM, 2>();
    Matrix<FLOAT, 3, 3> k = Matrix<FLOAT, 3, 3>();
    Matrix<FLOAT, IMAGE_HEIGTH, IMAGE_WIDTH> depth = Matrix<FLOAT, IMAGE_HEIGTH, IMAGE_WIDTH>();

    for (int32_t i = 0; i < match_m; i++)
    {
        match.val[i][0] = match_val[2 * i];
        match.val[i][1] = match_val[2 * i + 1];
    }
    match.m = match_m;

    for (int32_t i = 0; i < kp0_m; i++)
    {
        kp0.val[i][0] = kp0_val[2 * i];
        kp0.val[i][1] = kp0_val[2 * i + 1];
    }
    kp0.m = kp0_m;

    for (int32_t i = 0; i < kp1_m; i++)
    {
        kp1.val[i][0] = kp1_val[2 * i];
        kp1.val[i][1] = kp1_val[2 * i + 1];
    }
    kp1.m = kp1_m;

    for (int32_t i = 0; i < 3; i++)
        for (int32_t j = 0; j < 3; j++)
            k.val[i][j] = k_val[3 * i + j];

    for (int32_t i = 0; i < IMAGE_HEIGTH; i++)
        for (int32_t j = 0; j < IMAGE_WIDTH; j++)
            depth.val[i][j] = depth_val[IMAGE_WIDTH * i + j];


    EstimateMotion em = EstimateMotion();
    em.estimate(match, kp0, kp1, k, depth);

    rmat[9];
    tvec[3];
    for (int32_t i = 0; i < 3; i++)
        for (int32_t j = 0; j < 3; j++)
            rmat[3 * i + j] = em.rmat.val[i][j];
    for (int32_t i = 0; i < 3; i++)
        tvec[i] = em.tvec.val[i][0];

    return;
}

EstimateMotion::EstimateMotion()
{
}

void EstimateMotion::estimate(Matrix<int32_t, MAX_KEYPOINT_NUM, 2> &match,
                              Matrix<FLOAT, MAX_KEYPOINT_NUM, 2> &kp0,
                              Matrix<FLOAT, MAX_KEYPOINT_NUM, 2> &kp1,
                              Matrix<FLOAT, 3, 3> &k,
                              Matrix<FLOAT, IMAGE_HEIGTH, IMAGE_WIDTH> &depth)
{
    opoint = Matrix<FLOAT, MAX_KEYPOINT_NUM, 3>();
    ipoint = Matrix<FLOAT, MAX_KEYPOINT_NUM, 2>();

    cx = k.val[0][2];
    cy = k.val[1][2];
    fx = k.val[0][0];
    fy = k.val[1][1];

    int32_t j = 0;
    for (int32_t i = 0; i < match.m; i++)
    {
        FLOAT u = kp0.val[(int)(match.val[i][0])][0];
        FLOAT v = kp0.val[(int)(match.val[i][0])][1];
        FLOAT z = depth.val[(int)v][(int)u];
        if (z > MAX_DEPTH)
            continue;

        opoint.val[j][0] = z * (u - cx) / fx;
        opoint.val[j][1] = z * (v - cy) / fy;
        opoint.val[j][2] = z;

        ipoint.val[j][0] = kp1.val[(int)(match.val[i][1])][0];
        ipoint.val[j][1] = kp1.val[(int)(match.val[i][1])][1];
        j++;
    }
    opoint.setRow(j);
    ipoint.setRow(j);
    RANSAC_PnP();
}

void EstimateMotion::RANSAC_PnP()
{
    int32_t maxGoodCount = 0;
    int32_t count = opoint.m;
    int32_t niters = MAXITER;
    RNG_state = (unsigned long long)-1;
    bool best_mask[MAX_KEYPOINT_NUM];
    for (int32_t ii = 0; ii < niters; ii++)
    {
        getSubset();
        EPnP epnp = EPnP(subopoint, subipoint, fx, fy, cx, cy);
        epnp.compute();

        projPoint = projectPoint(epnp.rmat, epnp.tvec);

        err = Matrix<FLOAT, MAX_KEYPOINT_NUM, 1>();
        for (int32_t i = 0; i < count; i++)
        {
            err.val[i][0] = SQR(ipoint.val[i][0] - projPoint.val[i][0]) +
                            SQR(ipoint.val[i][1] - projPoint.val[i][1]);
        }

        bool mask[MAX_KEYPOINT_NUM];
        int32_t goodCount = 0;
        for (int32_t i = 0; i < count; i++)
        {
            int32_t f = err.val[i][0] <= SQR(THRESHOLD);
            mask[i] = f;
            goodCount += f;
        }
        if (goodCount > MAX(maxGoodCount, 4))
        {
            for (int i = 0; i < count; i++)
                best_mask[i] = mask[i];
            maxGoodCount = goodCount;
            {
                FLOAT ep = (double)(count - goodCount) / count;
                ep = MAX(ep, 0.);
                ep = MIN(ep, 1.);

                FLOAT denom = 1. - pow(1. - ep, 5);
                if (denom < DBL_MIN)
                    niters = 0;

                FLOAT num = log(1 - CONFIDENCE);
                denom = log(denom);

                niters = denom >= 0 || -num >= niters * (-denom) ? niters : round(num / denom);
            }
        }
    }

    opoint_inlier = Matrix<FLOAT, MAX_KEYPOINT_NUM, 3>();
    ipoint_inlier = Matrix<FLOAT, MAX_KEYPOINT_NUM, 2>();
    int32_t j = 0;
    for (int32_t i = 0; i < count; i++)
    {
        if (best_mask[i])
        {
            opoint_inlier.val[j][0] = opoint.val[i][0];
            opoint_inlier.val[j][1] = opoint.val[i][1];
            opoint_inlier.val[j][2] = opoint.val[i][2];

            ipoint_inlier.val[j][0] = ipoint.val[i][0];
            ipoint_inlier.val[j][1] = ipoint.val[i][1];
            j++;
        }
    }
    opoint_inlier.setRow(j);
    ipoint_inlier.setRow(j);

    PnPIterative pnp_iter = PnPIterative(opoint_inlier, ipoint_inlier, fx, fy, cx, cy);
    pnp_iter.compute(rmat, tvec);
}

void EstimateMotion::getSubset()
{
    subopoint = Matrix<FLOAT, MODEL_POINTS, 3>();
    subipoint = Matrix<FLOAT, MODEL_POINTS, 2>();

    int32_t idx[MODEL_POINTS];
    for (int32_t i = 0; i < MODEL_POINTS; i++)
    {
        unsigned long long idx_i;
        for (idx_i = RNG_uniform(0, opoint.m);
             FIND(idx, idx + i, idx_i) != idx + i;
             idx_i = RNG_uniform(0, opoint.m))
        {
        }

        idx[i] = idx_i;

        subopoint.val[i][0] = opoint.val[idx[i]][0];
        subopoint.val[i][1] = opoint.val[idx[i]][1];
        subopoint.val[i][2] = opoint.val[idx[i]][2];
        subipoint.val[i][0] = ipoint.val[idx[i]][0];
        subipoint.val[i][1] = ipoint.val[idx[i]][1];
    }
}

Matrix<FLOAT, MAX_KEYPOINT_NUM, 2> EstimateMotion::projectPoint(const Matrix<FLOAT, 3, 3> rmat, const Matrix<FLOAT, 3, 1> tvec)

{
    Matrix<FLOAT, MAX_KEYPOINT_NUM, 2> projPoint = Matrix<FLOAT, MAX_KEYPOINT_NUM, 2>();
    Matrix<FLOAT, 3, 1> opoint_1 = Matrix<FLOAT, 3, 1>();
    Matrix<FLOAT, 3, 1> rotpoint;
    for (int32_t i = 0; i < opoint.m; i++)
    {
        opoint_1.val[0][0] = opoint.val[i][0];
        opoint_1.val[1][0] = opoint.val[i][1];
        opoint_1.val[2][0] = opoint.val[i][2];
        rotpoint = rmat * opoint_1 + tvec;

        projPoint.val[i][0] = rotpoint.val[0][0] * fx / rotpoint.val[2][0] + cx;
        projPoint.val[i][1] = rotpoint.val[1][0] * fy / rotpoint.val[2][0] + cy;
    }
    return projPoint;
}

long long EstimateMotion::RNG_uniform(long long a, long long b)
{
    return a == b ? a : (long long)(RNG_next() % (b - a) + a);
}

unsigned EstimateMotion::RNG_next()
{
    RNG_state = (unsigned long long)(unsigned)RNG_state * RNG_COEF + (unsigned)(RNG_state >> 32);
    return (unsigned)RNG_state;
}
