#include "EstimateMotion.h"
#include <stdio.h>

void estimate_motion(MATCH *match, int match_num,
                     IPOINT *kp0, IPOINT *kp1,
                     FLOAT fx_in, FLOAT fy_in, FLOAT cx_in, FLOAT cy_in,
                     volatile FLOAT *depths,
                     int threshold_, FLOAT confidence_, int maxiter_,
                     FLOAT rmat[9], FLOAT tvec[3], bool take_last_)
{
#pragma HLS INTERFACE mode = m_axi depth = 500 port = match offset = slave bundle = BUS_A
#pragma HLS INTERFACE mode = m_axi depth = 500 port = kp0 offset = slave bundle = BUS_A
#pragma HLS INTERFACE mode = m_axi depth = 500 port = kp1 offset = slave
#pragma HLS INTERFACE mode = m_axi depth = 466616 port = depths offset = slave
#pragma HLS INTERFACE mode = m_axi depth = 9 port = rmat offset = slave
#pragma HLS INTERFACE mode = m_axi depth = 3 port = tvec offset = slave
#pragma HLS INTERFACE mode = s_axilite port = match
#pragma HLS INTERFACE mode = s_axilite port = kp0
#pragma HLS INTERFACE mode = s_axilite port = kp1
#pragma HLS INTERFACE mode = s_axilite port = depths
#pragma HLS INTERFACE mode = s_axilite port = rmat
#pragma HLS INTERFACE mode = s_axilite port = tvec
#pragma HLS INTERFACE mode = s_axilite port = match_num
#pragma HLS INTERFACE mode = s_axilite port = fx_in
#pragma HLS INTERFACE mode = s_axilite port = fy_in
#pragma HLS INTERFACE mode = s_axilite port = cx_in
#pragma HLS INTERFACE mode = s_axilite port = cy_in
#pragma HLS INTERFACE mode = s_axilite port = threshold_
#pragma HLS INTERFACE mode = s_axilite port = confidence_
#pragma HLS INTERFACE mode = s_axilite port = maxiter_
#pragma HLS INTERFACE mode = s_axilite port = take_last_
#pragma HLS INTERFACE mode = s_axilite port = return

    OPOINT opoint[MAX_KEYPOINT_NUM];
    IPOINT ipoint[MAX_KEYPOINT_NUM];

    FLOAT fx = fx_in, fy = fy_in, cx = cx_in, cy = cy_in;
    int point_num;
    bool take_last = take_last_;
    int threshold = threshold_;
    int maxiter = maxiter_;
    FLOAT confidence = confidence_;

    OPOINT opoint_inlier[MAX_KEYPOINT_NUM];
    IPOINT ipoint_inlier[MAX_KEYPOINT_NUM];

    FLOAT epnp_rmat[9];
    FLOAT epnp_tvec[3];
    bool best_mask[MAX_KEYPOINT_NUM];

    match_points(match, match_num, kp0, kp1, fx, fy, cx, cy, depths, point_num, opoint, ipoint);

    RANSAC_PnP(opoint, ipoint, point_num, fx, fy, cx, cy, threshold, confidence, maxiter, epnp_rmat, epnp_tvec, best_mask);

    int mask_num = 0;
    for (int i = 0; i < point_num; i++)
    {
        if (best_mask[i])
        {
            opoint_inlier[mask_num].x = opoint[i].x;
            opoint_inlier[mask_num].y = opoint[i].y;
            opoint_inlier[mask_num].z = opoint[i].z;

            ipoint_inlier[mask_num].x = ipoint[i].x;
            ipoint_inlier[mask_num].y = ipoint[i].y;
            mask_num++;
        }
    }

    pnp_iterative(opoint_inlier, ipoint_inlier, mask_num, epnp_rmat, epnp_tvec, fx, fy, cx, cy, rmat, tvec, take_last);

    return;
}

void match_points(MATCH *match, int match_num,
                  IPOINT *kp0, IPOINT *kp1,
                  FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy,
                  volatile FLOAT *depth, int &point_num,
                  OPOINT opoint[MAX_KEYPOINT_NUM],
                  IPOINT ipoint[MAX_KEYPOINT_NUM])
{
    point_num = 0;
    for (int i = 0; i < match_num; i++)
    {
        FLOAT u = kp0[match[i].a].x;
        FLOAT v = kp0[match[i].a].y;
        FLOAT z = depth[IMAGE_WIDTH * (int)v + (int)u];
        if (z > MAX_DEPTH)
            continue;

        opoint[point_num].x = z * (u - cx) / fx;
        opoint[point_num].y = z * (v - cy) / fy;
        opoint[point_num].z = z;

        ipoint[point_num].x = kp1[match[i].b].x;
        ipoint[point_num].y = kp1[match[i].b].y;
        point_num++;
    }
}

void RANSAC_PnP(OPOINT opoint[MAX_KEYPOINT_NUM],
                IPOINT ipoint[MAX_KEYPOINT_NUM],
                unsigned int point_num, FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy,
                int threshold, FLOAT confidence, int maxiter,
                FLOAT epnp_rmat[9], FLOAT epnp_tvec[3], bool best_mask[MAX_KEYPOINT_NUM])
{
    int maxGoodCount = 0;
    int niters = maxiter;
    RNG_state = (unsigned long long)-1;

    for (int ii = 0; ii < niters; ii++)
    {
        unsigned int idx[MODEL_POINTS];
        generate_5_indices(point_num, idx);

        OPOINT subopoint[MODEL_POINTS];
        IPOINT subipoint[MODEL_POINTS];
        for (int i = 0; i < MODEL_POINTS; i++)
        {
            subopoint[i] = opoint[idx[i]];
            subipoint[i] = ipoint[idx[i]];
        }

        epnp(subopoint, subipoint, fx, fy, cx, cy, epnp_rmat, epnp_tvec);

        IPOINT projPoint[MAX_KEYPOINT_NUM];
        projectPoint(opoint, point_num, fx, fy, cx, cy,
                     epnp_rmat, epnp_tvec, projPoint);

        bool mask[MAX_KEYPOINT_NUM];
        int goodCount = 0;
        for (int i = 0; i < point_num; i++)
        {
            FLOAT err = hls::pow(ipoint[i].x - projPoint[i].x, 2.f) +
                        hls::pow(ipoint[i].y - projPoint[i].y, 2.f);
            bool f = (err <= threshold);
            mask[i] = f;
            goodCount += f;
        }

        if (goodCount > hls::max(maxGoodCount, 4))
        {
            for (int i = 0; i < point_num; i++)
            {
                best_mask[i] = mask[i];
            }
            maxGoodCount = goodCount;

            FLOAT ep = (FLOAT)(point_num - goodCount) / point_num;
            ep = hls::max(ep, 0.f);
            ep = hls::min(ep, 1.f);

            FLOAT denom = 1.f - pow(1.f - ep, 5.f);
            if (denom < FLT_MIN)
            {
                niters = 0;
            }
            else
            {
                FLOAT num = hls::log(1.f - confidence);
                denom = hls::log(denom);
                if (denom < 0 && num > niters * denom)
                    niters = hls::round(num / denom);
            }
        }
    }
}

void generate_5_indices(unsigned int point_num, unsigned int idx[MODEL_POINTS])
{
#pragma HLS ARRAY_PARTITION variable = idx type = complete

    for (int i = 0; i < MODEL_POINTS; i++)
    {
#pragma HLS UNROLL
        idx[i] = 0;
    }

    for (int i = 0; i < MODEL_POINTS; i++)
    {
    generate:
        unsigned int random_num = RNG(point_num);

        for (int j = 0; j < MODEL_POINTS; j++)
        {
#pragma HLS UNROLL
            if (idx[j] == random_num)
                goto generate;
        }

        idx[i] = random_num;
    }
}

unsigned int RNG(unsigned int a)
{
    if (a != 0)
    {
        RNG_state = (unsigned)RNG_state * RNG_COEF + (RNG_state >> 32);
        return (unsigned int)((unsigned int)RNG_state % a);
    }
    else
        return 0;
}

void projectPoint(OPOINT opoint[MAX_KEYPOINT_NUM], int point_num, int fx, int fy, int cx, int cy,
                  FLOAT rmat[9], FLOAT tvec[3], IPOINT projPoint[MAX_KEYPOINT_NUM])

{
    for (int i = 0; i < point_num; i++)
    {
        OPOINT rotpoint;
        rotpoint.x = rmat[0] * opoint[i].x + rmat[1] * opoint[i].y + rmat[2] * opoint[i].z + tvec[0];
        rotpoint.y = rmat[3] * opoint[i].x + rmat[4] * opoint[i].y + rmat[5] * opoint[i].z + tvec[1];
        rotpoint.z = rmat[6] * opoint[i].x + rmat[7] * opoint[i].y + rmat[8] * opoint[i].z + tvec[2];

        projPoint[i].x = rotpoint.x * fx / rotpoint.z + cx;
        projPoint[i].y = rotpoint.y * fy / rotpoint.z + cy;
    }
}
