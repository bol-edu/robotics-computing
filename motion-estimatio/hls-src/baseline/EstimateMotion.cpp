#include "EstimateMotion.h"
#include <stdio.h>

//#define DBL_MIN 2.2250738585072014e-308

void estimate_motion(volatile int* match, int match_num,
					 volatile FLOAT* kp0, volatile FLOAT* kp1,
					 FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy,
					 volatile FLOAT* depths,
                     volatile FLOAT *rmat, volatile FLOAT *tvec)
{
#pragma HLS INTERFACE mode = m_axi depth = 1000 port = match offset = slave
#pragma HLS INTERFACE mode = m_axi depth = 1000 port = kp0 offset = slave
#pragma HLS INTERFACE mode = m_axi depth = 1000 port = kp1 offset = slave
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
#pragma HLS INTERFACE mode = s_axilite port = fx
#pragma HLS INTERFACE mode = s_axilite port = fy
#pragma HLS INTERFACE mode = s_axilite port = cx
#pragma HLS INTERFACE mode = s_axilite port = cy
#pragma HLS INTERFACE mode = s_axilite port = return

    OPOINT opoint[MAX_KEYPOINT_NUM];
    IPOINT ipoint[MAX_KEYPOINT_NUM];
    unsigned int point_num;

    match_points(match, match_num, kp0, kp1, fx, fy, cx, cy, depths, point_num, opoint, ipoint);

    RANSAC_PnP(opoint, ipoint, point_num, fx, fy, cx, cy, rmat, tvec);

    /*rmat[9];
    tvec[3];
    for (int32_t i = 0; i < 3; i++)
        for (int32_t j = 0; j < 3; j++)
            rmat[3 * i + j] = em.rmat.val[i][j];
    for (int32_t i = 0; i < 3; i++)
        tvec[i] = em.tvec.val[i][0];*/

    return;
}

void match_points(volatile int* match, int match_num,
					volatile FLOAT* kp0, volatile FLOAT* kp1,
					FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy,
					volatile FLOAT* depth, unsigned int &point_num,
					OPOINT opoint[MAX_KEYPOINT_NUM],
					IPOINT ipoint[MAX_KEYPOINT_NUM])
{
	point_num = 0;
    for (int i = 0; i < match_num; i++)
    {
        FLOAT u = kp0[2 * match[2 * i]];
        FLOAT v = kp0[2 * match[2 * i] + 1];
        FLOAT z = depth[IMAGE_WIDTH * (int)v + (int)u];
        if (z > MAX_DEPTH)
            continue;

        opoint[point_num].x = z * (u - cx) / fx;
        opoint[point_num].y = z * (v - cy) / fy;
        opoint[point_num].z = z;

        ipoint[point_num].x = kp1[2 * match[2 * i + 1]];
        ipoint[point_num].y = kp1[2 * match[2 * i + 1] + 1];
        point_num++;
    }
}

void RANSAC_PnP(OPOINT opoint[MAX_KEYPOINT_NUM],
	    		IPOINT ipoint[MAX_KEYPOINT_NUM],
				unsigned int point_num, FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy,
                volatile FLOAT *rmat, volatile FLOAT *tvec)
{
    int maxGoodCount = 0;
    int niters = MAXITER;
    RNG_state = (unsigned long long)-1;
    bool best_mask[MAX_KEYPOINT_NUM];

    unsigned int idx[MODEL_POINTS];
    generate_5_indices(point_num, idx);

    OPOINT subopoint[MODEL_POINTS];
    IPOINT subipoint[MODEL_POINTS];
    for(int i=0; i<MODEL_POINTS; i++)
    {
    	subopoint[i] = opoint[idx[i]];
    	subipoint[i] = ipoint[idx[i]];
    }

    epnp(subopoint, subipoint, fx, fy, cx, cy, rmat, tvec);

}

void generate_5_indices(unsigned int point_num, unsigned int idx[MODEL_POINTS])
{
#pragma HLS ARRAY_PARTITION variable=idx type=complete

	for(int i=0; i<MODEL_POINTS; i++)
	{
#pragma HLS UNROLL
		idx[i] = 0;
	}

    for (int i = 0; i < MODEL_POINTS; i++)
    {
        generate:
		unsigned int random_num = RNG(point_num);

		for(int j=0; j<MODEL_POINTS; j++)
		{
#pragma HLS UNROLL
			if(idx[j]==random_num)
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






/*
void EstimateMotion::estimate(volatile int32_t *match_val, unsigned int match_m,
                              volatile FLOAT *kp0_val, unsigned int kp0_m,
                              volatile FLOAT *kp1_val, unsigned int kp1_m,
                              volatile FLOAT *k, volatile FLOAT *depth)
{
    fx = k[0];
    fy = k[4];
    cx = k[2];
    cy = k[5];

    opoint = Matrix<FLOAT, MAX_KEYPOINT_NUM, 3>();
    ipoint = Matrix<FLOAT, MAX_KEYPOINT_NUM, 2>();
    int32_t j = 0;
    for (int32_t i = 0; i < match_m; i++)
    {
        FLOAT u = kp0_val[2 * match_val[2 * i]];        // kp0.val[match.val[i][0]][0];
        FLOAT v = kp0_val[2 * match_val[2 * i] + 1];    // kp0.val[match.val[i][0]][1];
        FLOAT z = depth[IMAGE_WIDTH * (int)v + (int)u]; // depth.val[(int)v][(int)u];
        if (z > MAX_DEPTH)
            continue;

        opoint.val[j][0] = z * (u - cx) / fx;
        opoint.val[j][1] = z * (v - cy) / fy;
        opoint.val[j][2] = z;

        ipoint.val[j][0] = kp1_val[2 * match_val[2 * i + 1]];     // kp1.val[match.val[i][1]][0];
        ipoint.val[j][1] = kp1_val[2 * match_val[2 * i + 1] + 1]; // kp1.val[match.val[i][1]][1];
        j++;
    }
    opoint.m = j; //.setRow(j);
    ipoint.m = j; //.setRow(j);
    RANSAC_PnP();
    return;
}

void EstimateMotion::RANSAC_PnP()
{
    int32_t maxGoodCount = 0;
    int32_t count = opoint.m;
    int32_t niters = MAXITER;
    RNG_state = (unsigned long long)-1;
    bool best_mask[MAX_KEYPOINT_NUM];

    getSubset();
    epnp(subopoint.val, subipoint.val, fx, fy, cx, cy, rmat.val, tvec.val);
    /*for (int32_t ii = 0; ii < niters; ii++)
    {
        getSubset();
        EPnP epnp = EPnP(subopoint.val, subipoint.val, fx, fy, cx, cy);
        epnp.compute();

        projPoint = projectPoint(epnp.rmat, epnp.tvec);

        bool mask[MAX_KEYPOINT_NUM];
        int32_t goodCount = 0;
        for (int32_t i = 0; i < count; i++)
        {
            FLOAT err = SQR(ipoint.val[i][0] - projPoint.val[i][0]) +
                        SQR(ipoint.val[i][1] - projPoint.val[i][1]);
            bool f = (err <= SQR(THRESHOLD));
            mask[i] = f;
            goodCount += f;
        }

        if (goodCount > MAX(maxGoodCount, 4))
        {
            memcpy(best_mask, mask, sizeof(bool) * count);
            maxGoodCount = goodCount;

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
    opoint_inlier.m = j;
    ipoint_inlier.m = j;
    PnPIterative pnp_iter = PnPIterative(opoint_inlier, ipoint_inlier, fx, fy, cx, cy);
    pnp_iter.compute(rmat, tvec);
    return;
}

void EstimateMotion::getSubset()
{
    // subopoint = Matrix<FLOAT, MODEL_POINTS, 3>();
    // subipoint = Matrix<FLOAT, MODEL_POINTS, 2>();

    unsigned int idx[MODEL_POINTS];
    for (int32_t i = 0; i < MODEL_POINTS; i++)
    {
        unsigned int idx_i;
        bool found = true;
        for (;;)
        {
            idx_i = RNG_uniform(opoint.m);
            for (int32_t j = 0; j < i; j++)
            {
                if (idx[j] == idx_i)
                {
                    found = false;
                    break;
                }
            }
            if (found)
                break;
        }

        idx[i] = idx_i;
        subopoint.val[i][0] = opoint.val[idx_i][0];
        subopoint.val[i][1] = opoint.val[idx_i][1];
        subopoint.val[i][2] = opoint.val[idx_i][2];
        subipoint.val[i][0] = ipoint.val[idx_i][0];
        subipoint.val[i][1] = ipoint.val[idx_i][1];
    }
    return;
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

unsigned int EstimateMotion::RNG_uniform(unsigned int a)
{
    if (a != 0)
    {
        RNG_state = (unsigned)RNG_state * RNG_COEF + (RNG_state >> 32);
        return (unsigned int)((unsigned int)RNG_state % a);
    }
    else
        return 0;
}*/
