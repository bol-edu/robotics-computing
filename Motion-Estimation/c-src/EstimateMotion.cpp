#include "EstimateMotion.h"

#define RNG_COEF 4164903690U
#define DBL_MAX 1.7976931348623158e+308
#define DBL_MIN 2.2250738585072014e-308
#define SIGN(a, b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define SWAP(a, b) \
    {              \
        temp = a;  \
        a = b;     \
        b = temp;  \
    }
static FLOAT sqrarg;
#define SQR(a) ((sqrarg = (a)) == 0.0 ? 0.0 : sqrarg * sqrarg)
#define MAX(a, b) (a > b ? a : b)
#define MIN(a, b) (a < b ? a : b)

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

int32_t ROUND(FLOAT x)
{
    return (int32_t)(x > 0 ? x + 0.5 : x - 0.5);
}

FLOAT POW(FLOAT a, int32_t n)
{
    FLOAT b = 1.0;
    for (int i = 0; i < n; i++)
        b *= a;
    return b;
}

EstimateMotion::EstimateMotion()
{
}

void EstimateMotion::estimate(Matrix &match, Matrix &kp0, Matrix &kp1,
                              Matrix &k, Matrix &depth,
                              Matrix &rmat, Matrix &tvec)
{
    max_depth = 3000;
    model_points = 5;
    RNG_state = (uint32_t)-1;
    confidence = 0.99;
    threshold = 8;
    maxIter = 1000;

    // 2D -> 3D projection
    Matrix opoint(match.m, 3);
    Matrix ipoint(match.m, 2);

    cx = k.val[0][2];
    cy = k.val[1][2];
    fx = k.val[0][0];
    fy = k.val[1][1];

    k.releaseMemory();

    int j = 0;
    for (int i = 0; i < match.m; i++)
    {
        FLOAT u = kp0.val[(int)(match.val[i][0])][0];
        FLOAT v = kp0.val[(int)(match.val[i][0])][1];
        FLOAT z = (FLOAT)depth.val[(int)v][(int)u];
        if (z > max_depth)
            continue;

        opoint.val[j][0] = z * (u - cx) / fx;
        opoint.val[j][1] = z * (v - cy) / fy;
        opoint.val[j][2] = z;

        ipoint.val[j][0] = kp1.val[(int)(match.val[i][1])][0];
        ipoint.val[j][1] = kp1.val[(int)(match.val[i][1])][1];
        j++;
    }
    opoint.m = j;
    ipoint.m = j;
    RANSAC_EPnP(opoint, ipoint, rmat, tvec);
}

void EstimateMotion::RANSAC_EPnP(Matrix opoint, Matrix ipoint, Matrix &_rmat, Matrix &_tvec)
{
    int32_t maxGoodCount = 0;
    int32_t count = opoint.m;
    int32_t niters = maxIter;
    RNG_state = (uint64_t)-1;
    bool *best_mask = new bool[count];
    for (int32_t ii = 0; ii < niters; ii++)
    {
        Matrix subopoint, subipoint;
        getSubset(opoint, ipoint, subopoint, subipoint);

        Matrix rmat, tvec;
        EPnP epnp = EPnP(subopoint, subipoint, fx, fy, cx, cy);
        epnp.compute(rmat, tvec);

        Matrix projPoint = projectPoint(opoint, rmat, tvec);
        Matrix err = Matrix(count, 1);
        for (int i = 0; i < count; i++)
        {
            err.val[i][0] = SQR(ipoint.val[i][0] - projPoint.val[i][0]) +
                            SQR(ipoint.val[i][1] - projPoint.val[i][1]);
        }

        bool *mask = new bool[count];
        int32_t goodCount = 0;
        for (int32_t i = 0; i < count; i++)
        {
            int32_t f = err.val[i][0] <= SQR(threshold);
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

                FLOAT denom = 1. - POW(1. - ep, 5);
                if (denom < DBL_MIN)
                    niters = 0;

                FLOAT num = std::log(1 - confidence);
                denom = std::log(denom);

                niters = denom >= 0 || -num >= niters * (-denom) ? niters : ROUND(num / denom);
            }
        }
    }

    Matrix opoint_inlier = Matrix(maxGoodCount, 3);
    Matrix ipoint_inlier = Matrix(maxGoodCount, 2);
    for (int32_t i = 0, j = 0; i < count; i++)
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

    PnPIterative pnp_iter(opoint_inlier, ipoint_inlier, fx, fy, cx, cy);
    pnp_iter.compute(_rmat, _tvec);
}

Matrix EstimateMotion::projectPoint(Matrix opoints, Matrix rmat, Matrix tvec)
{
    Matrix projPoint = Matrix(opoints.m, 2);
    Matrix opoint = Matrix(3, 1);
    Matrix rotpoint;
    for (int i = 0; i < opoints.m; i++)
    {
        opoint.val[0][0] = opoints.val[i][0];
        opoint.val[1][0] = opoints.val[i][1];
        opoint.val[2][0] = opoints.val[i][2];
        rotpoint = rmat * opoint + tvec;

        projPoint.val[i][0] = rotpoint.val[0][0] * fx / rotpoint.val[2][0] + cx;
        projPoint.val[i][1] = rotpoint.val[1][0] * fy / rotpoint.val[2][0] + cy;
    }
    return projPoint;
}

void EstimateMotion::getSubset(Matrix opoint, Matrix ipoint, Matrix &subopoint, Matrix &subipoint)
{
    subopoint = Matrix(model_points, 3);
    subipoint = Matrix(model_points, 2);

    int32_t *idx = new int32_t[model_points];
    for (int i = 0; i < model_points; i++)
    {
        uint32_t idx_i;
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

int64_t EstimateMotion::RNG_uniform(int64_t a, int64_t b)
{
    return a == b ? a : (int64_t)(RNG_next() % (b - a) + a);
}

unsigned EstimateMotion::RNG_next()
{
    RNG_state = (uint64_t)(unsigned)RNG_state * RNG_COEF + (unsigned)(RNG_state >> 32);
    return (unsigned)RNG_state;
}

EstimateMotion::~EstimateMotion()
{
}