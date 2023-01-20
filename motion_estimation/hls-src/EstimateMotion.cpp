#include "EstimateMotion.h"

EstimateMotion::EstimateMotion()
{
    max_depth = 3000;
    model_points = 5;
    RNG_COEF = 4164903690;
    RNG_state = (uint32_t)-1;
    confidence;
    threshold;
    maxIter;
}

void EstimateMotion::estimate(Matrix &match, Matrix &kp0, Matrix &kp1,
                              Matrix &k, Matrix &depth,
                              Matrix &rmat, Matrix &tvec)
{
    // 2D -> 3D projection
    Matrix opoint(match.r, 3);
    Matrix ipoint(match.r, 2);

    FLOAT cx = k.val[0][2];
    FLOAT cy = k.val[1][2];
    FLOAT fx = k.val[0][0];
    FLOAT fy = k.val[1][1];
    int j = 0;
    for (int i = 0; i < match.r; i++)
    {
        FLOAT u = kp0.val[(int)(match.val[i][0])][0];
        FLOAT v = kp0.val[(int)(match.val[i][0])][1];
        FLOAT z = depth.val[(int)v][(int)u];
        if (z > max_depth)
            continue;

        opoint.val[j][0] = z * (u - cx) / fx;
        opoint.val[j][1] = z * (v - cy) / fy;
        opoint.val[j][2] = z;

        ipoint.val[j][0] = kp1.val[(int)(match.val[i][1])][0];
        ipoint.val[j][1] = kp1.val[(int)(match.val[i][1])][1];
        j++;
    }
    opoint.r = j;
    ipoint.r = j;

    RANSAC_EPnP(opoint, ipoint, k);
}

void EstimateMotion::getSubset(Matrix &opoint, Matrix &ipoint, Matrix &subopoint, Matrix &subipoint)
{
    subopoint = Matrix(model_points, 3);
    subipoint = Matrix(model_points, 2);

    int32_t *idx = new int32_t[model_points];
    uint32_t idx_i = RNG_state;
    for (int i = 0; i < model_points; i++)
    {
        while (true)
        {
            idx_i %= opoint.r;
            int j;
            for (j = 0; j < i; j++)
            {
                if (idx_i == idx[j])
                    break;
            }
            if (j == i)
            {
                idx[i] = idx_i;
                idx_i = idx_i * RNG_COEF + (uint32_t)(idx_i >> 16);
                break;
            }
            idx_i = idx_i * RNG_COEF + (uint32_t)(idx_i >> 16);
        }
    }

    RNG_state = idx_i;
    for (int i = 0; i < model_points; i++)
    {
        subopoint.val[i][0] = opoint.val[idx[i]][0];
        subopoint.val[i][1] = opoint.val[idx[i]][1];
        subopoint.val[i][2] = opoint.val[idx[i]][2];
        subipoint.val[i][0] = ipoint.val[idx[i]][0];
        subipoint.val[i][1] = ipoint.val[idx[i]][1];
    }
}

void EstimateMotion::RANSAC_EPnP(Matrix opoint, Matrix ipoint, Matrix k)
{
    Matrix subopoint, subipoint, subopoint1, subipoint1;
    getSubset(opoint, ipoint, subopoint, subipoint);
    getSubset(opoint, ipoint, subopoint1, subipoint1);
    cout << subipoint << endl
         << endl;
    cout << subipoint1;
}

EstimateMotion::~EstimateMotion()
{
}