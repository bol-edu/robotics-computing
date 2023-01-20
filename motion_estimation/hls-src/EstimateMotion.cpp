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
    Matrix opoint(match.m, 3);
    Matrix ipoint(match.m, 2);

    cx = k.val[0][2];
    cy = k.val[1][2];
    fx = k.val[0][0];
    fy = k.val[1][1];
    int j = 0;
    for (int i = 0; i < match.m; i++)
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
    opoint.m = j;
    ipoint.m = j;

    RANSAC_EPnP(opoint, ipoint, k);
}

void EstimateMotion::RANSAC_EPnP(Matrix opoint, Matrix ipoint, Matrix k)
{
    Matrix subopoint, subipoint;
    getSubset(opoint, ipoint, subopoint, subipoint);

    Matrix rmat, tvec;
    EPnP(subopoint, subipoint, k, rmat, tvec);
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
            idx_i %= opoint.m;
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

void EstimateMotion::EPnP(Matrix &pws, Matrix &us, Matrix &k, Matrix &rmat, Matrix &tvec)
{
    int32_t number_of_correspondences = pws.m;

    // control points in world-coord;
    Matrix cws = Matrix(4, 3);
    // Take C0 as the reference points centroid:
    cws.val[0][0] = cws.val[0][1] = cws.val[0][2] = 0;
    for (int i = 0; i < number_of_correspondences; i++)
        for (int j = 0; j < 3; j++)
            cws.val[0][j] += pws.val[i][j];

    for (int j = 0; j < 3; j++)
        cws.val[0][j] /= number_of_correspondences;

    // Take C1, C2, and C3 from PCA on the reference points:
    Matrix PW0 = Matrix(number_of_correspondences, 3);
    for (int i = 0; i < number_of_correspondences; i++)
        for (int j = 0; j < 3; j++)
            PW0.val[i][j] = pws.val[i][j] - cws.val[0][j];

    Matrix PW0tPW0 = PW0.multrans();
    Matrix DC;  // = Matrix(3, 1);
    Matrix UCt; // = Matrix(3, 3);
    Matrix V;
    PW0tPW0.svd(UCt, DC, V);

    cout << DC << endl;
    cout << UCt << endl;
    // cvMulTransposed(PW0, &PW0tPW0, 1); // PW0tPW0 = PW0t . PW0
    // cvSVD(&PW0tPW0, &DC, &UCt, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);
    //  cvd(MtM, λ, ν, ...)
}

EstimateMotion::~EstimateMotion()
{
}