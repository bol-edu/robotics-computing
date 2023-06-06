#include "Math.h"
#include "sym_svd.hpp"

#define NUM_OF_PARAMS 6
#define NUM_OF_ERRS (2 * MAX_KEYPOINT_NUM)

class LevMarq
{
public:
    LevMarq(int _nerrs)
    {
        nerrs = _nerrs;
        errNorm = prevErrNorm = FLT_MAX;
        lambdaLg10 = -3;
        max_iter = 20;
        epsilon = FLT_EPSILON;
        state = STARTED;
        iters = 0;
    }

    bool update(FLOAT _param[NUM_OF_PARAMS],
                FLOAT matJ[NUM_OF_ERRS][NUM_OF_PARAMS])
    {
        is_matJ = false;
        is_err = false;

        if (state == DONE)
        {
            for (int i = 0; i < NUM_OF_PARAMS; i++)
            {
                _param[i] = param[i];
            }
            return false;
        }

        if (state == STARTED)
        {
            for (int i = 0; i < NUM_OF_PARAMS; i++)
            {
                _param[i] = param[i];
            }

            for (int i = 0; i < nerrs; i++)
            {
                err[i] = 0.f;
                for (int j = 0; j < NUM_OF_PARAMS; j++)
                {
                    J[i][j] = 0.f;
                    matJ[i][j] = 0.f;
                }
            }

            is_matJ = true;
            is_err = true;
            state = CALC_J;
            return true;
        }

        if (state == CALC_J)
        {
            for (int i = 0; i < NUM_OF_PARAMS; i++)
            {
                for (int j = 0; j < NUM_OF_PARAMS; j++)
                {
                    FLOAT p = 0.f;
                    for (int k = 0; k < nerrs; k++)
                    {
                        p += J[k][i] * J[k][j];
                    }
                    JtJ[i][j] = p;
                }
            }

            for (int i = 0; i < NUM_OF_PARAMS; i++)
            {
                FLOAT sum = 0.f;
                for (int j = 0; j < nerrs; j++)
                {
                    sum += J[j][i] * err[j];
                }
                JtErr[i] = sum;
            }

            for (int i = 0; i < NUM_OF_PARAMS; i++)
            {
                prevParam[i] = param[i];
            }

            step();

            if (iters == 0)
            {
                prevErrNorm = norm(err, nerrs);
            }

            for (int i = 0; i < NUM_OF_PARAMS; i++)
            {
                _param[i] = param[i];
            }

            for (int i = 0; i < nerrs; i++)
            {
                err[i] = 0.f;
            }

            is_err = true;
            state = CHECK_ERR;
            return true;
        }

        errNorm = norm(err, nerrs);
        if (errNorm > prevErrNorm)
        {
            if (++lambdaLg10 <= 16)
            {
                step();
                for (int i = 0; i < NUM_OF_PARAMS; i++)
                {
                    _param[i] = param[i];
                }

                for (int i = 0; i < nerrs; i++)
                {
                    err[i] = 0.f;
                }

                is_err = true;
                state = CHECK_ERR;
                return true;
            }
        }

        lambdaLg10 = hls::max(lambdaLg10 - 1, -16);
        FLOAT diff[NUM_OF_PARAMS];
        for (int i = 0; i < NUM_OF_PARAMS; i++)
        {
            diff[i] = param[i] - prevParam[i];
        }
        if (++iters >= max_iter || (norm(diff, NUM_OF_PARAMS) / norm(prevParam, NUM_OF_PARAMS) < epsilon))
        {
            for (int i = 0; i < NUM_OF_PARAMS; i++)
            {
                _param[i] = param[i];
            }
            state = DONE;
            return true;
        }

        prevErrNorm = errNorm;
        for (int i = 0; i < NUM_OF_PARAMS; i++)
        {
            _param[i] = param[i];
        }

        for (int i = 0; i < nerrs; i++)
        {
            for (int j = 0; j < NUM_OF_PARAMS; j++)
            {
                J[i][j] = 0.f;
                matJ[i][j] = 0.f;
            }
        }

        is_matJ = true;
        is_err = true;
        state = CALC_J;
        return true;
    }

    void step()
    {
        const FLOAT LOG10 = hls::log(10.f);
        FLOAT lambda = hls::exp(lambdaLg10 * LOG10);

        for (int i = 0; i < NUM_OF_PARAMS; i++)
        {
            for (int j = 0; j < NUM_OF_PARAMS; j++)
            {
                JtJN[i * NUM_OF_PARAMS + j] = JtJ[i][j];
            }
        }

        for (int i = 0; i < NUM_OF_PARAMS; i++)
        {
            JtJN[i * NUM_OF_PARAMS + i] *= 1. + lambda;
        }

        solve(JtJN, JtJW, JtErr, 6, 6);

        for (int i = 0; i < NUM_OF_PARAMS; i++)
            param[i] = prevParam[i] - JtJW[i];
    }

    enum
    {
        DONE = 0,
        STARTED = 1,
        CALC_J = 2,
        CHECK_ERR = 3
    };

    FLOAT prevParam[NUM_OF_PARAMS];
    FLOAT param[NUM_OF_PARAMS];
    FLOAT J[NUM_OF_ERRS][NUM_OF_PARAMS];
    FLOAT err[NUM_OF_ERRS];
    FLOAT JtJ[NUM_OF_PARAMS][NUM_OF_PARAMS];
    FLOAT JtJN[NUM_OF_PARAMS * NUM_OF_PARAMS];
    FLOAT JtErr[NUM_OF_PARAMS];
    FLOAT JtJV[NUM_OF_PARAMS];
    FLOAT JtJW[NUM_OF_PARAMS];
    FLOAT prevErrNorm, errNorm;
    FLOAT epsilon;
    int nerrs;
    int lambdaLg10;
    int max_iter;
    int state;
    int iters;
    bool is_matJ;
    bool is_err;
};

void Rodrigues_9_to_3(FLOAT src[9], FLOAT dst[3])
{
    FLOAT rx = src[7] - src[5];
    FLOAT ry = src[2] - src[6];
    FLOAT rz = src[3] - src[1];

    FLOAT s = hls::sqrt((rx * rx + ry * ry + rz * rz) * 0.25);
    FLOAT c = (src[0] + src[4] + src[8] - 1) * 0.5;
    c = c > 1. ? 1. : c < 0.54f ? 0.54f
                                : c;
    FLOAT theta = hls::acos(c);
    theta = (theta == 0) ? FLT_EPSILON : theta;

    FLOAT vth = 1 / (2 * s);

    vth *= theta;

    dst[0] = rx * vth;
    dst[1] = ry * vth;
    dst[2] = rz * vth;
}

void Rodrigues_3_to_9(FLOAT src[3], FLOAT dst[9])
{
    FLOAT rx = src[0];
    FLOAT ry = src[1];
    FLOAT rz = src[2];

    FLOAT theta = hls::sqrt(rx * rx + ry * ry + rz * rz);
    FLOAT c = hls::cos(theta);
    FLOAT s = hls::sin(theta);
    FLOAT c1 = 1 - c;
    FLOAT itheta = theta ? 1. / theta : 0.;

    rx *= itheta;
    ry *= itheta;
    rz *= itheta;

    FLOAT rrt[9];
#pragma HLS ARRAY_PARTITION variable = rrt type = complete

    for (int i = 0; i < 3; i++)
    {
        for (int j = i; j < 3; j++)
        {
            rrt[i * 3 + j] = src[i] * src[j];
            if (i != j)
                rrt[j * 3 + i] = src[i] * src[j];
        }
    }

    FLOAT r_x[9];
    r_x[0] = 0;
    r_x[1] = -rz;
    r_x[2] = ry;
    r_x[3] = rz;
    r_x[4] = 0;
    r_x[5] = -rx;
    r_x[6] = -ry;
    r_x[7] = rx;
    r_x[8] = 0;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (i == j)
                dst[i * 3 + j] = c + c1 * rrt[i * 3 + j] + r_x[i * 3 + j] * s;
            else
                dst[i * 3 + j] = c1 * rrt[i * 3 + j] + r_x[i * 3 + j] * s;
        }
    }
}

// jacob[3][9]
void Rodrigues(FLOAT src[3], FLOAT jacob[27], FLOAT dst[9])
{
    FLOAT rx = src[0];
    FLOAT ry = src[1];
    FLOAT rz = src[2];

    FLOAT theta = hls::sqrt(rx * rx + ry * ry + rz * rz);
    FLOAT c = hls::cos(theta);
    FLOAT s = hls::sin(theta);
    FLOAT c1 = 1 - c;
    FLOAT itheta = theta ? 1. / theta : 0.;

    rx *= itheta;
    ry *= itheta;
    rz *= itheta;

    FLOAT rrt[9];
    for (int i = 0; i < 3; i++)
    {
        for (int j = i; j < 3; j++)
        {
            rrt[i * 3 + j] = src[i] * src[j];
            if (i != j)
                rrt[j * 3 + i] = src[i] * src[j];
        }
    }

    FLOAT r_x[9];
    r_x[0] = 0;
    r_x[1] = -rz;
    r_x[2] = ry;
    r_x[3] = rz;
    r_x[4] = 0;
    r_x[5] = -rx;
    r_x[6] = -ry;
    r_x[7] = rx;
    r_x[8] = 0;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (i == j)
                dst[i * 3 + j] = c + c1 * rrt[i * 3 + j] + r_x[i * 3 + j] * s;
            else
                dst[i * 3 + j] = c1 * rrt[i * 3 + j] + r_x[i * 3 + j] * s;
        }
    }

    const FLOAT I[9] = {1, 0, 0,
                        0, 1, 0,
                        0, 0, 1};
    FLOAT drrt[27] = {rx + rx, ry, rz, ry, 0, 0, rz, 0, 0,
                      0, rx, 0, rx, ry + ry, rz, 0, rz, 0,
                      0, 0, rx, 0, 0, ry, rx, ry, rz + rz};
    FLOAT d_r_x_[27] = {0, 0, 0, 0, 0, -1, 0, 1, 0,
                        0, 0, 1, 0, 0, 0, -1, 0, 0,
                        0, -1, 0, 1, 0, 0, 0, 0, 0};

    for (int i = 0; i < 3; i++)
    {
        FLOAT ri = src[i];
        FLOAT a0 = -s * ri;
        FLOAT a1 = (s - 2 * c1 * itheta) * ri;
        FLOAT a2 = c1 * itheta;
        FLOAT a3 = (c - s * itheta) * ri;
        FLOAT a4 = s * itheta;
        for (int k = 0; k < 9; k++)
        {
            jacob[i * 9 + k] = a0 * I[k] + a1 * rrt[k] + a2 * drrt[i * 9 + k] +
                               a3 * r_x[k] + a4 * d_r_x_[i * 9 + k];
        }
    }
}

void ProjectPoints(OPOINT objectPoints[MAX_KEYPOINT_NUM],
                   int count, FLOAT r_vec[3], FLOAT t_vec[3],
                   FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy,
                   IPOINT imagePoints[MAX_KEYPOINT_NUM])
{
    FLOAT R[9];
    Rodrigues_3_to_9(r_vec, R);

    for (int i = 0; i < count; i++)
    {
        FLOAT X = objectPoints[i].x, Y = objectPoints[i].y, Z = objectPoints[i].z;
        FLOAT x = R[0] * X + R[1] * Y + R[2] * Z + t_vec[0];
        FLOAT y = R[3] * X + R[4] * Y + R[5] * Z + t_vec[1];
        FLOAT z = R[6] * X + R[7] * Y + R[8] * Z + t_vec[2];
        z = z ? 1. / z : 1;
        x *= z;
        y *= z;

        imagePoints[i].x = x * fx + cx;
        imagePoints[i].y = y * fy + cy;
    }
}

void ProjectPoints(OPOINT objectPoints[MAX_KEYPOINT_NUM],
                   int count, FLOAT r_vec[3], FLOAT t_vec[3],
                   FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy,
                   IPOINT imagePoints[MAX_KEYPOINT_NUM],
                   FLOAT dpdr[NUM_OF_ERRS][3], FLOAT dpdt[NUM_OF_ERRS][3])
{
    FLOAT dRdr[27];
    FLOAT R[9];
    Rodrigues(r_vec, dRdr, R);

    for (int i = 0; i < count; i++)
    {
        FLOAT X = objectPoints[i].x, Y = objectPoints[i].y, Z = objectPoints[i].z;
        FLOAT x = R[0] * X + R[1] * Y + R[2] * Z + t_vec[0];
        FLOAT y = R[3] * X + R[4] * Y + R[5] * Z + t_vec[1];
        FLOAT z = R[6] * X + R[7] * Y + R[8] * Z + t_vec[2];
        z = z ? 1. / z : 1;
        x *= z;
        y *= z;

        imagePoints[i].x = x * fx + cx;
        imagePoints[i].y = y * fy + cy;

        FLOAT dxdt[3] = {z, 0, -x * z};
        FLOAT dydt[3] = {0, z, -y * z};
        for (int j = 0; j < 3; j++)
        {
            dpdt[2 * i][j] = fx * dxdt[j];
            dpdt[2 * i + 1][j] = fy * dydt[j];
        }

        FLOAT dx0dr[3] =
            {X * dRdr[0] + Y * dRdr[1] + Z * dRdr[2],
             X * dRdr[9] + Y * dRdr[10] + Z * dRdr[11],
             X * dRdr[18] + Y * dRdr[19] + Z * dRdr[20]};
        FLOAT dy0dr[3] =
            {X * dRdr[3] + Y * dRdr[4] + Z * dRdr[5],
             X * dRdr[12] + Y * dRdr[13] + Z * dRdr[14],
             X * dRdr[21] + Y * dRdr[22] + Z * dRdr[23]};
        FLOAT dz0dr[3] =
            {X * dRdr[6] + Y * dRdr[7] + Z * dRdr[8],
             X * dRdr[15] + Y * dRdr[16] + Z * dRdr[17],
             X * dRdr[24] + Y * dRdr[25] + Z * dRdr[26]};

        for (int j = 0; j < 3; j++)
        {
            FLOAT dxdr = z * (dx0dr[j] - x * dz0dr[j]);
            FLOAT dydr = z * (dy0dr[j] - y * dz0dr[j]);
            dpdr[2 * i][j] = fx * dxdr;
            dpdr[2 * i + 1][j] = fy * dydr;
        }
    }
}

int decide_R_t(OPOINT opoints[MAX_KEYPOINT_NUM], IPOINT ipoints[MAX_KEYPOINT_NUM],
               int point_num, int fx, int fy, int cx, int cy,
               FLOAT rmat[9], FLOAT tvec[3],
               FLOAT epnp_rmat[9], FLOAT epnp_tvec[3],
               FLOAT rmat_former[9], FLOAT tvec_former[3], bool take_last)
{
    FLOAT err = 0, epnp_err = 0, former_err = 0;
    for (int i = 0; i < point_num; i++)
    {
        OPOINT opoint = opoints[i];
        IPOINT ipoint = ipoints[i];

        OPOINT rotpoint;
        IPOINT projPoint;

        rotpoint.x = rmat[0] * opoint.x + rmat[1] * opoint.y + rmat[2] * opoint.z + tvec[0];
        rotpoint.y = rmat[3] * opoint.x + rmat[4] * opoint.y + rmat[5] * opoint.z + tvec[1];
        rotpoint.z = rmat[6] * opoint.x + rmat[7] * opoint.y + rmat[8] * opoint.z + tvec[2];

        projPoint.x = rotpoint.x * fx / rotpoint.z + cx;
        projPoint.y = rotpoint.y * fy / rotpoint.z + cy;

        err += hls::pow(projPoint.x - ipoint.x, 2.f) + hls::pow(projPoint.y - ipoint.y, 2.f);

        OPOINT rotpoint_epnp;
        IPOINT projPoint_epnp;

        rotpoint_epnp.x = epnp_rmat[0] * opoint.x + epnp_rmat[1] * opoint.y + epnp_rmat[2] * opoint.z + epnp_tvec[0];
        rotpoint_epnp.y = epnp_rmat[3] * opoint.x + epnp_rmat[4] * opoint.y + epnp_rmat[5] * opoint.z + epnp_tvec[1];
        rotpoint_epnp.z = epnp_rmat[6] * opoint.x + epnp_rmat[7] * opoint.y + epnp_rmat[8] * opoint.z + epnp_tvec[2];

        projPoint_epnp.x = rotpoint_epnp.x * fx / rotpoint_epnp.z + cx;
        projPoint_epnp.y = rotpoint_epnp.y * fy / rotpoint_epnp.z + cy;

        epnp_err += hls::pow(projPoint_epnp.x - ipoint.x, 2.f) + hls::pow(projPoint_epnp.y - ipoint.y, 2.f);

        OPOINT rotpoint_former;
        IPOINT projPoint_former;

        rotpoint_former.x = rmat_former[0] * opoint.x + rmat_former[1] * opoint.y + rmat_former[2] * opoint.z + tvec_former[0];
        rotpoint_former.y = rmat_former[3] * opoint.x + rmat_former[4] * opoint.y + rmat_former[5] * opoint.z + tvec_former[1];
        rotpoint_former.z = rmat_former[6] * opoint.x + rmat_former[7] * opoint.y + rmat_former[8] * opoint.z + tvec_former[2];

        projPoint_former.x = rotpoint_former.x * fx / rotpoint_former.z + cx;
        projPoint_former.y = rotpoint_former.y * fy / rotpoint_former.z + cy;

        former_err += hls::pow(projPoint_former.x - ipoint.x, 2.f) + hls::pow(projPoint_former.y - ipoint.y, 2.f);
    }

    if (former_err < err && former_err < epnp_err && take_last)
        return 2;
    else if (epnp_err < err && epnp_err < former_err)
        return 1;
    else
        return 0;
}

void pnp_iterative(OPOINT opoint[MAX_KEYPOINT_NUM],
                   IPOINT ipoint[MAX_KEYPOINT_NUM],
                   int number_of_points, FLOAT epnp_rmat[9], FLOAT epnp_tvec[3],
                   FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy,
                   FLOAT rmat[9], FLOAT tvec[3], bool take_last)
{
    IPOINT mn[MAX_KEYPOINT_NUM];
    for (int i = 0; i < number_of_points; i++)
    {
        FLOAT x, y;
        x = ipoint[i].x;
        y = ipoint[i].y;

        mn[i].x = (x - cx) * (1. / fx);
        mn[i].y = (y - cy) * (1. / fy);
    }

    // non-planar structure. Use DLT method
    FLOAT matL[2 * MAX_KEYPOINT_NUM * 12];
#pragma HLS ARRAY_PARTITION variable = matL type = cyclic factor = 24
    for (int i = 0; i < number_of_points; i++)
    {
        FLOAT x = -mn[i].x, y = -mn[i].y;
        matL[24 * i] = matL[24 * i + 16] = opoint[i].x;
        matL[24 * i + 1] = matL[24 * i + 17] = opoint[i].y;
        matL[24 * i + 2] = matL[24 * i + 18] = opoint[i].z;
        matL[24 * i + 3] = matL[24 * i + 19] = 1.f;
        matL[24 * i + 4] = matL[24 * i + 5] = matL[24 * i + 6] = matL[24 * i + 7] = 0.f;
        matL[24 * i + 12] = matL[24 * i + 13] = matL[24 * i + 14] = matL[24 * i + 15] = 0.f;
        matL[24 * i + 8] = x * opoint[i].x;
        matL[24 * i + 9] = x * opoint[i].y;
        matL[24 * i + 10] = x * opoint[i].z;
        matL[24 * i + 11] = x;
        matL[24 * i + 20] = y * opoint[i].x;
        matL[24 * i + 21] = y * opoint[i].y;
        matL[24 * i + 22] = y * opoint[i].z;
        matL[24 * i + 23] = y;
    }

    FLOAT LL[12 * 12];
    multrans(matL, LL, 2 * number_of_points, 12);

    FLOAT LD[12];
    FLOAT LU[12 * 12];
    FLOAT LV[12 * 12];
    xf::solver::svd(LL, LD, LU, LV, 12);

    FLOAT RR[9];
    FLOAT tt[3];
    RR[0] = LV[11];
    RR[1] = LV[23];
    RR[2] = LV[35];
    RR[3] = LV[59];
    RR[4] = LV[71];
    RR[5] = LV[83];
    RR[6] = LV[107];
    RR[7] = LV[119];
    RR[8] = LV[131];
    tt[0] = LV[47];
    tt[1] = LV[95];
    tt[2] = LV[143];

    FLOAT det = RR[0] * RR[4] * RR[8] +
                RR[1] * RR[5] * RR[6] +
                RR[2] * RR[3] * RR[7] -
                RR[2] * RR[4] * RR[6] -
                RR[1] * RR[3] * RR[8] -
                RR[0] * RR[5] * RR[7];

    if (det < 0.f)
    {
        for (int i = 0; i < 9; i++)
        {
            RR[i] *= -1.f;
        }
        for (int i = 0; i < 3; i++)
        {
            tt[i] *= -1.f;
        }
    }

    FLOAT sc = norm(RR, 9);
    FLOAT matW[3];
    FLOAT matU[9];
    FLOAT matV[9];
    xf::solver::svd(RR, matW, matU, matV, 3);

    FLOAT matR[9];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            matR[i * 3 + j] = matU[i * 3] * matV[j * 3] +
                              matU[i * 3 + 1] * matV[j * 3 + 1] +
                              matU[i * 3 + 2] * matV[j * 3 + 2];
        }
    }

    FLOAT matR_norm = norm(matR, 9) / sc;
    FLOAT t[3];
    for (int i = 0; i < 3; i++)
    {
        t[i] = tt[i] * matR_norm;
    }

    int choice = decide_R_t(opoint, ipoint, number_of_points, fx, fy, cx, cy,
                            matR, t, epnp_rmat, epnp_tvec, rmat, tvec, take_last);

    switch (choice)
    {
    case 1:
        for (int i = 0; i < 9; i++)
            matR[i] = epnp_rmat[i];
        for (int i = 0; i < 3; i++)
            t[i] = epnp_tvec[i];
        break;
    case 2:
        for (int i = 0; i < 9; i++)
            matR[i] = rmat[i];
        for (int i = 0; i < 3; i++)
            t[i] = tvec[i];
        break;
    }

    FLOAT r[3];
    Rodrigues_9_to_3(matR, r);

    // refine extrinsic parameters using iterative algorithm
    LevMarq solver = LevMarq(2 * number_of_points);
    solver.param[0] = r[0];
    solver.param[1] = r[1];
    solver.param[2] = r[2];
    solver.param[3] = t[0];
    solver.param[4] = t[1];
    solver.param[5] = t[2];

    FLOAT matJ[NUM_OF_ERRS][NUM_OF_PARAMS];
    IPOINT err[MAX_KEYPOINT_NUM];
    FLOAT param[NUM_OF_PARAMS];
    FLOAT dpdr[NUM_OF_ERRS][3];
    FLOAT dpdt[NUM_OF_ERRS][3];

    for (;;)
    {
        bool proceed = solver.update(param, matJ);

        for (int i = 0; i < NUM_OF_PARAMS; i++)
        {
            solver.param[i] = param[i];
        }

        for (int i = 0; i < 3; i++)
        {
            r[i] = param[i];
            t[i] = param[i + 3];
        }

        if (!proceed || !solver.is_err)
            break;
        if (solver.is_matJ)
        {
            for (int i = 0; i < solver.nerrs; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    dpdr[i][j] = matJ[i][j];
                    dpdt[i][j] = matJ[i][j + 3];
                }
            }

            ProjectPoints(opoint, number_of_points, r, t,
                          fx, fy, cx, cy, err, dpdr, dpdt);

            for (int i = 0; i < solver.nerrs; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    solver.J[i][j] = dpdr[i][j];
                    solver.J[i][j + 3] = dpdt[i][j];
                }
            }
        }
        else
        {
            ProjectPoints(opoint, number_of_points, r, t,
                          fx, fy, cx, cy, err);
        }

        for (int i = 0; i < number_of_points; i++)
        {
            solver.err[2 * i] = err[i].x - ipoint[i].x;
            solver.err[2 * i + 1] = err[i].y - ipoint[i].y;
        }
    }

    Rodrigues_3_to_9(r, rmat);
    for (int i = 0; i < 3; i++)
    {
        tvec[i] = t[i];
    }
}
