#include "PnPIterative.h"

#define DBL_MAX 1.7976931348623158e+308
#define FLT_EPSILON 1.192092896e-07F // smallest such that 1.0+FLT_EPSILON != 1.0

Matrix<FLOAT, 3, 1> Rodrigues(Matrix<FLOAT, 3, 3> &src)
{
    FLOAT rx = src.val[2][1] - src.val[1][2];
    FLOAT ry = src.val[0][2] - src.val[2][0];
    FLOAT rz = src.val[1][0] - src.val[0][1];

    FLOAT s = sqrt((rx * rx + ry * ry + rz * rz) * 0.25);
    FLOAT c = (src.val[0][0] + src.val[1][1] + src.val[2][2] - 1) * 0.5;
    c = c > 1. ? 1. : c < -1. ? -1.
                              : c;
    FLOAT theta = acos(c);

    FLOAT vth = 1 / (2 * s);

    vth *= theta;

    Matrix<FLOAT, 3, 1> dst = Matrix<FLOAT, 3, 1>();
    dst.val[0][0] = rx * vth;
    dst.val[1][0] = ry * vth;
    dst.val[2][0] = rz * vth;
    return dst;
}

Matrix<FLOAT, 3, 3> Rodrigues(Matrix<FLOAT, 3, 1> &src)
{
    FLOAT rx = src.val[0][0];
    FLOAT ry = src.val[1][0];
    FLOAT rz = src.val[2][0];

    FLOAT theta = sqrt(rx * rx + ry * ry + rz * rz);
    FLOAT c = cos(theta);
    FLOAT s = sin(theta);
    FLOAT c1 = 1 - c;
    FLOAT itheta = theta ? 1. / theta : 0.;

    rx *= itheta;
    ry *= itheta;
    rz *= itheta;

    Matrix<FLOAT, 3, 3> rrt = src * src.trans();
    Matrix<FLOAT, 3, 3> r_x = Matrix<FLOAT, 3, 3>();
    r_x.val[0][1] = -rz;
    r_x.val[0][2] = ry;
    r_x.val[1][0] = rz;
    r_x.val[1][2] = -rx;
    r_x.val[2][0] = -ry;
    r_x.val[2][1] = rx;

    Matrix<FLOAT, 3, 3> dst = eye<FLOAT, 3>() * c + rrt * c1 + r_x * s;
    return dst;
}

Matrix<FLOAT, 3, 3> Rodrigues(Matrix<FLOAT, 3, 1> &src, Matrix<FLOAT, 3, 9> &jacob)
{
    FLOAT rx = src.val[0][0];
    FLOAT ry = src.val[1][0];
    FLOAT rz = src.val[2][0];

    FLOAT theta = sqrt(rx * rx + ry * ry + rz * rz);
    FLOAT c = cos(theta);
    FLOAT s = sin(theta);
    FLOAT c1 = 1 - c;
    FLOAT itheta = theta ? 1. / theta : 0.;

    rx *= itheta;
    ry *= itheta;
    rz *= itheta;

    Matrix<FLOAT, 3, 3> rrt = src * src.trans();
    Matrix<FLOAT, 3, 3> r_x = Matrix<FLOAT, 3, 3>();
    r_x.val[0][1] = -rz;
    r_x.val[0][2] = ry;
    r_x.val[1][0] = rz;
    r_x.val[1][2] = -rx;
    r_x.val[2][0] = -ry;
    r_x.val[2][1] = rx;

    Matrix<FLOAT, 3, 3> dst = eye<FLOAT, 3>() * c + rrt * c1 + r_x * s;

    const FLOAT I[] = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1};
    FLOAT drrt[] = {rx + rx, ry, rz, ry, 0, 0, rz, 0, 0,
                    0, rx, 0, rx, ry + ry, rz, 0, rz, 0,
                    0, 0, rx, 0, 0, ry, rx, ry, rz + rz};
    FLOAT d_r_x_[] = {0, 0, 0, 0, 0, -1, 0, 1, 0,
                      0, 0, 1, 0, 0, 0, -1, 0, 0,
                      0, -1, 0, 1, 0, 0, 0, 0, 0};
    jacob = Matrix<FLOAT, 3, 9>();
    for (int i = 0; i < 3; i++)
    {
        FLOAT ri = i == 0 ? rx : i == 1 ? ry
                                        : rz;
        FLOAT a0 = -s * ri, a1 = (s - 2 * c1 * itheta) * ri, a2 = c1 * itheta;
        FLOAT a3 = (c - s * itheta) * ri, a4 = s * itheta;
        for (int k = 0; k < 9; k++)
            jacob.val[i][k] = a0 * I[k] + a1 * rrt.val[k / 3][k % 3] + a2 * drrt[i * 9 + k] +
                              a3 * r_x.val[k / 3][k % 3] + a4 * d_r_x_[i * 9 + k];
    }
    return dst;
}

void ProjectPoints(Matrix<FLOAT, MAX_KEYPOINT_NUM, 3> &objectPoints,
                   Matrix<FLOAT, 3, 1> &r_vec,
                   Matrix<FLOAT, 3, 1> &t_vec,
                   FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy,
                   Matrix<FLOAT, MAX_KEYPOINT_NUM, 2> &imagePoints)
{
    int32_t count = objectPoints.m;

    Matrix<FLOAT, 3, 3> R = Rodrigues(r_vec);

    imagePoints = Matrix<FLOAT, MAX_KEYPOINT_NUM, 2>();
    for (int32_t i = 0; i < count; i++)
    {
        FLOAT X = objectPoints.val[i][0], Y = objectPoints.val[i][1], Z = objectPoints.val[i][2];
        FLOAT x = R.val[0][0] * X + R.val[0][1] * Y + R.val[0][2] * Z + t_vec.val[0][0];
        FLOAT y = R.val[1][0] * X + R.val[1][1] * Y + R.val[1][2] * Z + t_vec.val[1][0];
        FLOAT z = R.val[2][0] * X + R.val[2][1] * Y + R.val[2][2] * Z + t_vec.val[2][0];
        z = z ? 1. / z : 1;
        x *= z;
        y *= z;

        imagePoints.val[i][0] = x * fx + cx;
        imagePoints.val[i][1] = y * fy + cy;
    }
}

void ProjectPoints(Matrix<FLOAT, MAX_KEYPOINT_NUM, 3> &objectPoints,
                   Matrix<FLOAT, 3, 1> &r_vec,
                   Matrix<FLOAT, 3, 1> &t_vec,
                   FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy,
                   Matrix<FLOAT, MAX_KEYPOINT_NUM, 2> &imagePoints,
                   Matrix<FLOAT, NUM_OF_ERRS, 3> &dpdr,
                   Matrix<FLOAT, NUM_OF_ERRS, 3> &dpdt)
{
    int32_t count = objectPoints.m;

    Matrix<FLOAT, 3, 1> rr = r_vec;
    Matrix<FLOAT, 3, 9> dRdr;
    Matrix<FLOAT, 3, 3> R = Rodrigues(rr, dRdr);
    imagePoints = Matrix<FLOAT, MAX_KEYPOINT_NUM, 2>();
    for (int32_t i = 0; i < count; i++)
    {
        FLOAT X = objectPoints.val[i][0], Y = objectPoints.val[i][1], Z = objectPoints.val[i][2];
        FLOAT x = R.val[0][0] * X + R.val[0][1] * Y + R.val[0][2] * Z + t_vec.val[0][0];
        FLOAT y = R.val[1][0] * X + R.val[1][1] * Y + R.val[1][2] * Z + t_vec.val[1][0];
        FLOAT z = R.val[2][0] * X + R.val[2][1] * Y + R.val[2][2] * Z + t_vec.val[2][0];
        z = z ? 1. / z : 1;
        x *= z;
        y *= z;

        imagePoints.val[i][0] = x * fx + cx;
        imagePoints.val[i][1] = y * fy + cy;

        FLOAT dxdt[] = {z, 0, -x * z}, dydt[] = {0, z, -y * z};
        for (int32_t j = 0; j < 3; j++)
        {
            dpdt.val[2 * i][j] = fx * dxdt[j];
            dpdt.val[2 * i + 1][j] = fy * dydt[j];
        }

        FLOAT dx0dr[] =
            {
                X * dRdr.val[0][0] + Y * dRdr.val[0][1] + Z * dRdr.val[0][2],
                X * dRdr.val[1][0] + Y * dRdr.val[1][1] + Z * dRdr.val[1][2],
                X * dRdr.val[2][0] + Y * dRdr.val[2][1] + Z * dRdr.val[2][2]};
        FLOAT dy0dr[] =
            {
                X * dRdr.val[0][3] + Y * dRdr.val[0][4] + Z * dRdr.val[0][5],
                X * dRdr.val[1][3] + Y * dRdr.val[1][4] + Z * dRdr.val[1][5],
                X * dRdr.val[2][3] + Y * dRdr.val[2][4] + Z * dRdr.val[2][5]};
        FLOAT dz0dr[] =
            {
                X * dRdr.val[0][6] + Y * dRdr.val[0][7] + Z * dRdr.val[0][8],
                X * dRdr.val[1][6] + Y * dRdr.val[1][7] + Z * dRdr.val[1][8],
                X * dRdr.val[2][6] + Y * dRdr.val[2][7] + Z * dRdr.val[2][8]};
        for (int32_t j = 0; j < 3; j++)
        {
            double dxdr = z * (dx0dr[j] - x * dz0dr[j]);
            double dydr = z * (dy0dr[j] - y * dz0dr[j]);
            dpdr.val[2 * i][j] = fx * dxdr;
            dpdr.val[2 * i + 1][j] = fy * dydr;
        }
    }
}

PnPIterative::PnPIterative(Matrix<FLOAT, MAX_KEYPOINT_NUM, 3> &opoints,
                           Matrix<FLOAT, MAX_KEYPOINT_NUM, 2> &ipoints,
                           FLOAT _fx, FLOAT _fy, FLOAT _cx, FLOAT _cy)
{
    number_of_points = opoints.m;
    matM = Matrix<FLOAT, MAX_KEYPOINT_NUM, 3>();
    _m = Matrix<FLOAT, MAX_KEYPOINT_NUM, 2>();
    for (int32_t i = 0; i < number_of_points; i++)
    {
        for (int32_t j = 0; j < 3; j++)
            matM.val[i][j] = opoints.val[i][j];
        for (int32_t j = 0; j < 2; j++)
            _m.val[i][j] = ipoints.val[i][j];
    }
    cx = _cx;
    cy = _cy;
    fx = _fx;
    fy = _fy;

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            matV.val[i][j] = matM.val[i][j];
}

void PnPIterative::compute(Matrix<FLOAT, 3, 3> &rmat,
                           Matrix<FLOAT, 3, 1> &tvec)
{

    mn = Matrix<FLOAT, MAX_KEYPOINT_NUM, 2>();
    for (int32_t i = 0; i < number_of_points; i++)
    {
        FLOAT x, y;
        x = _m.val[i][0];
        y = _m.val[i][1];

        x = (x - cx) * (1. / fx);
        y = (y - cy) * (1. / fy);

        mn.val[i][0] = x;
        mn.val[i][1] = y;
    }

    // non-planar structure. Use DLT method
    matL = Matrix<FLOAT, 2 * MAX_KEYPOINT_NUM, 12>();
    for (int32_t i = 0; i < number_of_points; i++)
    {
        FLOAT x = -mn.val[i][0], y = -mn.val[i][1];
        matL.val[2 * i][0] = matL.val[2 * i + 1][4] = matM.val[i][0];
        matL.val[2 * i][1] = matL.val[2 * i + 1][5] = matM.val[i][1];
        matL.val[2 * i][2] = matL.val[2 * i + 1][6] = matM.val[i][2];
        matL.val[2 * i][3] = matL.val[2 * i + 1][7] = 1.;
        matL.val[2 * i][4] = matL.val[2 * i][5] = matL.val[2 * i][6] = matL.val[2 * i][7] = 0.;
        matL.val[2 * i + 1][0] = matL.val[2 * i + 1][1] = matL.val[2 * i + 1][2] = matL.val[2 * i + 1][3] = 0.;
        matL.val[2 * i][8] = x * matM.val[i][0];
        matL.val[2 * i][9] = x * matM.val[i][1];
        matL.val[2 * i][10] = x * matM.val[i][2];
        matL.val[2 * i][11] = x;
        matL.val[2 * i + 1][8] = y * matM.val[i][0];
        matL.val[2 * i + 1][9] = y * matM.val[i][1];
        matL.val[2 * i + 1][10] = y * matM.val[i][2];
        matL.val[2 * i + 1][11] = y;
    }

    LL = matL.multrans();

    LL.svd(LU, LW, LV);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            rmat.val[i][j] = LU.val[i][j];

    RR = Matrix<FLOAT, 3, 3>();
    RR.val[0][0] = LV.val[0][11];
    RR.val[0][1] = LV.val[1][11];
    RR.val[0][2] = LV.val[2][11];
    RR.val[1][0] = LV.val[4][11];
    RR.val[1][1] = LV.val[5][11];
    RR.val[1][2] = LV.val[6][11];
    RR.val[2][0] = LV.val[8][11];
    RR.val[2][1] = LV.val[9][11];
    RR.val[2][2] = LV.val[10][11];

    tt = Matrix<FLOAT, 3, 1>();
    tt.val[0][0] = LV.val[3][11];
    tt.val[1][0] = LV.val[7][11];
    tt.val[2][0] = LV.val[11][11];

    FLOAT det =
        RR.val[0][0] * RR.val[1][1] * RR.val[2][2] +
        RR.val[0][1] * RR.val[1][2] * RR.val[2][0] +
        RR.val[0][2] * RR.val[1][0] * RR.val[2][1] -
        RR.val[0][2] * RR.val[1][1] * RR.val[2][0] -
        RR.val[0][1] * RR.val[1][0] * RR.val[2][2] -
        RR.val[0][0] * RR.val[1][2] * RR.val[2][1];
    if (det < 0)
    {
        RR = RR * (-1.);
        tt = tt * (-1.);
    }

    FLOAT sc = RR.norm();
    RR.svd(matU, matW, matV);
    matR = matU * matV.trans();
    t = tt * (matR.norm() / sc);
    r = Rodrigues(matR);

    // refine extrinsic parameters using iterative algorithm
    LevMarq solver = LevMarq(2 * number_of_points);
    solver.param.val[0][0] = r.val[0][0];
    solver.param.val[1][0] = r.val[1][0];
    solver.param.val[2][0] = r.val[2][0];
    solver.param.val[3][0] = t.val[0][0];
    solver.param.val[4][0] = t.val[1][0];
    solver.param.val[5][0] = t.val[2][0];

    for (;;)
    {
        bool proceed = solver.update(param, matJ); //, err);
        solver.param = param;
        for (int32_t i = 0; i < 3; i++)
        {
            r.val[i][0] = param.val[i][0];
            t.val[i][0] = param.val[i + 3][0];
        }

        if (!proceed || !solver.is_err)
            break;
        if (solver.is_matJ)
        {
            for (int32_t i = 0; i < number_of_points * 2; i++)
            {
                for (int32_t j = 0; j < 3; j++)
                {
                    dpdr.val[i][j] = matJ.val[i][j];
                    dpdt.val[i][j] = param.val[i][j + 3];
                }
            }
            ProjectPoints(matM, r, t, fx, fy, cx, cy,
                          err, dpdr, dpdt);
            for (int32_t i = 0; i < number_of_points * 2; i++)
            {
                for (int32_t j = 0; j < 3; j++)
                {
                    solver.J.val[i][j] = dpdr.val[i][j];
                    solver.J.val[i][j + 3] = dpdt.val[i][j];
                }
            }
        }
        else
        {
            ProjectPoints(matM, r, t, fx, fy, cx, cy,
                          err);
        }

        for (int32_t i = 0; i < _m.m; i++)
        {
            solver.err.val[2 * i][0] = err.val[i][0] - _m.val[i][0];
            solver.err.val[2 * i + 1][0] = err.val[i][1] - _m.val[i][1];
        }
    }
    rmat = Rodrigues(r);
    tvec = t;
}

LevMarq::LevMarq(int32_t nerrs)
{
    prevParam = Matrix<FLOAT, NUM_OF_PARAMS, 1>();
    param = Matrix<FLOAT, NUM_OF_PARAMS, 1>();
    JtJ = Matrix<FLOAT, NUM_OF_PARAMS, NUM_OF_PARAMS>();
    JtErr = Matrix<FLOAT, NUM_OF_PARAMS, 1>();
    JtJN = Matrix<FLOAT, NUM_OF_PARAMS, NUM_OF_PARAMS>();
    JtJV = Matrix<FLOAT, NUM_OF_PARAMS, 1>();
    JtJW = Matrix<FLOAT, NUM_OF_PARAMS, 1>();
    J = Matrix<FLOAT, NUM_OF_ERRS, NUM_OF_PARAMS>();
    J.m = nerrs;
    err = Matrix<FLOAT, NUM_OF_ERRS, 1>();
    err.m = nerrs;
    errNorm = prevErrNorm = DBL_MAX;
    lambdaLg10 = -3;
    max_iter = 20;
    epsilon = FLT_EPSILON;
    state = STARTED;
    iters = 0;
}

bool LevMarq::update(Matrix<FLOAT, NUM_OF_PARAMS, 1> &_param,
                     Matrix<FLOAT, NUM_OF_ERRS, NUM_OF_PARAMS> &matJ)
{
    is_matJ = false;
    is_err = false;

    if (state == DONE)
    {
        _param = param;
        return false;
    }

    if (state == STARTED)
    {
        _param = param;
        for (int32_t i = 0; i < J.m; i++)
            for (int32_t j = 0; j < J.n; j++)
                J.val[i][j] = 0.0;
        for (int32_t i = 0; i < err.m; i++)
            for (int32_t j = 0; j < err.n; j++)
                err.val[i][j] = 0.0;
        matJ = J;
        is_matJ = true;
        is_err = true;
        state = CALC_J;
        return true;
    }

    if (state == CALC_J)
    {
        JtJ = J.multrans();
        JtErr = J.trans() * err;
        prevParam = param;
        step();
        if (iters == 0)
            prevErrNorm = err.norm();
        _param = param;
        for (int32_t i = 0; i < err.m; i++)
            for (int32_t j = 0; j < err.n; j++)
                err.val[i][j] = 0.0;
        is_err = true;
        state = CHECK_ERR;
        return true;
    }

    errNorm = err.norm();
    if (errNorm > prevErrNorm)
    {
        if (++lambdaLg10 <= 16)
        {
            step();
            _param = param;
            for (int32_t i = 0; i < err.m; i++)
                for (int32_t j = 0; j < err.n; j++)
                    err.val[i][j] = 0.0;
            is_err = true;
            state = CHECK_ERR;
            return true;
        }
    }

    lambdaLg10 = MAX(lambdaLg10 - 1, -16);
    if (++iters >= max_iter || ((param - prevParam).norm() / prevParam.norm() < epsilon))
    {
        _param = param;
        state = DONE;
        return true;
    }

    prevErrNorm = errNorm;
    _param = param;
    for (int32_t i = 0; i < J.m; i++)
        for (int32_t j = 0; j < J.n; j++)
            J.val[i][j] = 0.0;
    matJ = J;
    is_matJ = true;
    is_err = true;
    state = CALC_J;
    return true;
}

void LevMarq::step()
{
    const FLOAT LOG10 = log(10.);
    FLOAT lambda = exp(lambdaLg10 * LOG10);
    int nparams = param.m;

    JtJN = JtJ;
    for (int i = 0; i < JtJN.m; i++)
        JtJN.val[i][i] *= 1. + lambda;

    solve(JtJN, JtJW, JtErr);
    for (int32_t i = 0; i < JtJW.m; i++)
        param.val[i][0] = prevParam.val[i][0] - JtJW.val[i][0];
}
