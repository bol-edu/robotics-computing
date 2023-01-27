#include "PnPIterative.h"

#define MAX(a, b) (a > b ? a : b)
#define DBL_MAX 1.7976931348623158e+308
#define DBL_MIN 2.2250738585072014e-308

FLOAT NORM(Matrix a)
{
    FLOAT b = 0.0;
    for (int i = 0; i < a.m; i++)
        for (int j = 0; j < a.n; j++)
            b += a.val[i][j] * a.val[i][j];
    return sqrt(b);
}

Matrix Rodrigues(Matrix &src)
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

    Matrix dst = Matrix(3, 1);
    dst.val[0][0] = rx * vth;
    dst.val[1][0] = ry * vth;
    dst.val[2][0] = rz * vth;
    return dst;
}

void ProjectPoints(Matrix &objectPoints, Matrix &r_vec, Matrix &t_vec, Matrix &A,
                   Matrix &imagePoints, Matrix &dpdr, Matrix &dpdt)
{
    /*Ptr<CvMat> matM, _m;
    Ptr<CvMat> _dpdr, _dpdt;

    int i, j, count;
    int calc_derivatives;
    const CvPoint3D64f *M;
    CvPoint2D64f *m;
    double r[3], R[9], dRdr[27], t[3], a[9], k[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, fx, fy, cx, cy;
    Matx33d matTilt = Matx33d::eye();
    CvMat _r, matR = cvMat(3, 3, CV_64F, R);
    CvMat _t, _a = cvMat(3, 3, CV_64F, a);
    CvMat _dRdr = cvMat(3, 9, CV_64F, dRdr);
    double *dpdr_p = 0, *dpdt_p = 0;
    int dpdr_step = 0, dpdt_step = 0;

    int total = objectPoints->rows * objectPoints->cols * CV_MAT_CN(objectPoints->type); // 3*/

    int32_t count = objectPoints.m;

    matM.reset(cvCreateMat(objectPoints->rows, objectPoints->cols, CV_64FC3));
    cvConvert(objectPoints, matM);

    _m.reset(cvCreateMat(imagePoints->rows, imagePoints->cols, CV_64FC2));
    cvConvert(imagePoints, _m);

    M = (CvPoint3D64f *)matM->data.db;
    m = (CvPoint2D64f *)_m->data.db;

    _r = cvMat(r_vec->rows, r_vec->cols, CV_64FC1, r);
    cvConvert(r_vec, &_r);
    // Rodrigues(_r, matR, _dRdr);
    cvRodrigues2(&_r, &matR, &_dRdr);

    _t = cvMat(t_vec->rows, t_vec->cols, CV_64FC1, t);
    cvConvert(t_vec, &_t);

    cvConvert(A, &_a);
    fx = a[0];
    fy = a[4];
    cx = a[2];
    cy = a[5];

    if (dpdr)
    {
        _dpdr.reset(cvCloneMat(dpdr));
        dpdr_p = _dpdr->data.db;
        dpdr_step = _dpdr->step / sizeof(dpdr_p[0]);
    }

    if (dpdt)
    {
        _dpdt.reset(cvCloneMat(dpdt));

        dpdt_p = _dpdt->data.db;
        dpdt_step = _dpdt->step / sizeof(dpdt_p[0]);
    }

    calc_derivatives = dpdr || dpdt;

    for (i = 0; i < count; i++)
    {

        double X = M[i].x, Y = M[i].y, Z = M[i].z;
        double x = R[0] * X + R[1] * Y + R[2] * Z + t[0];
        double y = R[3] * X + R[4] * Y + R[5] * Z + t[1];
        double z = R[6] * X + R[7] * Y + R[8] * Z + t[2];
        double r2, r4, r6, a1, a2, a3, cdist, icdist2;
        double xd, yd, xd0, yd0, invProj;
        Vec3d vecTilt;
        Vec3d dVecTilt;
        Matx22d dMatTilt;
        Vec2d dXdYd;

        double z0 = z;
        z = z ? 1. / z : 1;
        x *= z;
        y *= z;

        r2 = x * x + y * y;
        r4 = r2 * r2;
        r6 = r4 * r2;
        a1 = 2 * x * y;
        a2 = r2 + 2 * x * x;
        a3 = r2 + 2 * y * y;
        cdist = 1 + k[0] * r2 + k[1] * r4 + k[4] * r6;
        icdist2 = 1. / (1 + k[5] * r2 + k[6] * r4 + k[7] * r6);
        xd0 = x * cdist * icdist2 + k[2] * a1 + k[3] * a2 + k[8] * r2 + k[9] * r4;
        yd0 = y * cdist * icdist2 + k[2] * a3 + k[3] * a1 + k[10] * r2 + k[11] * r4;

        // additional distortion by projecting onto a tilt plane
        vecTilt = matTilt * Vec3d(xd0, yd0, 1);
        invProj = vecTilt(2) ? 1. / vecTilt(2) : 1;
        xd = invProj * vecTilt(0);
        yd = invProj * vecTilt(1);

        m[i].x = xd * fx + cx;
        m[i].y = yd * fy + cy;

        if (calc_derivatives)
        {
            for (int row = 0; row < 2; ++row)
                for (int col = 0; col < 2; ++col)
                    dMatTilt(row, col) = matTilt(row, col) * vecTilt(2) - matTilt(2, col) * vecTilt(row);
            double invProjSquare = (invProj * invProj);
            dMatTilt *= invProjSquare;

            if (dpdt_p)
            {
                double dxdt[] = {z, 0, -x * z}, dydt[] = {0, z, -y * z};
                for (j = 0; j < 3; j++)
                {
                    double dr2dt = 2 * x * dxdt[j] + 2 * y * dydt[j];
                    double dcdist_dt = k[0] * dr2dt + 2 * k[1] * r2 * dr2dt + 3 * k[4] * r4 * dr2dt;
                    double dicdist2_dt = -icdist2 * icdist2 * (k[5] * dr2dt + 2 * k[6] * r2 * dr2dt + 3 * k[7] * r4 * dr2dt);
                    double da1dt = 2 * (x * dydt[j] + y * dxdt[j]);
                    double dmxdt = (dxdt[j] * cdist * icdist2 + x * dcdist_dt * icdist2 + x * cdist * dicdist2_dt +
                                    k[2] * da1dt + k[3] * (dr2dt + 4 * x * dxdt[j]) + k[8] * dr2dt + 2 * r2 * k[9] * dr2dt);
                    double dmydt = (dydt[j] * cdist * icdist2 + y * dcdist_dt * icdist2 + y * cdist * dicdist2_dt +
                                    k[2] * (dr2dt + 4 * y * dydt[j]) + k[3] * da1dt + k[10] * dr2dt + 2 * r2 * k[11] * dr2dt);
                    dXdYd = dMatTilt * Vec2d(dmxdt, dmydt);
                    dpdt_p[j] = fx * dXdYd(0);
                    dpdt_p[dpdt_step + j] = fy * dXdYd(1);
                }
                dpdt_p += dpdt_step * 2;
            }

            if (dpdr_p)
            {
                double dx0dr[] =
                    {
                        X * dRdr[0] + Y * dRdr[1] + Z * dRdr[2],
                        X * dRdr[9] + Y * dRdr[10] + Z * dRdr[11],
                        X * dRdr[18] + Y * dRdr[19] + Z * dRdr[20]};
                double dy0dr[] =
                    {
                        X * dRdr[3] + Y * dRdr[4] + Z * dRdr[5],
                        X * dRdr[12] + Y * dRdr[13] + Z * dRdr[14],
                        X * dRdr[21] + Y * dRdr[22] + Z * dRdr[23]};
                double dz0dr[] =
                    {
                        X * dRdr[6] + Y * dRdr[7] + Z * dRdr[8],
                        X * dRdr[15] + Y * dRdr[16] + Z * dRdr[17],
                        X * dRdr[24] + Y * dRdr[25] + Z * dRdr[26]};
                for (j = 0; j < 3; j++)
                {
                    double dxdr = z * (dx0dr[j] - x * dz0dr[j]);
                    double dydr = z * (dy0dr[j] - y * dz0dr[j]);
                    double dr2dr = 2 * x * dxdr + 2 * y * dydr;
                    double dcdist_dr = (k[0] + 2 * k[1] * r2 + 3 * k[4] * r4) * dr2dr;
                    double dicdist2_dr = -icdist2 * icdist2 * (k[5] + 2 * k[6] * r2 + 3 * k[7] * r4) * dr2dr;
                    double da1dr = 2 * (x * dydr + y * dxdr);
                    double dmxdr = (dxdr * cdist * icdist2 + x * dcdist_dr * icdist2 + x * cdist * dicdist2_dr +
                                    k[2] * da1dr + k[3] * (dr2dr + 4 * x * dxdr) + (k[8] + 2 * r2 * k[9]) * dr2dr);
                    double dmydr = (dydr * cdist * icdist2 + y * dcdist_dr * icdist2 + y * cdist * dicdist2_dr +
                                    k[2] * (dr2dr + 4 * y * dydr) + k[3] * da1dr + (k[10] + 2 * r2 * k[11]) * dr2dr);
                    dXdYd = dMatTilt * Vec2d(dmxdr, dmydr);
                    dpdr_p[j] = fx * dXdYd(0);
                    dpdr_p[dpdr_step + j] = fy * dXdYd(1);
                }
                dpdr_p += dpdr_step * 2;
            }
        }
    }

    cvConvert(_m, imagePoints);

    if (_dpdr != dpdr)
        cvConvert(_dpdr, dpdr);

    if (_dpdt != dpdt)
        cvConvert(_dpdt, dpdt);
}

PnPIterative::PnPIterative(Matrix &opoints, Matrix &ipoints, FLOAT _fx, FLOAT _fy, FLOAT _cx, FLOAT _cy)
{
    number_of_points = opoints.m;
    matM = Matrix(number_of_points, 3);
    _m = Matrix(number_of_points, 2);
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
}

void PnPIterative::compute(Matrix &rmat, Matrix &tvec)
{
    const int32_t max_iter = 20;

    Matrix mn = Matrix(number_of_points, 2);
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
    Matrix matL = Matrix(2 * number_of_points, 12);
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

    Matrix LL = matL.multrans();

    Matrix LW, LU, LV;
    LL.svd(LU, LW, LV);

    Matrix RR = Matrix(3, 3);
    RR.val[0][0] = LV.val[0][11];
    RR.val[0][1] = LV.val[1][11];
    RR.val[0][2] = LV.val[2][11];
    RR.val[1][0] = LV.val[4][11];
    RR.val[1][1] = LV.val[5][11];
    RR.val[1][2] = LV.val[6][11];
    RR.val[2][0] = LV.val[8][11];
    RR.val[2][1] = LV.val[9][11];
    RR.val[2][2] = LV.val[10][11];

    Matrix tt = Matrix(3, 1);
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

    FLOAT sc = NORM(RR);
    Matrix matW, matU, matV;
    RR.svd(matU, matW, matV);
    Matrix matR = matU * matV.trans();
    Matrix t = tt * (NORM(matR) / sc);
    Matrix r = Rodrigues(matR);

    // refine extrinsic parameters using iterative algorithm
    LevMarq solver(6, number_of_points * 2);
    solver.param.val[0][0] = r.val[0][0];
    solver.param.val[1][0] = r.val[1][0];
    solver.param.val[2][0] = r.val[2][0];
    solver.param.val[3][0] = t.val[0][0];
    solver.param.val[4][0] = t.val[1][0];
    solver.param.val[5][0] = t.val[2][0];

    for (;;)
    {
        Matrix matJ, err, param;
        // const CvMat *__param = 0;
        bool proceed = solver.update(param, matJ, err);
        solver.param = param;
        // cvCopy(__param, &_param);
        if (!proceed || !err)
            break;
        cvReshape(_err, _err, 2, 1);
        if (matJ)
        {
            cvGetCols(matJ, &_dpdr, 0, 3);
            cvGetCols(matJ, &_dpdt, 3, 6);
            cvProjectPoints2(matM, &_r, &_t, &matA, 0,
                             _err, &_dpdr, &_dpdt);
        }
        else
        {
            cvProjectPoints2(matM, &_r, &_t, &matA, 0,
                             _err, 0, 0);
        }
        cvSub(_err, _m, _err);
        cvReshape(_err, _err, 1, 2 * count);
    }
    /*cvCopy(solver.param, &_param);

    _r = cvMat(c_rvec.rows, c_rvec.cols, CV_64F, param);
    _t = cvMat(c_tvec.rows, c_tvec.cols, CV_64F, param + 3);

    cvConvert(&_r, &c_rvec);
    cvConvert(&_t, &c_tvec);*/
}

LevMarq::LevMarq(int32_t nparams, int32_t nerrs)
{
    clear();
    prevParam = Matrix(nparams, 1);
    param = Matrix(nparams, 1);
    JtJ = Matrix(nparams, nparams);
    JtErr = Matrix(nparams, 1);
    JtJN = Matrix(nparams, nparams);
    JtJV = Matrix(nparams, 1);
    JtJW = Matrix(nparams, 1);
    J = Matrix(nerrs, nparams);
    err = Matrix(nerrs, 1);
    errNorm = prevErrNorm = DBL_MAX;
    lambdaLg10 = -3;
    max_iter = 20;
    epsilon = FLT_EPSILON;
    state = STARTED;
    iters = 0;
}

bool LevMarq::update(Matrix &_param, Matrix &matJ, Matrix &_err)
{
    for (int32_t i = 0; i < matJ.m; i++)
        for (int32_t j = 0; j < matJ.n; j++)
            matJ.val[i][j] = 0.0;
    for (int32_t i = 0; i < _err.m; i++)
        for (int32_t j = 0; j < _err.n; j++)
            _err.val[i][j] = 0.0;

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
        _err = err;
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
            prevErrNorm = NORM(err);
        _param = param;
        for (int32_t i = 0; i < err.m; i++)
            for (int32_t j = 0; j < err.n; j++)
                err.val[i][j] = 0.0;
        _err = err;
        state = CHECK_ERR;
        return true;
    }

    errNorm = NORM(err);
    if (errNorm > prevErrNorm)
    {
        if (++lambdaLg10 <= 16)
        {
            step();
            _param = param;
            for (int32_t i = 0; i < err.m; i++)
                for (int32_t j = 0; j < err.n; j++)
                    err.val[i][j] = 0.0;
            _err = err;
            state = CHECK_ERR;
            return true;
        }
    }

    lambdaLg10 = MAX(lambdaLg10 - 1, -16);
    if (++iters >= max_iter || (NORM(param - prevParam) / NORM(prevParam) < epsilon))
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
    _err = err;
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

    Matrix::solve(JtJN, JtJW, JtErr);
}

void LevMarq::clear()
{
    prevParam.releaseMemory();
    param.releaseMemory();
    J.releaseMemory();
    err.releaseMemory();
    JtJ.releaseMemory();
    JtJN.releaseMemory();
    JtErr.releaseMemory();
    JtJV.releaseMemory();
    JtJW.releaseMemory();
}

LevMarq::~LevMarq()
{
    clear();
}