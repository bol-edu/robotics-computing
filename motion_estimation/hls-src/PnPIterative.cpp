#include "PnPIterative.h"

#define MAX(a, b) (a > b ? a : b)

FLOAT NORM(Matrix a)
{
    FLOAT b = 0.0;
    for (int i = 0; i < a.m; i++)
        for (int j = 0; j < a.n; j++)
            b += a.val[i][j] * a.val[i][j];
    return sqrt(b);
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

    FLOAT sc = NORM(RR); //// !!!!!!!!!!!!!!!!!!!!!
    Matrix matW, matU, matV;
    RR.svd(matU, matW, matV);
    Matrix matR = matU.trans() * matV;
    cout << matR << endl
         << endl;
    // cvGEMM(&matU, &matV, 1, 0, 0, &matR, CV_GEMM_A_T);
    // matR = cvMat(Mat(matU.rows, matU.cols, matU.type, matU.data.db).t() * Mat(matV.rows, matV.cols, matV.type, matV.data.db));
    // cvScale(&_tt, &_t, cvNorm(&matR) / sc);
    // cvRodrigues2(&matR, &_r, 0);

    /*cvReshape(matM, matM, 3, 1);
    cvReshape(_mn, _mn, 2, 1);

    // refine extrinsic parameters using iterative algorithm
    CvLevMarq solver(6, count * 2, cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, max_iter, FLT_EPSILON), true);
    cvCopy(&_param, solver.param);

    for (;;)
    {
        CvMat *matJ = 0, *_err = 0;
        const CvMat *__param = 0;
        bool proceed = solver.update(__param, matJ, _err);
        cvCopy(__param, &_param);
        if (!proceed || !_err)
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
    cvCopy(solver.param, &_param);

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