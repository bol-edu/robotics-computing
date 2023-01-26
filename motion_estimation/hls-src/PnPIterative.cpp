#include "PnPIterative.h"

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

    Matrix _mn = Matrix(number_of_points, 2);
    for (int32_t i = 0; i < number_of_points; i++)
    {
        FLOAT x, y;
        x = _m.val[i][0];
        y = _m.val[i][1];

        x = (x - cx) * (1. / fx);
        y = (y - cy) * (1. / fy);

        _mn.val[i][0] = x;
        _mn.val[i][1] = y;
    }

    Matrix Mc = Matrix(1, 3);
    for (int32_t i = 0; i < number_of_points; i++)
    {
        for (int32_t j = 0; j < 3; j++)
            Mc.val[0][j] += matM.val[i][j];
    }
    for (int32_t j = 0; j < 3; j++)
        Mc.val[0][j] /= number_of_points;

    cout << Mc << endl
         << endl;
    /*cvReshape(matM, matM, 1, count);
    cvMulTransposed(matM, &_MM, 1, &_Mc);
    __cvSVD(&_MM, &matW, 0, &matV, CV_SVD_MODIFY_A + CV_SVD_V_T);


        // non-planar structure. Use DLT method
        double *L;
        double LL[12 * 12], LW[12], LV[12 * 12], sc;
        CvMat _LL = cvMat(12, 12, CV_64F, LL);
        CvMat _LW = cvMat(12, 1, CV_64F, LW);
        CvMat _LV = cvMat(12, 12, CV_64F, LV);
        CvMat _RRt, _RR, _tt;
        CvPoint3D64f *M = (CvPoint3D64f *)matM->data.db;
        CvPoint2D64f *mn = (CvPoint2D64f *)_mn->data.db;

        matL.reset(cvCreateMat(2 * count, 12, CV_64F));
        L = matL->data.db;

        for (i = 0; i < count; i++, L += 24)
        {
            double x = -mn[i].x, y = -mn[i].y;
            L[0] = L[16] = M[i].x;
            L[1] = L[17] = M[i].y;
            L[2] = L[18] = M[i].z;
            L[3] = L[19] = 1.;
            L[4] = L[5] = L[6] = L[7] = 0.;
            L[12] = L[13] = L[14] = L[15] = 0.;
            L[8] = x * M[i].x;
            L[9] = x * M[i].y;
            L[10] = x * M[i].z;
            L[11] = x;
            L[20] = y * M[i].x;
            L[21] = y * M[i].y;
            L[22] = y * M[i].z;
            L[23] = y;
        }

        cvMulTransposed(matL, &_LL, 1);
        __cvSVD(&_LL, &_LW, 0, &_LV, CV_SVD_MODIFY_A + CV_SVD_V_T);
        _RRt = cvMat(3, 4, CV_64F, LV + 11 * 12);
        cvGetCols(&_RRt, &_RR, 0, 3);
        cvGetCol(&_RRt, &_tt, 3);

        if (cvDet(&_RR) < 0)
        {
            cvScale(&_RRt, &_RRt, -1);
        }

        sc = cvNorm(&_RR);
        __cvSVD(&_RR, &matW, &matU, &matV, CV_SVD_MODIFY_A + CV_SVD_U_T + CV_SVD_V_T);
        // cvGEMM(&matU, &matV, 1, 0, 0, &matR, CV_GEMM_A_T);
        matR = cvMat(Mat(matU.rows, matU.cols, matU.type, matU.data.db).t() * Mat(matV.rows, matV.cols, matV.type, matV.data.db));
        cvScale(&_tt, &_t, cvNorm(&matR) / sc);
        cvRodrigues2(&matR, &_r, 0);

        cvReshape(matM, matM, 3, 1);
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