#include "EstimateMotion.h"

using namespace cv;

void EstimateMotion::estimate_motion(vector<vector<DMatch>> match, vector<KeyPoint> kp1, vector<KeyPoint> kp2,
                                     Mat k, Mat depth1, int max_depth,
                                     Mat &rmat, Mat &tvec,
                                     Mat &image1_points, Mat &image2_points,
                                     int maxIter, float error, float confidence)
{
    Mat image1_points__ = Mat(0, 2, CV_32F);
    image1_points = Mat(0, 2, CV_32F);
    Mat image2_points__ = Mat(0, 2, CV_32F);
    image2_points = Mat(0, 2, CV_32F);
    Mat rvec;
    Mat distCoef = Mat::zeros(1, 5, CV_32F);

    for (int m = 0; m < match.size(); m++)
    {
        image1_points__.push_back(kp1[match[m][0].queryIdx].pt);
        image2_points__.push_back(kp2[match[m][0].trainIdx].pt);
    }

    if (!depth1.empty())
    {
        float cx = k.at<float>(0, 2);
        float cy = k.at<float>(1, 2);
        float fx = k.at<float>(0, 0);
        float fy = k.at<float>(1, 1);
        Mat object_points = Mat::zeros(0, 3, CV_32F);

        for (int i = 0; i < image1_points__.rows; i++)
        {

            float u = image1_points__.at<float>(i, 0);
            float v = image1_points__.at<float>(i, 1);
            float z = depth1.at<float>((int)v, (int)u);

            if (z > max_depth)
            {
                continue;
            }

            float x = z * (u - cx) / fx;
            float y = z * (v - cy) / fy;

            Mat vec = Mat(1, 3, CV_32F);
            vec.at<float>(0, 0) = x;
            vec.at<float>(0, 1) = y;
            vec.at<float>(0, 2) = z;

            object_points.push_back(vec);
            image1_points.push_back(image1_points__.row(i));
            image2_points.push_back(image2_points__.row(i));
        }

        __solvePnPRansac__(object_points, image2_points, k, distCoef, rvec, tvec,
                           false, maxIter, error, confidence, SOLVEPNP_ITERATIVE);

        rmat.create(3, 3, rvec.depth());

        CvMat _crvec = cvMat(rvec), _crmat = cvMat(rmat), _cjacobian;

        bool ok = cvRodrigues2(&_crvec, &_crmat, 0) > 0;
        if (!ok)
            rmat = Scalar(0);
    }
}

bool EstimateMotion::__solvePnPRansac__(Mat opoints, Mat ipoints,
                                        Mat cameraMatrix, Mat distCoeffs,
                                        Mat &rvec, Mat &tvec, bool useExtrinsicGuess,
                                        int iterationsCount, float reprojectionError, double confidence,
                                        int flags)
{
    int npoints = opoints.rows;

    rvec = Mat(3, 1, CV_64FC1);
    tvec = Mat(3, 1, CV_64FC1);

    int modelPoints = 5;
    int ransac_kernel_method = SOLVEPNP_EPNP;

    double param1 = reprojectionError; // reprojection error
    double param2 = confidence;        // confidence
    int param3 = iterationsCount;      // number maximum iterations

    Mat _local_model(3, 2, CV_64FC1);
    Mat _mask_local_inliers(1, opoints.rows, CV_8UC1);

    // call Ransac
    // int result = __createRANSACPointSetRegistrator__(cb, modelPoints,
    // param1, param2, param3)->__run__(opoints, ipoints, _local_model, _mask_local_inliers);

    bool result = false;
    {
        OutputArray _model = _local_model;
        OutputArray _mask = _mask_local_inliers;
        Mat err, mask, model, bestModel, ms1, ms2;

        int iter, niters = MAX(iterationsCount, 1);
        int d1 = opoints.cols;
        int count = opoints.rows, maxGoodCount = 0;

        RNG rng((uint64)-1);

        if (count < modelPoints)
            return false;

        Mat bestMask0, bestMask;

        _mask.create(count, 1, CV_8U, -1, true);
        bestMask0 = bestMask = _mask.getMat();

        for (iter = 0; iter < niters; iter++)
        {
            int i, nmodels;

            // bool found = __getSubset__(m1, m2, ms1, ms2, rng, 10000);
            {
                cv::AutoBuffer<int> _idx(modelPoints);
                int *idx = _idx.data();

                const int d2 = ipoints.channels();

                int esz1 = (int)opoints.elemSize1() * d1;
                int esz2 = (int)ipoints.elemSize1() * d2;
                esz1 /= sizeof(int);
                esz2 /= sizeof(int);

                const int *m1ptr = opoints.ptr<int>();
                const int *m2ptr = ipoints.ptr<int>();

                ms1.create(modelPoints, 1, CV_32FC3);
                ms2.create(modelPoints, 1, CV_32FC2);

                int *ms1ptr = ms1.ptr<int>();
                int *ms2ptr = ms2.ptr<int>();

                for (int iters = 0; iters < 1000; ++iters)
                {
                    int i;

                    for (i = 0; i < modelPoints; ++i)
                    {
                        int idx_i;

                        for (idx_i = rng.uniform(0, count);
                             std::find(idx, idx + i, idx_i) != idx + i;
                             idx_i = rng.uniform(0, count))
                        {
                        }

                        idx[i] = idx_i;

                        for (int k = 0; k < esz1; ++k)
                            ms1ptr[i * esz1 + k] = m1ptr[idx_i * esz1 + k];

                        for (int k = 0; k < esz2; ++k)
                            ms2ptr[i * esz2 + k] = m2ptr[idx_i * esz2 + k];
                    }
                    break;
                }
            }
            // nmodels = cb->__runKernel__(ms1, ms2, model);
            {
                nmodels = __solvePnP__(ms1, ms2, cameraMatrix, distCoeffs,
                                       rvec, tvec, ransac_kernel_method);

                Mat _local_model;
                hconcat(rvec, tvec, _local_model);
                _local_model.copyTo(model);
            }
            Size modelSize(model.cols, model.rows / nmodels);

            for (i = 0; i < nmodels; i++)
            {
                Mat model_i = model.rowRange(i * modelSize.height, (i + 1) * modelSize.height);
                // int goodCount = __findInliers__(m1, m2, model_i, err, mask, threshold);
                int goodCount;
                {
                    // cb->__computeError__(m1, m2, model, err);
                    {
                        int i, count = opoints.checkVector(3);
                        Mat _rvec = model_i.col(0);
                        Mat _tvec = model_i.col(1);

                        Mat projpoints(count, 2, CV_32FC1);
                        projectPoints(opoints, _rvec, _tvec, cameraMatrix, distCoeffs, projpoints);

                        const Point2f *ipoints_ptr = ipoints.ptr<Point2f>();
                        const Point2f *projpoints_ptr = projpoints.ptr<Point2f>();

                        err.create(count, 1, CV_32FC1);

                        for (i = 0; i < count; ++i)
                            err.ptr<float>()[i] = (float)norm(Matx21f(ipoints_ptr[i] - projpoints_ptr[i]), NORM_L2SQR);
                    }
                    mask.create(err.size(), CV_8U);

                    const float *errptr = err.ptr<float>();
                    uchar *maskptr = mask.ptr<uchar>();
                    float t = (float)(reprojectionError * reprojectionError);
                    int i, n = (int)err.total();
                    goodCount = 0;
                    for (i = 0; i < n; i++)
                    {
                        int f = errptr[i] <= t;
                        maskptr[i] = (uchar)f;
                        goodCount += f;
                    }
                }

                if (goodCount > MAX(maxGoodCount, modelPoints - 1))
                {
                    std::swap(mask, bestMask);
                    model_i.copyTo(bestModel);
                    maxGoodCount = goodCount;
                    // niters = RANSACUpdateNumIters(confidence, (double)(count - goodCount) / count, modelPoints, niters);
                    {
                        double ep = (double)(count - goodCount) / count;
                        double p = confidence;
                        p = MAX(p, 0.);
                        p = MIN(p, 1.);
                        ep = MAX(ep, 0.);
                        ep = MIN(ep, 1.);

                        // avoid inf's & nan's
                        double num = MAX(1. - p, DBL_MIN);
                        double denom = 1. - std::pow(1. - ep, modelPoints);
                        if (denom < DBL_MIN)
                            niters = 0;

                        num = std::log(num);
                        denom = std::log(denom);

                        niters = denom >= 0 || -num >= niters * (-denom) ? niters : cvRound(num / denom);
                    }
                }
            }
        }

        transpose(bestMask, bestMask0);
        bestModel.copyTo(_model);
        result = true;
    }
label:

    vector<Point3d> opoints_inliers;
    vector<Point2d> ipoints_inliers;
    opoints = opoints.reshape(3);
    ipoints = ipoints.reshape(2);
    opoints.convertTo(opoints_inliers, CV_64F);
    ipoints.convertTo(ipoints_inliers, CV_64F);

    const uchar *mask = _mask_local_inliers.ptr<uchar>();
    int npoints1; // = __compressElems__(&opoints_inliers[0], mask, 1, npoints);
    //(T* ptr, const uchar* mask, int mstep, int count)
    {
        int i, j;
        for (i = j = 0; i < npoints; i++)
            if (mask[i])
            {
                if (i > j)
                    opoints_inliers[j] = opoints_inliers[i];
                j++;
            }
        npoints1 = j;
    }
    //__compressElems__(&ipoints_inliers[0], mask, 1, npoints);
    {
        int i, j;
        for (i = j = 0; i < npoints; i++)
            if (mask[i])
            {
                if (i > j)
                    ipoints_inliers[j] = ipoints_inliers[i];
                j++;
            }
    }

    opoints_inliers.resize(npoints1);
    ipoints_inliers.resize(npoints1);
    result = __solvePnP__(opoints_inliers, ipoints_inliers, cameraMatrix,
                          distCoeffs, rvec, tvec, flags)
                 ? 1
                 : -1;

    return true;
}

bool EstimateMotion::__solvePnP__(InputArray _opoints, InputArray _ipoints,
                                  InputArray _cameraMatrix, InputArray _distCoeffs,
                                  OutputArray _rvec, OutputArray _tvec, int flags)
{

    vector<Mat> rvecs, tvecs;
    Mat opoints = _opoints.getMat(), ipoints = _ipoints.getMat();
    int npoints = max(opoints.checkVector(3, CV_32F), opoints.checkVector(3, CV_64F));

    opoints = opoints.reshape(3, npoints);
    ipoints = ipoints.reshape(2, npoints);

    Mat cameraMatrix0 = _cameraMatrix.getMat();
    Mat cameraMatrix = Mat_<double>(cameraMatrix0);

    vector<Mat> vec_rvecs, vec_tvecs;
    if (flags == SOLVEPNP_EPNP)
    {
        Mat undistortedPoints;
        // undistortPoints(ipoints, undistortedPoints, cameraMatrix);
        {
            undistortedPoints.create(ipoints.rows, 1, CV_32FC2);

            const CvPoint2D32f *srcf = (const CvPoint2D32f *)ipoints.data;
            CvPoint2D32f *dstf = (CvPoint2D32f *)undistortedPoints.data;

            /*****************************************************/
            int stype = CV_MAT_TYPE(ipoints.type());
            int dtype = CV_MAT_TYPE(undistortedPoints.type());
            int sstep = ipoints.step / CV_ELEM_SIZE(stype);
            int dstep = undistortedPoints.step / CV_ELEM_SIZE(dtype);
            /*****************************************************/

            double fx = cameraMatrix.at<double>(0, 0);
            double fy = cameraMatrix.at<double>(1, 1);
            double cx = cameraMatrix.at<double>(0, 2);
            double cy = cameraMatrix.at<double>(1, 2);

            int n = ipoints.rows + ipoints.cols - 1;
            for (int i = 0; i < n; i++)
            {
                double x, y;
                x = srcf[i * sstep].x;
                y = srcf[i * sstep].y;

                x = (x - cx) / fx;
                y = (y - cy) / fy;

                dstf[i * dstep].x = (float)x;
                dstf[i * dstep].y = (float)y;
            }
        }

        epnp PnP(cameraMatrix, opoints, undistortedPoints);

        Mat rvec, tvec, R;
        PnP.compute_pose(R, tvec);

        rvec.create(3, 1, R.depth());

        CvMat _cR = cvMat(R), _crvec = cvMat(rvec), _cjacobian;

        bool ok = cvRodrigues2(&_cR, &_crvec, 0) > 0;
        if (!ok)
            rvec = Scalar(0);

        vec_rvecs.push_back(rvec);
        vec_tvecs.push_back(tvec);
    }

    if (flags == SOLVEPNP_ITERATIVE)
    {
        Mat rvec, tvec;
        rvec.create(3, 1, CV_64FC1);
        tvec.create(3, 1, CV_64FC1);

        CvMat c_objectPoints = cvMat(opoints), c_imagePoints = cvMat(ipoints);
        CvMat c_cameraMatrix = cvMat(cameraMatrix);
        CvMat c_rvec = cvMat(rvec), c_tvec = cvMat(tvec);
        // cvFindExtrinsicCameraParams2(&c_objectPoints, &c_imagePoints, &c_cameraMatrix,
        //&c_rvec, &c_tvec);
        // void cvFindExtrinsicCameraParams2(const CvMat * objectPoints,
        // const CvMat * imagePoints, const CvMat * A, CvMat * rvec, CvMat * tvec)
        {
            const int max_iter = 20;
            Ptr<CvMat> matM, _Mxy, _m, _mn, matL;

            int i, count;
            double a[9], R[9];
            double MM[9] = {0}, U[9] = {0}, V[9] = {0}, W[3] = {0};
            Scalar Mc;
            double param[6] = {0};
            CvMat matA = cvMat(3, 3, CV_64F, a);
            CvMat matR = cvMat(3, 3, CV_64F, R);
            CvMat _r = cvMat(3, 1, CV_64F, param);
            CvMat _t = cvMat(3, 1, CV_64F, param + 3);
            CvMat _Mc = cvMat(1, 3, CV_64F, Mc.val);
            CvMat _MM = cvMat(3, 3, CV_64F, MM);
            CvMat matU = cvMat(3, 3, CV_64F, U);
            CvMat matV = cvMat(3, 3, CV_64F, V);
            CvMat matW = cvMat(3, 1, CV_64F, W);
            CvMat _param = cvMat(6, 1, CV_64F, param);
            CvMat _dpdr, _dpdt;

            count = MAX(c_objectPoints.cols, c_objectPoints.rows);
            matM.reset(cvCreateMat(1, count, CV_64FC3));
            _m.reset(cvCreateMat(1, count, CV_64FC2));

            // cvConvertPointsHomogeneous(objectPoints, matM);
            {
                Mat dst = cvarrToMat(matM);
                const Mat dst0 = dst;

                dst = opoints.reshape(dst0.channels(), dst0.rows);

                dst.convertTo(dst0, dst0.type());
            }
            // cvConvertPointsHomogeneous(imagePoints, _m);
            {
                Mat dst = cvarrToMat(_m);
                const Mat dst0 = dst;

                dst = ipoints.reshape(dst0.channels(), dst0.rows);

                dst.convertTo(dst0, dst0.type());
            }
            cvConvert(&c_cameraMatrix, &matA);

            _mn.reset(cvCreateMat(1, count, CV_64FC2));
            _Mxy.reset(cvCreateMat(1, count, CV_64FC2));

            // cvUndistortPoints(_m, _mn, &matA, &_Ar);
            const CvPoint2D64f *srcd = (const CvPoint2D64f *)_m->data.ptr;
            CvPoint2D64f *dstd = (CvPoint2D64f *)_mn->data.ptr;
            int stype = CV_MAT_TYPE(_m->type);
            int dtype = CV_MAT_TYPE(_mn->type);
            int sstep = 1;
            int dstep = 1;

            double fx = CV_MAT_ELEM(matA, double, 0, 0);
            double fy = CV_MAT_ELEM(matA, double, 1, 1);
            double cx = CV_MAT_ELEM(matA, double, 0, 2);
            double cy = CV_MAT_ELEM(matA, double, 1, 2);

            int n = _m->rows + _m->cols - 1;
            for (int i = 0; i < n; i++)
            {
                double x, y;
                x = srcd[i * sstep].x;
                y = srcd[i * sstep].y;

                x = (x - cx) * (1. / fx);
                y = (y - cy) * (1. / fy);

                dstd[i * dstep].x = x;
                dstd[i * dstep].y = y;
            }

            Mc = cvAvg(matM);
            cvReshape(matM, matM, 1, count);
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
            cvConvert(&_t, &c_tvec);
        }

        vec_rvecs.push_back(rvec);
        vec_tvecs.push_back(tvec);
    }

    int solutions = (int)(vec_rvecs.size());

    OutputArrayOfArrays _rvecs = rvecs;
    OutputArrayOfArrays _tvecs = tvecs;

    _rvecs.create(solutions, 1, CV_64F);
    _tvecs.create(solutions, 1, CV_64F);

    for (int i = 0; i < solutions; i++)
    {
        _rvecs.getMatRef(i) = vec_rvecs[i];
        _tvecs.getMatRef(i) = vec_tvecs[i];
    }

    if (solutions > 0)
    {
        rvecs[0].convertTo(_rvec, _rvec.depth());
        tvecs[0].convertTo(_tvec, _tvec.depth());
    }

    return solutions > 0;
}

void EstimateMotion::__cvSVD(CvArr *aarr, CvArr *warr, CvArr *uarr, CvArr *varr, int flags)
{
    Mat a = cvarrToMat(aarr), w = cvarrToMat(warr), u, v;
    int m = a.rows, n = a.cols, type = a.type(), mn = max(m, n), nm = min(m, n);

    SVDD svd;

    svd.w = w;

    if (uarr)
    {
        u = cvarrToMat(uarr);
        svd.u = u;
    }

    v = cvarrToMat(varr);
    svd.vt = v;

    _SVDcompute(a, svd.w, svd.u, svd.vt);

    if (flags & CV_SVD_U_T)
        transpose(svd.u, u);
}

uchar *EstimateMotion::alignPtr(uchar *ptr, int n)
{
    return (uchar *)(((size_t)ptr + n - 1) & -n);
}

void EstimateMotion::_SVDcompute(Mat src, OutputArray _w,
                                 OutputArray _u, OutputArray _vt)
{
    int m = src.rows, n = src.cols;
    int type = src.type();

    size_t esz = src.elemSize(), astep = alignSize(m * esz, 16), vstep = alignSize(n * esz, 16);
    AutoBuffer<uchar> _buf(n * astep + n * vstep + n * esz + 32);
    uchar *buf = alignPtr(_buf.data(), 16);
    Mat temp_a(n, m, type, buf, astep);
    Mat temp_w(n, 1, type, buf + n * astep);
    Mat temp_u(n, m, type, buf, astep), temp_v;
    temp_v = Mat(n, n, type, alignPtr(buf + n * astep + n * esz, 16), vstep);

    transpose(src, temp_a);

    double minval = DBL_MIN;
    double eps = DBL_EPSILON * 10;

    AutoBuffer<double> Wbuf(n);
    double *W = Wbuf.data();
    int i, j, k, iter, max_iter = max(m, 30);
    double c, s;
    double sd;
    astep /= sizeof(temp_a.ptr<double>()[0]);
    vstep /= sizeof(temp_v.ptr<double>()[0]);

    for (i = 0; i < n; i++)
    {
        for (k = 0, sd = 0; k < m; k++)
        {
            double t = temp_a.ptr<double>()[i * astep + k];
            sd += (double)t * t;
        }
        W[i] = sd;

        for (k = 0; k < n; k++)
            temp_v.ptr<double>()[i * vstep + k] = 0;
        temp_v.ptr<double>()[i * vstep + i] = 1;
    }

    for (iter = 0; iter < max_iter; iter++)
    {
        bool changed = false;

        for (i = 0; i < n - 1; i++)
            for (j = i + 1; j < n; j++)
            {
                double *Ai = temp_a.ptr<double>() + i * astep, *Aj = temp_a.ptr<double>() + j * astep;
                double a = W[i], p = 0, b = W[j];

                for (k = 0; k < m; k++)
                    p += (double)Ai[k] * Aj[k];

                if (std::abs(p) <= eps * std::sqrt((double)a * b))
                    continue;

                p *= 2;
                double beta = a - b, gamma = hypot((double)p, beta);
                if (beta < 0)
                {
                    double delta = (gamma - beta) * 0.5;
                    s = (double)std::sqrt(delta / gamma);
                    c = (double)(p / (gamma * s * 2));
                }
                else
                {
                    c = (double)std::sqrt((gamma + beta) / (gamma * 2));
                    s = (double)(p / (gamma * c * 2));
                }

                a = b = 0;
                for (k = 0; k < m; k++)
                {
                    double t0 = c * Ai[k] + s * Aj[k];
                    double t1 = -s * Ai[k] + c * Aj[k];
                    Ai[k] = t0;
                    Aj[k] = t1;

                    a += (double)t0 * t0;
                    b += (double)t1 * t1;
                }
                W[i] = a;
                W[j] = b;

                changed = true;

                double *Vi = temp_v.ptr<double>() + i * vstep, *Vj = temp_v.ptr<double>() + j * vstep;
                k = 0; // vblas.givens(Vi, Vj, n, c, s);

                for (; k < n; k++)
                {
                    double t0 = c * Vi[k] + s * Vj[k];
                    double t1 = -s * Vi[k] + c * Vj[k];
                    Vi[k] = t0;
                    Vj[k] = t1;
                }
            }
        if (!changed)
            break;
    }

    for (i = 0; i < n; i++)
    {
        for (k = 0, sd = 0; k < m; k++)
        {
            double t = temp_a.ptr<double>()[i * astep + k];
            sd += (double)t * t;
        }
        W[i] = std::sqrt(sd);
    }

    for (i = 0; i < n - 1; i++)
    {
        j = i;
        for (k = i + 1; k < n; k++)
        {
            if (W[j] < W[k])
                j = k;
        }
        if (i != j)
        {
            std::swap(W[i], W[j]);
            if (temp_v.ptr<double>())
            {
                for (k = 0; k < m; k++)
                    std::swap(temp_a.ptr<double>()[i * astep + k], temp_a.ptr<double>()[j * astep + k]);

                for (k = 0; k < n; k++)
                    std::swap(temp_v.ptr<double>()[i * vstep + k], temp_v.ptr<double>()[j * vstep + k]);
            }
        }
    }

    for (i = 0; i < n; i++)
        temp_w.ptr<double>()[i] = (double)W[i];

    if (!temp_v.ptr<double>())
        return;

    RNG rng(0x12345678);
    for (i = 0; i < n; i++)
    {
        sd = i < n ? W[i] : 0;

        for (int ii = 0; ii < 100 && sd <= minval; ii++)
        {
            // if we got a zero singular value, then in order to get the corresponding left singular vector
            // we generate a random vector, project it to the previously computed left singular vectors,
            // subtract the projection and normalize the difference.
            const double val0 = (double)(1. / m);
            for (k = 0; k < m; k++)
            {
                double val = (rng.next() & 256) != 0 ? val0 : -val0;
                temp_a.ptr<double>()[i * astep + k] = val;
            }
            for (iter = 0; iter < 2; iter++)
            {
                for (j = 0; j < i; j++)
                {
                    sd = 0;
                    for (k = 0; k < m; k++)
                        sd += temp_a.ptr<double>()[i * astep + k] * temp_a.ptr<double>()[j * astep + k];
                    double asum = 0;
                    for (k = 0; k < m; k++)
                    {
                        double t = (double)(temp_a.ptr<double>()[i * astep + k] - sd * temp_a.ptr<double>()[j * astep + k]);
                        temp_a.ptr<double>()[i * astep + k] = t;
                        asum += std::abs(t);
                    }
                    asum = asum > eps * 100 ? 1 / asum : 0;
                    for (k = 0; k < m; k++)
                        temp_a.ptr<double>()[i * astep + k] *= asum;
                }
            }
            sd = 0;
            for (k = 0; k < m; k++)
            {
                double t = temp_a.ptr<double>()[i * astep + k];
                sd += (double)t * t;
            }
            sd = std::sqrt(sd);
        }

        s = (double)(sd > minval ? 1 / sd : 0.);
        for (k = 0; k < m; k++)
            temp_a.ptr<double>()[i * astep + k] *= s;
    }

    temp_w.copyTo(_w);
    temp_v.copyTo(_vt);
    transpose(temp_u, _u);
}

int EstimateMotion::cvRodrigues2(const CvMat *src, CvMat *dst, CvMat *jacobian)
{
    double J[27] = {0};
    CvMat matJ = cvMat(3, 9, CV_64F, J);

    int elem_size = CV_ELEM_SIZE(CV_MAT_DEPTH(src->type)); // = 8

    if (src->cols == 1 || src->rows == 1)
    {
        int step = src->rows > 1 ? src->step / elem_size : 1;

        Point3d r;

        r.x = src->data.db[0];
        r.y = src->data.db[step];
        r.z = src->data.db[step * 2];

        double theta = sqrt((double)r.x * r.x + (double)r.y * r.y + (double)r.z * r.z);

        double c = cos(theta);
        double s = sin(theta);
        double c1 = 1. - c;
        double itheta = theta ? 1. / theta : 0.;

        r *= itheta;

        Matx33d rrt(r.x * r.x, r.x * r.y, r.x * r.z,
                    r.x * r.y, r.y * r.y, r.y * r.z,
                    r.x * r.z, r.y * r.z, r.z * r.z); // r cross r
        Matx33d r_x(0, -r.z, r.y,
                    r.z, 0, -r.x,
                    -r.y, r.x, 0);

        Matx33d R = c * Matx33d::eye() + c1 * rrt + s * r_x;

        Mat(R).convertTo(cvarrToMat(dst), dst->type);

        if (jacobian)
        {
            const double I[] = {1, 0, 0,
                                0, 1, 0,
                                0, 0, 1};
            double drrt[] = {r.x + r.x, r.y, r.z, r.y, 0, 0, r.z, 0, 0,
                             0, r.x, 0, r.x, r.y + r.y, r.z, 0, r.z, 0,
                             0, 0, r.x, 0, 0, r.y, r.x, r.y, r.z + r.z};
            double d_r_x_[] = {0, 0, 0, 0, 0, -1, 0, 1, 0,
                               0, 0, 1, 0, 0, 0, -1, 0, 0,
                               0, -1, 0, 1, 0, 0, 0, 0, 0};
            for (int i = 0; i < 3; i++)
            {
                double ri = i == 0 ? r.x : i == 1 ? r.y
                                                  : r.z;
                double a0 = -s * ri, a1 = (s - 2 * c1 * itheta) * ri, a2 = c1 * itheta;
                double a3 = (c - s * itheta) * ri, a4 = s * itheta;
                for (int k = 0; k < 9; k++)
                    J[i * 9 + k] = a0 * I[k] + a1 * rrt.val[k] + a2 * drrt[i * 9 + k] +
                                   a3 * r_x.val[k] + a4 * d_r_x_[i * 9 + k];
            }
        }
    }
    else if (src->cols == 3 && src->rows == 3)
    {
        Matx33d U, Vt;
        Vec3d W;
        double theta, s, c;
        int step = dst->rows > 1 ? dst->step / elem_size : 1;

        Matx33d R = cvarrToMat(src);

        _SVDcompute(Mat(R), W, U, Vt);

        R = U * Vt;

        Point3d r(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));

        s = std::sqrt((r.x * r.x + r.y * r.y + r.z * r.z) * 0.25);
        c = (R(0, 0) + R(1, 1) + R(2, 2) - 1) * 0.5;
        c = c > 1. ? 1. : c < -1. ? -1.
                                  : c;
        theta = acos(c);

        double vth = 1 / (2 * s);

        if (jacobian)
        {
            double t, dtheta_dtr = -1. / s;
            double dvth_dtheta = -vth * c / s;
            double d1 = 0.5 * dvth_dtheta * dtheta_dtr;
            double d2 = 0.5 * dtheta_dtr;
            double dvardR[5 * 9] =
                {
                    0, 0, 0, 0, 0, 1, 0, -1, 0,
                    0, 0, -1, 0, 0, 0, 1, 0, 0,
                    0, 1, 0, -1, 0, 0, 0, 0, 0,
                    d1, 0, 0, 0, d1, 0, 0, 0, d1,
                    d2, 0, 0, 0, d2, 0, 0, 0, d2};
            double dvar2dvar[] =
                {
                    vth, 0, 0, r.x, 0,
                    0, vth, 0, r.y, 0,
                    0, 0, vth, r.z, 0,
                    0, 0, 0, 0, 1};
            double domegadvar2[] =
                {
                    theta, 0, 0, r.x * vth,
                    0, theta, 0, r.y * vth,
                    0, 0, theta, r.z * vth};

            CvMat _dvardR = cvMat(5, 9, CV_64FC1, dvardR);
            CvMat _dvar2dvar = cvMat(4, 5, CV_64FC1, dvar2dvar);
            CvMat _domegadvar2 = cvMat(3, 4, CV_64FC1, domegadvar2);
            double t0[3 * 5];
            CvMat _t0 = cvMat(3, 5, CV_64FC1, t0);

            cvMatMul(&_domegadvar2, &_dvar2dvar, &_t0);
            cvMatMul(&_t0, &_dvardR, &matJ);

            // transpose every row of matJ (treat the rows as 3x3 matrices)
            CV_SWAP(J[1], J[3], t);
            CV_SWAP(J[2], J[6], t);
            CV_SWAP(J[5], J[7], t);
            CV_SWAP(J[10], J[12], t);
            CV_SWAP(J[11], J[15], t);
            CV_SWAP(J[14], J[16], t);
            CV_SWAP(J[19], J[21], t);
            CV_SWAP(J[20], J[24], t);
            CV_SWAP(J[23], J[25], t);
        }

        vth *= theta;
        r *= vth;

        dst->data.db[0] = r.x;
        dst->data.db[step] = r.y;
        dst->data.db[step * 2] = r.z;
    }

    if (jacobian)
    {
        cvCopy(&matJ, jacobian);
    }

    return 1;
}

void EstimateMotion::cvProjectPoints2(const CvMat *objectPoints,
                                      const CvMat *r_vec,
                                      const CvMat *t_vec,
                                      const CvMat *A,
                                      const CvMat *distCoeffs,
                                      CvMat *imagePoints, CvMat *dpdr,
                                      CvMat *dpdt)
{
    Ptr<CvMat> matM, _m;
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

    int total = objectPoints->rows * objectPoints->cols * CV_MAT_CN(objectPoints->type); // 3

    count = total / 3;

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

using namespace std;
namespace cv
{

    epnp::epnp(const Mat &cameraMatrix, const Mat &opoints, const Mat &ipoints)
    {
        init_camera_parameters<double>(cameraMatrix);

        number_of_correspondences = opoints.checkVector(3, CV_32F);

        pws.resize(3 * number_of_correspondences);
        us.resize(2 * number_of_correspondences);

        init_points<Point3f, Point2f>(opoints, ipoints);

        alphas.resize(4 * number_of_correspondences);
        pcs.resize(3 * number_of_correspondences);

        max_nr = 0;
        A1 = NULL;
        A2 = NULL;
    }

    epnp::~epnp()
    {
        if (A1)
            delete[] A1;
        if (A2)
            delete[] A2;
    }

    void epnp::compute_pose(Mat &R, Mat &t)
    {
        // choose_control_points();
        //  Take C0 as the reference points centroid:
        cws[0][0] = cws[0][1] = cws[0][2] = 0;
        for (int i = 0; i < number_of_correspondences; i++)
            for (int j = 0; j < 3; j++)
                cws[0][j] += pws[3 * i + j];

        for (int j = 0; j < 3; j++)
            cws[0][j] /= number_of_correspondences;

        // Take C1, C2, and C3 from PCA on the reference points:
        CvMat *PW0 = cvCreateMat(number_of_correspondences, 3, CV_64F);

        double pw0tpw0[3 * 3] = {}, dc[3] = {}, uct[3 * 3] = {};
        CvMat PW0tPW0 = cvMat(3, 3, CV_64F, pw0tpw0);
        CvMat DC = cvMat(3, 1, CV_64F, dc);
        CvMat UCt = cvMat(3, 3, CV_64F, uct);

        for (int i = 0; i < number_of_correspondences; i++)
            for (int j = 0; j < 3; j++)
                PW0->data.db[3 * i + j] = pws[3 * i + j] - cws[0][j];

        cvMulTransposed(PW0, &PW0tPW0, 1);
        cvSVD(&PW0tPW0, &DC, &UCt, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);

        cvReleaseMat(&PW0);

        for (int i = 1; i < 4; i++)
        {
            double k = sqrt(dc[i - 1] / number_of_correspondences);
            for (int j = 0; j < 3; j++)
                cws[i][j] = cws[0][j] + k * uct[3 * (i - 1) + j];
        }

        // compute_barycentric_coordinates();
        double cc[3 * 3] = {}, cc_inv[3 * 3] = {};
        CvMat CC = cvMat(3, 3, CV_64F, cc);
        CvMat CC_inv = cvMat(3, 3, CV_64F, cc_inv);

        for (int i = 0; i < 3; i++)
            for (int j = 1; j < 4; j++)
                cc[3 * i + j - 1] = cws[j][i] - cws[0][i];

        cvInvert(&CC, &CC_inv, CV_SVD);
        double *ci = cc_inv;
        for (int i = 0; i < number_of_correspondences; i++)
        {
            double *pi = &pws[0] + 3 * i;
            double *a = &alphas[0] + 4 * i;

            for (int j = 0; j < 3; j++)
            {
                a[1 + j] =
                    ci[3 * j] * (pi[0] - cws[0][0]) +
                    ci[3 * j + 1] * (pi[1] - cws[0][1]) +
                    ci[3 * j + 2] * (pi[2] - cws[0][2]);
            }
            a[0] = 1.0f - a[1] - a[2] - a[3];
        }

        CvMat *M = cvCreateMat(2 * number_of_correspondences, 12, CV_64F);

        for (int i = 0; i < number_of_correspondences; i++)
        {
            // fill_M(M, 2 * i, &alphas[0] + 4 * i, us[2 * i], us[2 * i + 1]);
            double *M1 = M->data.db + (2 * i) * 12;
            double *M2 = M1 + 12;

            for (int j = 0; j < 4; j++)
            {
                M1[3 * j] = (&alphas[0] + 4 * i)[j] * fu;
                M1[3 * j + 1] = 0.0;
                M1[3 * j + 2] = (&alphas[0] + 4 * i)[j] * (uc - us[2 * i]);

                M2[3 * j] = 0.0;
                M2[3 * j + 1] = (&alphas[0] + 4 * i)[j] * fv;
                M2[3 * j + 2] = (&alphas[0] + 4 * i)[j] * (vc - us[2 * i + 1]);
            }
        }

        double mtm[12 * 12] = {}, d[12] = {}, ut[12 * 12] = {};
        CvMat MtM = cvMat(12, 12, CV_64F, mtm);
        CvMat D = cvMat(12, 1, CV_64F, d);
        CvMat Ut = cvMat(12, 12, CV_64F, ut);

        cvMulTransposed(M, &MtM, 1);
        cvSVD(&MtM, &D, &Ut, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);
        cvReleaseMat(&M);

        double l_6x10[6 * 10] = {}, rho[6] = {};
        CvMat L_6x10 = cvMat(6, 10, CV_64F, l_6x10);
        CvMat Rho = cvMat(6, 1, CV_64F, rho);

        // compute_L_6x10(ut, l_6x10);
        const double *v[4];

        v[0] = ut + 12 * 11;
        v[1] = ut + 12 * 10;
        v[2] = ut + 12 * 9;
        v[3] = ut + 12 * 8;

        double dv[4][6][3] = {};

        for (int i = 0; i < 4; i++)
        {
            int a = 0, b = 1;
            for (int j = 0; j < 6; j++)
            {
                dv[i][j][0] = v[i][3 * a] - v[i][3 * b];
                dv[i][j][1] = v[i][3 * a + 1] - v[i][3 * b + 1];
                dv[i][j][2] = v[i][3 * a + 2] - v[i][3 * b + 2];

                b++;
                if (b > 3)
                {
                    a++;
                    b = a + 1;
                }
            }
        }

        for (int i = 0; i < 6; i++)
        {
            double *row = l_6x10 + 10 * i;

            row[0] = dot(dv[0][i], dv[0][i]);
            row[1] = 2.0f * dot(dv[0][i], dv[1][i]);
            row[2] = dot(dv[1][i], dv[1][i]);
            row[3] = 2.0f * dot(dv[0][i], dv[2][i]);
            row[4] = 2.0f * dot(dv[1][i], dv[2][i]);
            row[5] = dot(dv[2][i], dv[2][i]);
            row[6] = 2.0f * dot(dv[0][i], dv[3][i]);
            row[7] = 2.0f * dot(dv[1][i], dv[3][i]);
            row[8] = 2.0f * dot(dv[2][i], dv[3][i]);
            row[9] = dot(dv[3][i], dv[3][i]);
        }

        // compute_rho(rho);
        rho[0] = (cws[0][0] - cws[1][0]) * (cws[0][0] - cws[1][0]) +
                 (cws[0][1] - cws[1][1]) * (cws[0][1] - cws[1][1]) +
                 (cws[0][2] - cws[1][2]) * (cws[0][2] - cws[1][2]);
        rho[1] = (cws[0][0] - cws[2][0]) * (cws[0][0] - cws[2][0]) +
                 (cws[0][1] - cws[2][1]) * (cws[0][1] - cws[2][1]) +
                 (cws[0][2] - cws[2][2]) * (cws[0][2] - cws[2][2]);
        rho[2] = (cws[0][0] - cws[3][0]) * (cws[0][0] - cws[3][0]) +
                 (cws[0][1] - cws[3][1]) * (cws[0][1] - cws[3][1]) +
                 (cws[0][2] - cws[3][2]) * (cws[0][2] - cws[3][2]);
        rho[3] = (cws[1][0] - cws[2][0]) * (cws[1][0] - cws[2][0]) +
                 (cws[1][1] - cws[2][1]) * (cws[1][1] - cws[2][1]) +
                 (cws[1][2] - cws[2][2]) * (cws[1][2] - cws[2][2]);
        rho[4] = (cws[1][0] - cws[3][0]) * (cws[1][0] - cws[3][0]) +
                 (cws[1][1] - cws[3][1]) * (cws[1][1] - cws[3][1]) +
                 (cws[1][2] - cws[3][2]) * (cws[1][2] - cws[3][2]);
        rho[5] = (cws[2][0] - cws[3][0]) * (cws[2][0] - cws[3][0]) +
                 (cws[2][1] - cws[3][1]) * (cws[2][1] - cws[3][1]) +
                 (cws[2][2] - cws[3][2]) * (cws[2][2] - cws[3][2]);

        double Betas[4][4] = {}, rep_errors[4] = {};
        double Rs[4][3][3] = {}, ts[4][3] = {};

        // find_betas_approx_1(&L_6x10, &Rho, Betas[1]);
        double l_6x4[6 * 4] = {}, b4[4] = {};
        CvMat L_6x4 = cvMat(6, 4, CV_64F, l_6x4);
        CvMat B4 = cvMat(4, 1, CV_64F, b4);

        for (int i = 0; i < 6; i++)
        {
            cvmSet(&L_6x4, i, 0, cvmGet(&L_6x10, i, 0));
            cvmSet(&L_6x4, i, 1, cvmGet(&L_6x10, i, 1));
            cvmSet(&L_6x4, i, 2, cvmGet(&L_6x10, i, 3));
            cvmSet(&L_6x4, i, 3, cvmGet(&L_6x10, i, 6));
        }

        cvSolve(&L_6x4, &Rho, &B4, CV_SVD); ////////////////////////////////////

        if (b4[0] < 0)
        {
            Betas[1][0] = sqrt(-b4[0]);
            Betas[1][1] = -b4[1] / Betas[1][0];
            Betas[1][2] = -b4[2] / Betas[1][0];
            Betas[1][3] = -b4[3] / Betas[1][0];
        }
        else
        {
            Betas[1][0] = sqrt(b4[0]);
            Betas[1][1] = b4[1] / Betas[1][0];
            Betas[1][2] = b4[2] / Betas[1][0];
            Betas[1][3] = b4[3] / Betas[1][0];
        }

        gauss_newton(&L_6x10, &Rho, Betas[1]);
        rep_errors[1] = compute_R_and_t(ut, Betas[1], Rs[1], ts[1]);

        // find_betas_approx_2(&L_6x10, &Rho, Betas[2]);
        double l_6x3[6 * 3] = {}, b3[3] = {};
        CvMat L_6x3 = cvMat(6, 3, CV_64F, l_6x3);
        CvMat B3 = cvMat(3, 1, CV_64F, b3);

        for (int i = 0; i < 6; i++)
        {
            cvmSet(&L_6x3, i, 0, cvmGet(&L_6x10, i, 0));
            cvmSet(&L_6x3, i, 1, cvmGet(&L_6x10, i, 1));
            cvmSet(&L_6x3, i, 2, cvmGet(&L_6x10, i, 2));
        }

        cvSolve(&L_6x3, &Rho, &B3, CV_SVD);

        if (b3[0] < 0)
        {
            Betas[2][0] = sqrt(-b3[0]);
            Betas[2][1] = (b3[2] < 0) ? sqrt(-b3[2]) : 0.0;
        }
        else
        {
            Betas[2][0] = sqrt(b3[0]);
            Betas[2][1] = (b3[2] > 0) ? sqrt(b3[2]) : 0.0;
        }

        if (b3[1] < 0)
            Betas[2][0] = -Betas[2][0];

        Betas[2][2] = 0.0;
        Betas[2][3] = 0.0;
        gauss_newton(&L_6x10, &Rho, Betas[2]);
        rep_errors[2] = compute_R_and_t(ut, Betas[2], Rs[2], ts[2]);

        // find_betas_approx_3(&L_6x10, &Rho, Betas[3]);
        double l_6x5[6 * 5] = {}, b5[5] = {};
        CvMat L_6x5 = cvMat(6, 5, CV_64F, l_6x5);
        CvMat B5 = cvMat(5, 1, CV_64F, b5);

        for (int i = 0; i < 6; i++)
        {
            cvmSet(&L_6x5, i, 0, cvmGet(&L_6x10, i, 0));
            cvmSet(&L_6x5, i, 1, cvmGet(&L_6x10, i, 1));
            cvmSet(&L_6x5, i, 2, cvmGet(&L_6x10, i, 2));
            cvmSet(&L_6x5, i, 3, cvmGet(&L_6x10, i, 3));
            cvmSet(&L_6x5, i, 4, cvmGet(&L_6x10, i, 4));
        }

        cvSolve(&L_6x5, &Rho, &B5, CV_SVD);

        if (b5[0] < 0)
        {
            Betas[3][0] = sqrt(-b5[0]);
            Betas[3][1] = (b5[2] < 0) ? sqrt(-b5[2]) : 0.0;
        }
        else
        {
            Betas[3][0] = sqrt(b5[0]);
            Betas[3][1] = (b5[2] > 0) ? sqrt(b5[2]) : 0.0;
        }
        if (b5[1] < 0)
            Betas[3][0] = -Betas[3][0];
        Betas[3][2] = b5[3] / Betas[3][0];
        Betas[3][3] = 0.0;

        gauss_newton(&L_6x10, &Rho, Betas[3]);
        rep_errors[3] = compute_R_and_t(ut, Betas[3], Rs[3], ts[3]);

        int N = 1;
        if (rep_errors[2] < rep_errors[1])
            N = 2;
        if (rep_errors[3] < rep_errors[N])
            N = 3;

        Mat(3, 1, CV_64F, ts[N]).copyTo(t);
        Mat(3, 3, CV_64F, Rs[N]).copyTo(R);
    }

    double epnp::dot(const double *v1, const double *v2)
    {
        return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
    }

    double epnp::compute_R_and_t(const double *ut, const double *betas,
                                 double R[3][3], double t[3])
    {
        for (int i = 0; i < 4; i++)
            ccs[i][0] = ccs[i][1] = ccs[i][2] = 0.0f;

        for (int i = 0; i < 4; i++)
        {
            const double *v = ut + 12 * (11 - i);
            for (int j = 0; j < 4; j++)
                for (int k = 0; k < 3; k++)
                    ccs[j][k] += betas[i] * v[3 * j + k];
        }

        for (int i = 0; i < number_of_correspondences; i++)
        {
            double *a = &alphas[0] + 4 * i;
            double *pc = &pcs[0] + 3 * i;

            for (int j = 0; j < 3; j++)
                pc[j] = a[0] * ccs[0][j] + a[1] * ccs[1][j] + a[2] * ccs[2][j] + a[3] * ccs[3][j];
        }

        if (pcs[2] < 0.0)
        {
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 3; j++)
                    ccs[i][j] = -ccs[i][j];

            for (int i = 0; i < number_of_correspondences; i++)
            {
                pcs[3 * i] = -pcs[3 * i];
                pcs[3 * i + 1] = -pcs[3 * i + 1];
                pcs[3 * i + 2] = -pcs[3 * i + 2];
            }
        }

        // estimate_R_and_t(R, t);
        double pc0[3] = {}, pw0[3] = {};

        pc0[0] = pc0[1] = pc0[2] = 0.0;
        pw0[0] = pw0[1] = pw0[2] = 0.0;

        for (int i = 0; i < number_of_correspondences; i++)
        {
            const double *pc = &pcs[3 * i];
            const double *pw = &pws[3 * i];

            for (int j = 0; j < 3; j++)
            {
                pc0[j] += pc[j];
                pw0[j] += pw[j];
            }
        }
        for (int j = 0; j < 3; j++)
        {
            pc0[j] /= number_of_correspondences;
            pw0[j] /= number_of_correspondences;
        }

        double abt[3 * 3] = {}, abt_d[3] = {}, abt_u[3 * 3] = {}, abt_v[3 * 3] = {};
        CvMat ABt = cvMat(3, 3, CV_64F, abt);
        CvMat ABt_D = cvMat(3, 1, CV_64F, abt_d);
        CvMat ABt_U = cvMat(3, 3, CV_64F, abt_u);
        CvMat ABt_V = cvMat(3, 3, CV_64F, abt_v);

        cvSetZero(&ABt);
        for (int i = 0; i < number_of_correspondences; i++)
        {
            double *pc = &pcs[3 * i];
            double *pw = &pws[3 * i];

            for (int j = 0; j < 3; j++)
            {
                abt[3 * j] += (pc[j] - pc0[j]) * (pw[0] - pw0[0]);
                abt[3 * j + 1] += (pc[j] - pc0[j]) * (pw[1] - pw0[1]);
                abt[3 * j + 2] += (pc[j] - pc0[j]) * (pw[2] - pw0[2]);
            }
        }

        cvSVD(&ABt, &ABt_D, &ABt_U, &ABt_V, CV_SVD_MODIFY_A);

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                R[i][j] = dot(abt_u + 3 * i, abt_v + 3 * j);

        const double det =
            R[0][0] * R[1][1] * R[2][2] + R[0][1] * R[1][2] * R[2][0] + R[0][2] * R[1][0] * R[2][1] -
            R[0][2] * R[1][1] * R[2][0] - R[0][1] * R[1][0] * R[2][2] - R[0][0] * R[1][2] * R[2][1];

        if (det < 0)
        {
            R[2][0] = -R[2][0];
            R[2][1] = -R[2][1];
            R[2][2] = -R[2][2];
        }

        t[0] = pc0[0] - dot(R[0], pw0);
        t[1] = pc0[1] - dot(R[1], pw0);
        t[2] = pc0[2] - dot(R[2], pw0);

        double sum2 = 0.0;

        for (int i = 0; i < number_of_correspondences; i++)
        {
            double *pw = &pws[3 * i];
            double Xc = dot(R[0], pw) + t[0];
            double Yc = dot(R[1], pw) + t[1];
            double inv_Zc = 1.0 / (dot(R[2], pw) + t[2]);
            double ue = uc + fu * Xc * inv_Zc;
            double ve = vc + fv * Yc * inv_Zc;
            double u = us[2 * i], v = us[2 * i + 1];

            sum2 += sqrt((u - ue) * (u - ue) + (v - ve) * (v - ve));
        }

        return sum2 / number_of_correspondences;
    }

    void epnp::gauss_newton(const CvMat *L_6x10, const CvMat *Rho, double betas[4])
    {
        const int iterations_number = 5;

        double a[6 * 4] = {}, b[6] = {}, x[4] = {};
        CvMat A = cvMat(6, 4, CV_64F, a);
        CvMat B = cvMat(6, 1, CV_64F, b);
        CvMat X = cvMat(4, 1, CV_64F, x);

        for (int k = 0; k < iterations_number; k++)
        {
            // compute_A_and_b_gauss_newton(L_6x10->data.db, Rho->data.db,
            // betas, &A, &B);
            for (int i = 0; i < 6; i++)
            {
                const double *rowL = (L_6x10->data.db) + i * 10;
                double *rowA = A.data.db + i * 4;

                rowA[0] = 2 * rowL[0] * betas[0] + rowL[1] * betas[1] + rowL[3] * betas[2] + rowL[6] * betas[3];
                rowA[1] = rowL[1] * betas[0] + 2 * rowL[2] * betas[1] + rowL[4] * betas[2] + rowL[7] * betas[3];
                rowA[2] = rowL[3] * betas[0] + rowL[4] * betas[1] + 2 * rowL[5] * betas[2] + rowL[8] * betas[3];
                rowA[3] = rowL[6] * betas[0] + rowL[7] * betas[1] + rowL[8] * betas[2] + 2 * rowL[9] * betas[3];

                cvmSet(&B, i, 0, (Rho->data.db)[i] - (rowL[0] * betas[0] * betas[0] + rowL[1] * betas[0] * betas[1] + rowL[2] * betas[1] * betas[1] + rowL[3] * betas[0] * betas[2] + rowL[4] * betas[1] * betas[2] + rowL[5] * betas[2] * betas[2] + rowL[6] * betas[0] * betas[3] + rowL[7] * betas[1] * betas[3] + rowL[8] * betas[2] * betas[3] + rowL[9] * betas[3] * betas[3]));
            }

            // qr_solve(&A, &B, &X);
            const int nr = A.rows;
            const int nc = A.cols;
            if (nc <= 0 || nr <= 0)
                return;

            if (max_nr != 0 && max_nr < nr)
            {
                delete[] A1;
                delete[] A2;
            }
            if (max_nr < nr)
            {
                max_nr = nr;
                A1 = new double[nr];
                A2 = new double[nr];
            }

            double *pA = A.data.db, *ppAkk = pA;
            for (int k = 0; k < nc; k++)
            {
                double *ppAik1 = ppAkk, eta = fabs(*ppAik1);
                for (int i = k + 1; i < nr; i++)
                {
                    double elt = fabs(*ppAik1);
                    if (eta < elt)
                        eta = elt;
                    ppAik1 += nc;
                }
                if (eta == 0)
                {
                    A1[k] = A2[k] = 0.0;
                    return;
                }
                else
                {
                    double *ppAik2 = ppAkk, sum2 = 0.0, inv_eta = 1. / eta;
                    for (int i = k; i < nr; i++)
                    {
                        *ppAik2 *= inv_eta;
                        sum2 += *ppAik2 * *ppAik2;
                        ppAik2 += nc;
                    }
                    double sigma = sqrt(sum2);
                    if (*ppAkk < 0)
                        sigma = -sigma;
                    *ppAkk += sigma;
                    A1[k] = sigma * *ppAkk;
                    A2[k] = -eta * sigma;
                    for (int j = k + 1; j < nc; j++)
                    {
                        double *ppAik = ppAkk, sum = 0;
                        for (int i = k; i < nr; i++)
                        {
                            sum += *ppAik * ppAik[j - k];
                            ppAik += nc;
                        }
                        double tau = sum / A1[k];
                        ppAik = ppAkk;
                        for (int i = k; i < nr; i++)
                        {
                            ppAik[j - k] -= tau * *ppAik;
                            ppAik += nc;
                        }
                    }
                }
                ppAkk += nc + 1;
            }

            // b <- Qt b
            double *ppAjj = pA, *pb = B.data.db;
            for (int j = 0; j < nc; j++)
            {
                double *ppAij = ppAjj, tau = 0;
                for (int i = j; i < nr; i++)
                {
                    tau += *ppAij * pb[i];
                    ppAij += nc;
                }
                tau /= A1[j];
                ppAij = ppAjj;
                for (int i = j; i < nr; i++)
                {
                    pb[i] -= tau * *ppAij;
                    ppAij += nc;
                }
                ppAjj += nc + 1;
            }

            // X = R-1 b
            double *pX = X.data.db;
            pX[nc - 1] = pb[nc - 1] / A2[nc - 1];
            for (int i = nc - 2; i >= 0; i--)
            {
                double *ppAij = pA + i * nc + (i + 1), sum = 0;

                for (int j = i + 1; j < nc; j++)
                {
                    sum += *ppAij * pX[j];
                    ppAij++;
                }
                pX[i] = (pb[i] - sum) / A2[i];
            }

            for (int i = 0; i < 4; i++)
                betas[i] += x[i];
        }
    }
}