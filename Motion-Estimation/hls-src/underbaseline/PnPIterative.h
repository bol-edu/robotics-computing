#ifndef __PNPITER_H__
#define __PNPITER_H__

#include "Matrix.h"

#define NUM_OF_PARAMS 6
#define NUM_OF_ERRS (2 * MAX_KEYPOINT_NUM)

class PnPIterative
{
public:
    PnPIterative(Matrix<FLOAT, MAX_KEYPOINT_NUM, 3> &opoints,
                 Matrix<FLOAT, MAX_KEYPOINT_NUM, 2> &ipoints,
                 FLOAT _fx, FLOAT _fy, FLOAT _cx, FLOAT _cy);

    void compute(Matrix<FLOAT, 3, 3> &rmat,
                 Matrix<FLOAT, 3, 1> &tvec);

private:
    int32_t number_of_points;
    Matrix<FLOAT, MAX_KEYPOINT_NUM, 2> _m;
    Matrix<FLOAT, MAX_KEYPOINT_NUM, 3> matM;
    FLOAT cx;
    FLOAT cy;
    FLOAT fx;
    FLOAT fy;

    Matrix<FLOAT, MAX_KEYPOINT_NUM, 2> mn;
    Matrix<FLOAT, 2 * MAX_KEYPOINT_NUM, 12> matL;
    Matrix<FLOAT, 12, 12> LL;
    Matrix<FLOAT, 12, 1> LW;
    Matrix<FLOAT, 12, 12> LU;
    Matrix<FLOAT, 12, 12> LV;
    Matrix<FLOAT, 3, 3> RR;
    Matrix<FLOAT, 3, 1> tt;
    Matrix<FLOAT, 3, 1> matW;
    Matrix<FLOAT, 3, 3> matU;
    Matrix<FLOAT, 3, 3> matV;
    Matrix<FLOAT, 3, 3> matR;
    Matrix<FLOAT, 3, 1> t;
    Matrix<FLOAT, 3, 1> r;

    Matrix<FLOAT, NUM_OF_ERRS, NUM_OF_PARAMS> matJ;
    Matrix<FLOAT, MAX_KEYPOINT_NUM, 2> err;
    Matrix<FLOAT, NUM_OF_PARAMS, 1> param;
    Matrix<FLOAT, NUM_OF_ERRS, 3> dpdr;
    Matrix<FLOAT, NUM_OF_ERRS, 3> dpdt;
};

class LevMarq
{
public:
    LevMarq(int32_t nerrs);

    bool update(Matrix<FLOAT, NUM_OF_PARAMS, 1> &_param,
                Matrix<FLOAT, NUM_OF_ERRS, NUM_OF_PARAMS> &matJ);
    // Matrix<FLOAT, NUM_OF_ERRS, 1> &err);

    void step();

    enum
    {
        DONE = 0,
        STARTED = 1,
        CALC_J = 2,
        CHECK_ERR = 3
    };

    Matrix<FLOAT, NUM_OF_PARAMS, 1> prevParam;
    Matrix<FLOAT, NUM_OF_PARAMS, 1> param;
    Matrix<FLOAT, NUM_OF_ERRS, NUM_OF_PARAMS> J;
    Matrix<FLOAT, NUM_OF_ERRS, 1> err;
    Matrix<FLOAT, NUM_OF_PARAMS, NUM_OF_PARAMS> JtJ;
    Matrix<FLOAT, NUM_OF_PARAMS, NUM_OF_PARAMS> JtJN;
    Matrix<FLOAT, NUM_OF_PARAMS, 1> JtErr;
    Matrix<FLOAT, NUM_OF_PARAMS, 1> JtJV;
    Matrix<FLOAT, NUM_OF_PARAMS, 1> JtJW;
    FLOAT prevErrNorm, errNorm;
    FLOAT epsilon;
    int32_t lambdaLg10;
    int32_t max_iter;
    int32_t state;
    int32_t iters;
    bool is_matJ;
    bool is_err;
};

#endif
