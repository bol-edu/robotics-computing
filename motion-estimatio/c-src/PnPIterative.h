#include "Matrix.h"

#define FLT_EPSILON 1.192092896e-07F // smallest such that 1.0+FLT_EPSILON != 1.0

class PnPIterative
{
public:
    PnPIterative(Matrix &opoints, Matrix &ipoints, FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy);
    void compute(Matrix &rmat, Matrix &tvec);

private:
    int32_t number_of_points;
    Matrix _m;
    Matrix matM;
    FLOAT cx;
    FLOAT cy;
    FLOAT fx;
    FLOAT fy;
};

class LevMarq
{
public:
    // LevMarq();
    LevMarq(int32_t nparams, int32_t nerrs);
    ~LevMarq();
    bool update(Matrix &param, Matrix &J, Matrix &err);

    void clear();
    void step();
    enum
    {
        DONE = 0,
        STARTED = 1,
        CALC_J = 2,
        CHECK_ERR = 3
    };

    Matrix prevParam;
    Matrix param;
    Matrix J;
    Matrix err;
    Matrix JtJ;
    Matrix JtJN;
    Matrix JtErr;
    Matrix JtJV;
    Matrix JtJW;
    FLOAT prevErrNorm, errNorm;
    FLOAT epsilon;
    int32_t lambdaLg10;
    int32_t max_iter;
    int32_t state;
    int32_t iters;
    bool is_matJ;
    bool is_err;
};
