#include "Matrix.h"
#include "EPnP.h"
#include "PnPIterative.h"
#include <stdio.h>

class EstimateMotion
{
private:
    int32_t max_depth;
    int32_t model_points;
    uint64_t RNG_state;
    FLOAT confidence;
    FLOAT threshold;
    int32_t maxIter;
    FLOAT cx;
    FLOAT cy;
    FLOAT fx;
    FLOAT fy;

    void RANSAC_EPnP(Matrix opoint, Matrix ipoint, Matrix &_rmat, Matrix &_tvec);
    void getSubset(Matrix opoint, Matrix ipoint, Matrix &subopoint, Matrix &subipoint);
    Matrix projectPoint(Matrix opoint, Matrix rmat, Matrix tvec);

    unsigned RNG_next();
    int64_t RNG_uniform(int64_t a, int64_t b);

public:
    EstimateMotion();
    void estimate(Matrix &match, Matrix &kp0, Matrix &kp1,
                  Matrix &k, Matrix &depth,
                  Matrix &rmat, Matrix &tvec);
    ~EstimateMotion();
};
