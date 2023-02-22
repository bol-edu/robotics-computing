#ifndef __EM_H__
#define __EM_H__

#include <ap_axi_sdata.h>
#include "Matrix.h"
#include "EPnP.h"
#include "PnPIterative.h"

typedef ap_uint<64> uint64;
typedef ap_int<64> int64;

void estimate_motion(Matrix &match, Matrix &kp0, Matrix &kp1,
                     Matrix &k, Matrix &depth,
                     Matrix &rmat, Matrix &tvec);

class EstimateMotion
{
private:
    int32_t max_depth;
    int32_t maxIter;
    uint64 RNG_state;
    int32_t model_points;
    FLOAT confidence;
    FLOAT threshold;
    FLOAT cx;
    FLOAT cy;
    FLOAT fx;
    FLOAT fy;

    void RANSAC_EPnP(Matrix opoint, Matrix ipoint, Matrix &_rmat, Matrix &_tvec);
    void getSubset(Matrix opoint, Matrix ipoint, Matrix &subopoint, Matrix &subipoint);
    Matrix projectPoint(Matrix opoint, Matrix rmat, Matrix tvec);

    unsigned RNG_next();
    int64 RNG_uniform(int64 a, int64 b);

public:
    EstimateMotion();
    void estimate(Matrix &match, Matrix &kp0, Matrix &kp1,
                  Matrix &k, Matrix &depth,
                  Matrix &rmat, Matrix &tvec);
    ~EstimateMotion();
};

#endif
