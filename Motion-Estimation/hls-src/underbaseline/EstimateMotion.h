#ifndef __EM_H__
#define __EM_H__

// #include <ap_axi_sdata.h>
#include "EPnP.h"
#include "PnPIterative.h"

#define MAX_DEPTH 3000
#define MODEL_POINTS 5
#define CONFIDENCE 0.99
#define THRESHOLD 8
#define MAXITER 1000
#define RNG_COEF 4164903690ULL

// typedef ap_uint<64> uint64;
// typedef ap_int<64> int64;

void estimate_motion(volatile int32_t *match_val, unsigned int match_m,
                     volatile FLOAT *kp0_val, unsigned int kp0_m,
                     volatile FLOAT *kp1_val, unsigned int kp1_m,
                     volatile FLOAT *k, volatile FLOAT *depth_val,
                     volatile FLOAT *rmat, volatile FLOAT *tvec);

class EstimateMotion
{
private:
    // uint64
    unsigned long long RNG_state;
    FLOAT cx;
    FLOAT cy;
    FLOAT fx;
    FLOAT fy;

    Matrix<FLOAT, MAX_KEYPOINT_NUM, 3> opoint;
    Matrix<FLOAT, MAX_KEYPOINT_NUM, 2> ipoint;

    void RANSAC_PnP();

    void getSubset();
    Matrix<FLOAT, MODEL_POINTS, 3> subopoint;
    Matrix<FLOAT, MODEL_POINTS, 2> subipoint;

    unsigned int RNG_uniform(unsigned int a);

    Matrix<FLOAT, MAX_KEYPOINT_NUM, 2> projectPoint(const Matrix<FLOAT, 3, 3> rmat,
                                                    const Matrix<FLOAT, 3, 1> tvec);
    Matrix<FLOAT, MAX_KEYPOINT_NUM, 2> projPoint;

    Matrix<FLOAT, MAX_KEYPOINT_NUM, 1> err;

    Matrix<FLOAT, MAX_KEYPOINT_NUM, 3> opoint_inlier;
    Matrix<FLOAT, MAX_KEYPOINT_NUM, 2> ipoint_inlier;

public:
    Matrix<FLOAT, 3, 3> rmat;
    Matrix<FLOAT, 3, 1> tvec;

    EstimateMotion();
    void estimate(volatile int32_t *match_val, unsigned int match_m,
                  volatile FLOAT *kp0_val, unsigned int kp0_m,
                  volatile FLOAT *kp1_val, unsigned int kp1_m,
                  volatile FLOAT *k, volatile FLOAT *depth);
};

#endif
