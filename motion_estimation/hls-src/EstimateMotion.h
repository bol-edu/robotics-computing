#include "Matrix.h"
#include "EPnP.h"
#include <stdio.h>

class EstimateMotion
{
private:
    int32_t max_depth;
    int32_t model_points;
    uint32_t RNG_COEF;
    uint32_t RNG_state;
    FLOAT confidence;
    FLOAT threshold;
    FLOAT maxIter;

    void RANSAC_EPnP(Matrix opoint, Matrix ipoint, Matrix k);
    void getSubset(Matrix &opoint, Matrix &ipoint, Matrix &subopoint, Matrix &subipoint);

public:
    EstimateMotion();
    void estimate(Matrix &match, Matrix &kp0, Matrix &kp1,
                  Matrix &k, Matrix &depth,
                  Matrix &rmat, Matrix &tvec);
    ~EstimateMotion();
};
