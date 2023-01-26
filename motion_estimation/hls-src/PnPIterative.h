#include "Matrix.h"

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