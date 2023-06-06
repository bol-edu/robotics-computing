#ifndef __EPNP_H__
#define __EPNP_H__

#include "Matrix.h"

#define MODEL_POINTS 5

void gauss_newton(FLOAT L_6x10[6][10], FLOAT Rho[6], FLOAT betas[4]);

void epnp(OPOINT opoint[MODEL_POINTS],
          IPOINT ipoint[MODEL_POINTS],
          FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy,
          volatile FLOAT *rmat, volatile FLOAT *tvec);

#endif
