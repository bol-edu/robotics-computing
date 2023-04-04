#ifndef __EM_H__
#define __EM_H__

#include "EPnP.h"
//#include "PnPIterative.h"

#define MAX_DEPTH 3000
#define MODEL_POINTS 5
#define CONFIDENCE 0.99
#define THRESHOLD 8
#define MAXITER 1000
#define RNG_COEF 4164903690ULL

static unsigned long long RNG_state;

void estimate_motion(volatile int* match, int match_num,
					 volatile FLOAT* kp0, volatile FLOAT* kp1,
					 FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy,
					 volatile FLOAT* depth,
                     volatile FLOAT *rmat, volatile FLOAT *tvec);

void match_points(volatile int* match, int match_num,
					volatile FLOAT* kp0, volatile FLOAT* kp1,
					FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy,
					volatile FLOAT* depth, unsigned int &point_num,
					OPOINT opoint[MAX_KEYPOINT_NUM],
					IPOINT ipoint[MAX_KEYPOINT_NUM]);

void RANSAC_PnP(OPOINT opoint[MAX_KEYPOINT_NUM],
				IPOINT ipoint[MAX_KEYPOINT_NUM],
				unsigned int point_num, FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy,
                volatile FLOAT *rmat, volatile FLOAT *tvec);

void generate_5_indices(unsigned int point_num, unsigned int idx[MODEL_POINTS]);

unsigned int RNG(unsigned int a);


#endif
