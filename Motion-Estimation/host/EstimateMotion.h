#include "helper.h"
#include <hls_math.h>
#include <float.h>

#include "EPnP.h"
#include "PnPIterative.h"

#define MAX_DEPTH 3000
#define MODEL_POINTS 5
#define CONFIDENCE 0.99f
#define THRESHOLD 64
#define MAXITER 1000
#define RNG_COEF 4164903690ULL

static unsigned long long RNG_state;

extern "C"{
void estimate_motion(MATCH* match, int match_num,
					 IPOINT* kp0, IPOINT* kp1,
					 FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy,
					 volatile FLOAT* depths,
		             FLOAT rmat[9], FLOAT tvec[3]);
}

void match_points(MATCH* match, int match_num,
				  IPOINT* kp0, IPOINT* kp1,
				  FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy,
				  volatile FLOAT* depth, int &point_num,
				  OPOINT opoint[MAX_KEYPOINT_NUM],
				  IPOINT ipoint[MAX_KEYPOINT_NUM]);

void RANSAC_PnP(OPOINT opoint[MAX_KEYPOINT_NUM],
				IPOINT ipoint[MAX_KEYPOINT_NUM],
				unsigned int point_num, FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy,
                FLOAT rmat[9], FLOAT tvec[3]);

void generate_5_indices(unsigned int point_num, unsigned int idx[MODEL_POINTS]);

unsigned int RNG(unsigned int a);

void projectPoint(OPOINT opoint[MAX_KEYPOINT_NUM], int point_num, int fx, int fy, int cx, int cy,
				  FLOAT rmat[9], FLOAT tvec[3], IPOINT projPoint[MAX_KEYPOINT_NUM]);

