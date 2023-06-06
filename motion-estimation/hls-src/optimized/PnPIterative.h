#include "Math.h"
#include "sym_svd.hpp"

#define NUM_OF_PARAMS 6
#define NUM_OF_ERRS (2 * MAX_KEYPOINT_NUM)

class LevMarq
{
public:
    LevMarq(int _nerrs);

    bool update(FLOAT _param[NUM_OF_PARAMS],
                FLOAT matJ[NUM_OF_ERRS][NUM_OF_PARAMS]);

    void step();

    enum
    {
        DONE = 0,
        STARTED = 1,
        CALC_J = 2,
        CHECK_ERR = 3
    };

    FLOAT prevParam[NUM_OF_PARAMS];
    FLOAT param[NUM_OF_PARAMS];
    FLOAT J[NUM_OF_ERRS][NUM_OF_PARAMS];
    FLOAT err[NUM_OF_ERRS];
    FLOAT JtJ[NUM_OF_PARAMS][NUM_OF_PARAMS];
    FLOAT JtJN[NUM_OF_PARAMS * NUM_OF_PARAMS];
    FLOAT JtErr[NUM_OF_PARAMS];
    FLOAT JtJV[NUM_OF_PARAMS];
    FLOAT JtJW[NUM_OF_PARAMS];
    FLOAT prevErrNorm, errNorm;
    FLOAT epsilon;
    int nerrs;
    int lambdaLg10;
    int max_iter;
    int state;
    int iters;
    bool is_matJ;
    bool is_err;
};

void Rodrigues_9_to_3(FLOAT src[9], FLOAT dst[3]);

void Rodrigues_3_to_9(FLOAT src[3], FLOAT dst[9]);

// jacob[3][9]
void Rodrigues(FLOAT src[3], FLOAT jacob[27], FLOAT dst[9]);

void ProjectPoints(OPOINT objectPoints[MAX_KEYPOINT_NUM],
                   int count, FLOAT r_vec[3], FLOAT t_vec[3],
                   FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy,
                   IPOINT imagePoints[MAX_KEYPOINT_NUM]);

void ProjectPoints(OPOINT objectPoints[MAX_KEYPOINT_NUM],
                   int count, FLOAT r_vec[3], FLOAT t_vec[3],
                   FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy,
                   IPOINT imagePoints[MAX_KEYPOINT_NUM],
                   FLOAT dpdr[NUM_OF_ERRS][3], FLOAT dpdt[NUM_OF_ERRS][3]);

int decide_R_t(OPOINT opoints[MAX_KEYPOINT_NUM], IPOINT ipoints[MAX_KEYPOINT_NUM],
               int point_num, int fx, int fy, int cx, int cy,
               FLOAT rmat[9], FLOAT tvec[3],
               FLOAT epnp_rmat[9], FLOAT epnp_tvec[3],
               FLOAT rmat_former[9], FLOAT tvec_former[3], bool take_last);

void pnp_iterative(OPOINT opoint[MAX_KEYPOINT_NUM],
                   IPOINT ipoint[MAX_KEYPOINT_NUM],
                   int number_of_points, FLOAT epnp_rmat[9], FLOAT epnp_tvec[3],
                   FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy,
                   FLOAT rmat[9], FLOAT tvec[3], bool take_last);
