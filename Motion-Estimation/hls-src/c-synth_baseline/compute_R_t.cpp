#include "compute_R_t.h"

void compute_R_and_t_x3(FLOAT u[12 * 12],
				FLOAT Beta3[4],FLOAT Beta4[4],FLOAT Beta5[4],
                FLOAT alphas[MODEL_POINTS][4],
                FLOAT pws[MODEL_POINTS][3],
                FLOAT R[9][3], FLOAT t[9])
{
    FLOAT u3[12][4];
    FLOAT alphas3[MODEL_POINTS][4];
    //FLOAT beta3[4];
    FLOAT pws3[MODEL_POINTS][3];
    FLOAT R3[3][3];
    FLOAT t3[3];

    FLOAT u4[12][4];
    FLOAT alphas4[MODEL_POINTS][4];
    //FLOAT beta4[4];
    FLOAT pws4[MODEL_POINTS][3];
    FLOAT R4[3][3];
    FLOAT t4[3];

    FLOAT u5[12][4];
    FLOAT alphas5[MODEL_POINTS][4];
    //FLOAT beta5[4];
    FLOAT pws5[MODEL_POINTS][3];
    FLOAT R5[3][3];
    FLOAT t5[3];

split_u:
    for (int i = 0; i < 12; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            u3[i][j] = u[i * 12 + j + 8];
            u4[i][j] = u[i * 12 + j + 8];
            u5[i][j] = u[i * 12 + j + 8];
        }
    }

split_alpha:
    for (int i = 0; i < MODEL_POINTS; i++)
        for (int j = 0; j < 4; j++)
        {
            alphas3[i][j] = alphas[i][j];
            alphas4[i][j] = alphas[i][j];
            alphas5[i][j] = alphas[i][j];
        }

split_pws:
    for (int i = 0; i < MODEL_POINTS; i++)
        for (int j = 0; j < 3; j++)
        {
            pws3[i][j] = pws[i][j];
            pws4[i][j] = pws[i][j];
            pws5[i][j] = pws[i][j];
        }

/*
split_beta:
    for (int i = 0; i < 4; i++)
    {
        beta3[i] = Beta3[i];
        beta4[i] = Beta4[i];
        beta5[i] = Beta5[i];
    }*/

    estimate_R_t(u3, Beta3, alphas3, pws3,
                 u4, Beta4, alphas4, pws4,
                 u5, Beta5, alphas5, pws5, R3, t3, R4, t4, R5, t5);

merge_R_t:
    for(int i=0; i<3; i++)
    {
    	for(int j=0; j<3; j++)
    	{
    		R[i][j] = R3[i][j];
    		R[i+3][j] = R4[i][j];
    		R[i+6][j] = R5[i][j];
    	}
		t[i] = t3[i];
		t[i+3] = t4[i];
		t[i+6] = t5[i];
    }
    return;
}

void estimate_R_t(FLOAT u3[12][4], FLOAT Betas3[4],
                  FLOAT alphas3[MODEL_POINTS][4],
                  FLOAT pws3[MODEL_POINTS][3],
                  FLOAT u4[12][4], FLOAT Betas4[4],
                  FLOAT alphas4[MODEL_POINTS][4],
                  FLOAT pws4[MODEL_POINTS][3],
                  FLOAT u5[12][4], FLOAT Betas5[4],
                  FLOAT alphas5[MODEL_POINTS][4],
                  FLOAT pws5[MODEL_POINTS][3],
                  FLOAT R3[3][3], FLOAT t3[3],
                  FLOAT R4[3][3], FLOAT t4[3],
                  FLOAT R5[3][3], FLOAT t5[3])
{
#pragma HLS ALLOCATION function instances=compute_R_and_t limit=1
    compute_R_and_t(u3, Betas3, alphas3, pws3, R3, t3);
    compute_R_and_t(u4, Betas4, alphas4, pws4, R4, t4);
    compute_R_and_t(u5, Betas5, alphas5, pws5, R5, t5);

}


void compute_R_and_t(FLOAT u[12][4], FLOAT betas[4],
                      FLOAT alphas[MODEL_POINTS][4],
                      FLOAT pws[MODEL_POINTS][3],
                      FLOAT R[3][3], FLOAT t[3])
{
    FLOAT ccs[4][3] = {0};
    FLOAT pcs[MODEL_POINTS][3];
#pragma HLS ARRAY_PARTITION variable=u complete dim=0
#pragma HLS ARRAY_PARTITION variable=alphas complete dim=0
#pragma HLS ARRAY_PARTITION variable=betas complete dim=0
#pragma HLS ARRAY_PARTITION variable=pws complete dim=0
#pragma HLS ARRAY_PARTITION variable=ccs complete dim=0
#pragma HLS ARRAY_PARTITION variable=pcs complete dim=0

ccs:
    for (int i = 0; i < 4; i++)
    {
#pragma HLS PIPELINE
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 3; k++)
                ccs[j][k] += betas[i] * u[3 * j + k][3 - i];
    }

pcs:
    for (int i = 0; i < MODEL_POINTS; i++)
    {
#pragma HLS PIPELINE
        for (int j = 0; j < 3; j++)
            pcs[i][j] = alphas[i][0] * ccs[0][j] +
                        alphas[i][1] * ccs[1][j] +
                        alphas[i][2] * ccs[2][j] +
                        alphas[i][3] * ccs[3][j];
    }

    if (pcs[0][2] < 0.0)
    {
    ccs_neg:
        for (int i = 0; i < 4; i++)
        {
#pragma HLS PIPELINE
            for (int j = 0; j < 3; j++)
                ccs[i][j] = -ccs[i][j];
        }

    pcs_neg:
        for (int i = 0; i < MODEL_POINTS; i++)
        {
            pcs[i][0] = -pcs[i][0];
            pcs[i][1] = -pcs[i][1];
            pcs[i][2] = -pcs[i][2];
        }
    }

    FLOAT pc0[3], pw0[3];
#pragma HLS ARRAY_PARTITION variable=pc0 complete dim=0
#pragma HLS ARRAY_PARTITION variable=pw0 complete dim=0

    FLOAT pc0_0 = 0.0;
    FLOAT pc0_1 = 0.0;
    FLOAT pc0_2 = 0.0;
    FLOAT pw0_0 = 0.0;
    FLOAT pw0_1 = 0.0;
    FLOAT pw0_2 = 0.0;

pc0_pw0:
    for (int i = 0; i < MODEL_POINTS; i++)
    {
    	pc0_0 += pcs[i][0];
    	pc0_1 += pcs[i][1];
    	pc0_2 += pcs[i][2];
    	pw0_0 += pws[i][0];
    	pw0_1 += pws[i][1];
    	pw0_2 += pws[i][2];
    }

    pc0[0] = pc0_0 / MODEL_POINTS;
    pc0[1] = pc0_1 / MODEL_POINTS;
    pc0[2] = pc0_2 / MODEL_POINTS;
    pw0[0] = pw0_0 / MODEL_POINTS;
    pw0[1] = pw0_1 / MODEL_POINTS;
    pw0[2] = pw0_2 / MODEL_POINTS;
/*
pc0_pw0_average:
    for (int j = 0; j < 3; j++)
    {
        pc0[j] /= MODEL_POINTS;
        pw0[j] /= MODEL_POINTS;
    }*/

    FLOAT ABt[3*3] = {0};
#pragma HLS ARRAY_PARTITION variable=ABt complete dim=0
    FLOAT ABt_D[3];
    FLOAT ABt_U[3*3];
    FLOAT ABt_V[3*3];

ABt:
    for (int i = 0; i < MODEL_POINTS; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            ABt[j * 3 + 0] += (pcs[i][j] - pc0[j]) * (pws[i][0] - pw0[0]);
            ABt[j * 3 + 1] += (pcs[i][j] - pc0[j]) * (pws[i][1] - pw0[1]);
            ABt[j * 3 + 2] += (pcs[i][j] - pc0[j]) * (pws[i][2] - pw0[2]);
        }
    }

    xf::solver::svd<FLOAT>(ABt, ABt_D, ABt_U, ABt_V, 3);

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
        	FLOAT sum = 0;
        	for(int k=0; k<3; k++)
        	{
        		sum += ABt_U[i * 3 + k] * ABt_V[j * 3 + k];
        	}
        	R[i][j] = sum;
        }
    }

    const FLOAT det =
        R[0][0] * R[1][1] * R[2][2] +
        R[0][1] * R[1][2] * R[2][0] +
        R[0][2] * R[1][0] * R[2][1] -
        R[0][2] * R[1][1] * R[2][0] -
        R[0][1] * R[1][0] * R[2][2] -
        R[0][0] * R[1][2] * R[2][1];

    if (det < 0)
    {
        R[2][0] = -R[2][0];
        R[2][1] = -R[2][1];
        R[2][2] = -R[2][2];
    }

    t[0] = pc0[0] - doty(R[0], pw0);
    t[1] = pc0[1] - doty(R[1], pw0);
    t[2] = pc0[2] - doty(R[2], pw0);
}


FLOAT doty(FLOAT a[3], FLOAT b[3])
{
    FLOAT a_int[3], b_int[3];
#pragma HLS array_partition variable=a_int dim=1 complete
#pragma HLS array_partition variable=b_int dim=1 complete
    FLOAT product = 0;

    for (int i = 0; i < 3; i++)
    {
#pragma HLS pipeline
        a_int[i] = a[i];
    }
    for (int i = 0; i < 3; i++)
    {
#pragma HLS pipeline
        b_int[i] = b[i];
    }

    for (int i = 0; i < 3; i++)
    {
#pragma HLS unroll
        product += a_int[i] * b_int[i];
    }

    return product;
}

