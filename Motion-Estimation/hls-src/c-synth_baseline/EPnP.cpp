#include "EPnP.h"
#include "sym_svd.hpp"
#include "compute_R_t.h"

void epnp(OPOINT opoint[MODEL_POINTS],
          IPOINT ipoint[MODEL_POINTS],
          FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy,
          volatile FLOAT *rmat, volatile FLOAT *tvec)
{
	FLOAT pws[MODEL_POINTS][3];
	FLOAT us[MODEL_POINTS][2];

    for (int i = 0; i < MODEL_POINTS; i++)
    {
        pws[i][0] = opoint[i].x;
        pws[i][1] = opoint[i].y;
        pws[i][2] = opoint[i].z;

        us[i][0] = ipoint[i].x;
        us[i][1] = ipoint[i].y;
    }

	FLOAT cws[4][3];
    cws[0][0] = cws[0][1] = cws[0][2] = 0;
    for (int i = 0; i < MODEL_POINTS; i++)
    	for (int j = 0; j < 3; j++)
                cws[0][j] += pws[i][j];

    for (int j = 0; j < 3; j++)
    	cws[0][j] /= MODEL_POINTS;


	FLOAT PW0[MODEL_POINTS*3];
	for (int i = 0; i < MODEL_POINTS; i++)
		for (int j = 0; j < 3; j++)
			PW0[i * 3 + j] = pws[i][j] - cws[0][j];

	FLOAT PW0tPW0[12*12];//[3*3];
	multrans(PW0, PW0tPW0, MODEL_POINTS, 3);
    FLOAT DC[12];//[3];
    FLOAT UC[12*12];//[3*3];
    FLOAT VC[12*12];//[3*3];

//#pragma HLS ALLOCATION function instances=xf::solver::svd<FLOAT> limit=1
    xf::solver::svd<FLOAT>(PW0tPW0, DC, UC, VC, 3);

    for (int i = 1; i < 4; i++)
    {
    	FLOAT k = sqrt(DC[i - 1] / MODEL_POINTS);
    	for (int j = 0; j < 3; j++)
    	{
    		cws[i][j] = cws[0][j] + k * UC[j * 3 + i - 1];
    	}
    }

    FLOAT CC[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 1; j < 4; j++)
            CC[i][j - 1] = cws[j][i] - cws[0][i];

    FLOAT CC_inv[3][3];
    inv<3>(CC, CC_inv);

    FLOAT alphas[MODEL_POINTS][4];
    for (int i = 0; i < MODEL_POINTS; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            alphas[i][1 + j] =
                CC_inv[j][0] * (pws[i][0] - cws[0][0]) +
                CC_inv[j][1] * (pws[i][1] - cws[0][1]) +
                CC_inv[j][2] * (pws[i][2] - cws[0][2]);
        }
        alphas[i][0] = 1.0f - alphas[i][1] - alphas[i][2] - alphas[i][3];
    }

    // fill_M ();
    FLOAT M[2 * MODEL_POINTS * 12];
    for (int i = 0; i < MODEL_POINTS; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            M[2 * i * 12 + 3 * j] = alphas[i][j] * fx;
            M[2 * i * 12 + 3 * j + 1] = 0.0;
            M[2 * i * 12 + 3 * j + 2] = alphas[i][j] * (cx - us[i][0]);

            M[(2 * i + 1) * 12 + 3 * j] = 0.0;
            M[(2 * i + 1) * 12 + 3 * j + 1] = alphas[i][j] * fy;
            M[(2 * i + 1) * 12 + 3 * j + 2] = alphas[i][j] * (cy - us[i][1]);
        }
    }

    FLOAT MtM[12*12];
    multrans(M, MtM, 2*MODEL_POINTS, 12);

    FLOAT DM[12];
    FLOAT UM[12*12];
    FLOAT VM[12*12];
    //svd<12, 12>(MtM, UM, DM, VM);

    xf::solver::svd<FLOAT>(MtM, DM, UM, VM, 12);

    // compute_L_6x10();
    FLOAT L_6x10[6][10];
    FLOAT Rho[6];
    FLOAT dv[4][6][3];

    for (int i = 0; i < 4; i++)
    {
    	int a = 0, b = 1;
    	for (int j = 0; j < 6; j++)
    	{
    		dv[i][j][0] = UM[(3 * b) * 12 + 11 - i] - UM[(3 * a) * 12 + 11 - i];
    		dv[i][j][1] = UM[(3 * b + 1) * 12 + 11 - i] - UM[(3 * a + 1) * 12 + 11 - i];
    		dv[i][j][2] = UM[(3 * b + 2) * 12 + 11 - i] - UM[(3 * a + 2) * 12 + 11 - i];

    		b++;
    		if (b > 3)
    		{
    			a++;
    			b = a + 1;
    		}
    	}
    }

    for (int i = 0; i < 6; i++)
    {
        L_6x10[i][0] = dot(dv[0][i], dv[0][i]);
        L_6x10[i][1] = 2.0f * dot(dv[0][i], dv[1][i]);
        L_6x10[i][2] = dot(dv[1][i], dv[1][i]);
        L_6x10[i][3] = 2.0f * dot(dv[0][i], dv[2][i]);
        L_6x10[i][4] = 2.0f * dot(dv[1][i], dv[2][i]);
        L_6x10[i][5] = dot(dv[2][i], dv[2][i]);
        L_6x10[i][6] = 2.0f * dot(dv[0][i], dv[3][i]);
        L_6x10[i][7] = 2.0f * dot(dv[1][i], dv[3][i]);
        L_6x10[i][8] = 2.0f * dot(dv[2][i], dv[3][i]);
        L_6x10[i][9] = dot(dv[3][i], dv[3][i]);
    }

    // compute_rho(rho);
    Rho[0] = (cws[0][0] - cws[1][0]) * (cws[0][0] - cws[1][0]) +
                    (cws[0][1] - cws[1][1]) * (cws[0][1] - cws[1][1]) +
                    (cws[0][2] - cws[1][2]) * (cws[0][2] - cws[1][2]);
    Rho[1] = (cws[0][0] - cws[2][0]) * (cws[0][0] - cws[2][0]) +
                    (cws[0][1] - cws[2][1]) * (cws[0][1] - cws[2][1]) +
                    (cws[0][2] - cws[2][2]) * (cws[0][2] - cws[2][2]);
    Rho[2] = (cws[0][0] - cws[3][0]) * (cws[0][0] - cws[3][0]) +
                    (cws[0][1] - cws[3][1]) * (cws[0][1] - cws[3][1]) +
                    (cws[0][2] - cws[3][2]) * (cws[0][2] - cws[3][2]);
    Rho[3] = (cws[1][0] - cws[2][0]) * (cws[1][0] - cws[2][0]) +
                    (cws[1][1] - cws[2][1]) * (cws[1][1] - cws[2][1]) +
                    (cws[1][2] - cws[2][2]) * (cws[1][2] - cws[2][2]);
    Rho[4] = (cws[1][0] - cws[3][0]) * (cws[1][0] - cws[3][0]) +
                    (cws[1][1] - cws[3][1]) * (cws[1][1] - cws[3][1]) +
                    (cws[1][2] - cws[3][2]) * (cws[1][2] - cws[3][2]);
    Rho[5] = (cws[2][0] - cws[3][0]) * (cws[2][0] - cws[3][0]) +
                    (cws[2][1] - cws[3][1]) * (cws[2][1] - cws[3][1]) +
                    (cws[2][2] - cws[3][2]) * (cws[2][2] - cws[3][2]);

    FLOAT Betas[4][4];

    // find_betas_approx_1(&L_6x10, &Rho, Betas[1]);
    FLOAT L_6x4[30];
    FLOAT L_6x3[30];
    FLOAT L_6x5[30];

    for (int i = 0; i < 6; i++)
    {
        L_6x4[i*4] = L_6x10[i][0];
        L_6x4[i*4 + 1] = L_6x10[i][1];
        L_6x4[i*4 + 2] = L_6x10[i][3];
        L_6x4[i*4 + 3] = L_6x10[i][6];

    	L_6x3[i*3] = L_6x10[i][0];
    	L_6x3[i*3 + 1] = L_6x10[i][1];
    	L_6x3[i*3 + 2] = L_6x10[i][2];

        L_6x5[i*5] = L_6x10[i][0];
        L_6x5[i*5 + 1] = L_6x10[i][1];
        L_6x5[i*5 + 2] = L_6x10[i][2];
        L_6x5[i*5 + 3] = L_6x10[i][3];
        L_6x5[i*5 + 4] = L_6x10[i][4];
    }

    FLOAT B4[5];
    FLOAT B3[5];
    FLOAT B5[5];
    //solve<6, 4>(L_6x4, B4, Rho);
    //solve<6, 3>(L_6x3, B3, Rho);
    //solve<6, 5>(L_6x5, B5, Rho);

    solve(L_6x4, B4, Rho, 6, 4);
    solve(L_6x3, B3, Rho, 6, 3);
    solve(L_6x5, B5, Rho, 6, 5);

    Betas[1][0] = sqrt(fabs(B4[0]));
    Betas[1][1] = fabs(B4[1]) / Betas[1][0];
    Betas[1][2] = fabs(B4[2]) / Betas[1][0];
    Betas[1][3] = fabs(B4[3]) / Betas[1][0];

    if (B3[0] < 0)
    {
    	Betas[2][0] = sqrt(-B3[0]);
    	Betas[2][1] = (B3[2] < 0) ? sqrt(-B3[2]) : 0.0;
    }
    else
    {
    	Betas[2][0] = sqrt(B3[0]);
    	Betas[2][1] = (B3[2] > 0) ? sqrt(B3[2]) : 0.0;
    }
    if (B3[1] < 0)
    	Betas[2][0] = -Betas[2][0];

    Betas[2][2] = 0.0;
    Betas[2][3] = 0.0;


    if (B5[0] < 0)
    {
    	Betas[3][0] = sqrt(-B5[0]);
    	Betas[3][1] = (B5[2] < 0) ? sqrt(-B5[2]) : 0.0;
    }
    else
    {
    	Betas[3][0] = sqrt(B5[0]);
    	Betas[3][1] = (B5[2] > 0) ? sqrt(B5[2]) : 0.0;
    }
    if (B5[1] < 0)
    	Betas[3][0] = -Betas[3][0];
    Betas[3][2] = B5[3] / Betas[3][0];
    Betas[3][3] = 0.0;

    gauss_newton(L_6x10, Rho, Betas[1]);
    gauss_newton(L_6x10, Rho, Betas[2]);
    gauss_newton(L_6x10, Rho, Betas[3]);

    FLOAT Rs[9][3];
    FLOAT ts[9];
    compute_R_and_t_x3(UM, Betas[2], Betas[1], Betas[3], alphas, pws, Rs, ts);

    FLOAT rep_errors3 = 0.0;
    FLOAT rep_errors4 = 0.0;
    FLOAT rep_errors5 = 0.0;

    for (int i = 0; i < MODEL_POINTS; i++)
    {
        FLOAT Xc3 = dot(Rs[0], pws[i]) + ts[0];
        FLOAT Yc3 = dot(Rs[1], pws[i]) + ts[1];
        FLOAT inv_Zc3 = 1.0 / (dot(Rs[2], pws[i]) + ts[2]);
        FLOAT ue3 = cx + fx * Xc3 * inv_Zc3;
        FLOAT ve3 = cy + fy * Yc3 * inv_Zc3;

        FLOAT Xc4 = dot(Rs[3], pws[i]) + ts[3];
        FLOAT Yc4 = dot(Rs[4], pws[i]) + ts[4];
        FLOAT inv_Zc4 = 1.0 / (dot(Rs[5], pws[i]) + ts[5]);
        FLOAT ue4 = cx + fx * Xc4 * inv_Zc4;
        FLOAT ve4 = cy + fy * Yc4 * inv_Zc4;

        FLOAT Xc5 = dot(Rs[6], pws[i]) + ts[6];
        FLOAT Yc5 = dot(Rs[7], pws[i]) + ts[7];
        FLOAT inv_Zc5 = 1.0 / (dot(Rs[8], pws[i]) + ts[8]);
        FLOAT ue5 = cx + fx * Xc5 * inv_Zc5;
        FLOAT ve5 = cy + fy * Yc5 * inv_Zc5;

        rep_errors3 += hls::sqrt((us[i][0] - ue3) * (us[i][0] - ue3) + (us[i][1] - ve3) * (us[i][1] - ve3));
        rep_errors4 += hls::sqrt((us[i][0] - ue4) * (us[i][0] - ue4) + (us[i][1] - ve4) * (us[i][1] - ve4));
        rep_errors5 += hls::sqrt((us[i][0] - ue5) * (us[i][0] - ue5) + (us[i][1] - ve5) * (us[i][1] - ve5));
    }

    int N = 0;
    FLOAT rep_errors = rep_errors3;
    if (rep_errors4 < rep_errors3)
    {
    	rep_errors = rep_errors4;
    	N=3;
    }
    if (rep_errors5 < rep_errors)
    	N=6;

    for(int i=0; i<3; i++)
    	for(int j=0; j<3; j++)
    		rmat[i*3+j] = Rs[i+N][j];

    for(int i=0; i<3; i++)
    	tvec[i] = ts[i+N];
}


void gauss_newton(FLOAT L_6x10[6][10], FLOAT Rho[6], FLOAT betas[4])
{
    const int iterations_number = 5;

    FLOAT A[30];
    FLOAT B[6];
    FLOAT X[4];


    for (int k = 0; k < iterations_number; k++)
    {
        for (int i = 0; i < 6; i++)
        {
            A[i*4] = 2 * L_6x10[i][0] * betas[0] +
                          L_6x10[i][1] * betas[1] +
                          L_6x10[i][3] * betas[2] +
                          L_6x10[i][6] * betas[3];
            A[i*4 + 1] = L_6x10[i][1] * betas[0] +
                          2 * L_6x10[i][2] * betas[1] +
                          L_6x10[i][4] * betas[2] +
                          L_6x10[i][7] * betas[3];
            A[i*4 + 2] = L_6x10[i][3] * betas[0] +
                          L_6x10[i][4] * betas[1] +
                          2 * L_6x10[i][5] * betas[2] +
                          L_6x10[i][8] * betas[3];
            A[i*4 + 3] = L_6x10[i][6] * betas[0] +
                          L_6x10[i][7] * betas[1] +
                          L_6x10[i][8] * betas[2] +
                          2 * L_6x10[i][9] * betas[3];

            B[i] = Rho[i] -(L_6x10[i][0] * betas[0] * betas[0] +
                           L_6x10[i][1] * betas[0] * betas[1] +
                           L_6x10[i][2] * betas[1] * betas[1] +
                           L_6x10[i][3] * betas[0] * betas[2] +
                           L_6x10[i][4] * betas[1] * betas[2] +
                           L_6x10[i][5] * betas[2] * betas[2] +
                           L_6x10[i][6] * betas[0] * betas[3] +
                           L_6x10[i][7] * betas[1] * betas[3] +
                           L_6x10[i][8] * betas[2] * betas[3] +
                           L_6x10[i][9] * betas[3] * betas[3]);
        }

        //solve<6, 4>(A,X,B);
        solve(A, X, B, 6 , 4);

        for (int i = 0; i < 4; i++)
            betas[i] += X[i];

    }
}
