#include "EPnP.h"

void swap(FLOAT &a, FLOAT &b)
{
    FLOAT temp = a;
    a = b;
    b = temp;
}

void inv(FLOAT input[3][3], FLOAT output[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (i == j)
                output[i][j] = 1;
            else
                output[i][j] = 0;
        }
    }

    const FLOAT eps = 1e-20;

    int indxc[3];
    int indxr[3];
    int ipiv[3];

    int i, icol, i3, j, k, l, ll;
    FLOAT big, dum, pivinv, temp;

    for (j = 0; j < 3; j++)
        ipiv[j] = 0;

    for (i = 0; i < 3; i++)
    {
        big = 0.0;

        for (j = 0; j < 3; j++)
        {
            if (ipiv[j] != 1)
            {
                for (k = 0; k < 3; k++)
                {
                    if (ipiv[k] == 0)
                    {
                        if (hls::fabs(input[j][k]) >= big)
                        {
                            big = hls::fabs(input[j][k]);
                            i3 = j;
                            icol = k;
                        }
                    }
                }
            }
        }
        ++(ipiv[icol]);

        if (i3 != icol)
        {
            for (l = 0; l < 3; l++)
                swap(input[i3][l], input[icol][l]);
            for (l = 0; l < 3; l++)
                swap(output[i3][l], output[icol][l]);
        }

        indxr[i] = i3;
        indxc[i] = icol;

        if (hls::fabs(input[icol][icol]) < eps)
        {
            for (int ii = 0; ii < 3; ii++)
            {
                indxc[ii] = 0;
                indxr[ii] = 0;
                ipiv[ii] = 0;
            }
            return;
        }

        pivinv = 1.0 / input[icol][icol];
        input[icol][icol] = 1.0;
        for (l = 0; l < 3; l++)
            input[icol][l] *= pivinv;
        for (l = 0; l < 3; l++)
            output[icol][l] *= pivinv;

        for (ll = 0; ll < 3; ll++)
            if (ll != icol)
            {
                dum = input[ll][icol];
                input[ll][icol] = 0.0;
                for (l = 0; l < 3; l++)
                    input[ll][l] -= input[icol][l] * dum;
                for (l = 0; l < 3; l++)
                    output[ll][l] -= output[icol][l] * dum;
            }
    }

    for (l = 3 - 1; l >= 0; l--)
    {
        if (indxr[l] != indxc[l])
            for (k = 0; k < 3; k++)
                swap(input[k][indxr[l]], input[k][indxc[l]]);
    }
}

void compute_R_and_t(FLOAT u[12][4], FLOAT betas[4],
                     FLOAT alphas[MODEL_POINTS][4],
                     FLOAT pws[MODEL_POINTS][3],
                     FLOAT R[3][3], FLOAT t[3])
{
    FLOAT ccs[4][3] = {0};
    FLOAT pcs[MODEL_POINTS][3];
#pragma HLS ARRAY_PARTITION variable = u complete dim = 0
#pragma HLS ARRAY_PARTITION variable = alphas complete dim = 0
#pragma HLS ARRAY_PARTITION variable = betas complete dim = 0
#pragma HLS ARRAY_PARTITION variable = pws complete dim = 0
#pragma HLS ARRAY_PARTITION variable = ccs complete dim = 0
#pragma HLS ARRAY_PARTITION variable = pcs complete dim = 0

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
#pragma HLS ARRAY_PARTITION variable = pc0 complete dim = 0
#pragma HLS ARRAY_PARTITION variable = pw0 complete dim = 0

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

    FLOAT ABt[3 * 3] = {0};
#pragma HLS ARRAY_PARTITION variable = ABt complete dim = 0
    FLOAT ABt_D[3];
    FLOAT ABt_U[3 * 3];
    FLOAT ABt_V[3 * 3];

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
            for (int k = 0; k < 3; k++)
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

    t[0] = pc0[0] - dot(R[0], pw0);
    t[1] = pc0[1] - dot(R[1], pw0);
    t[2] = pc0[2] - dot(R[2], pw0);
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
#pragma HLS ALLOCATION function instances = compute_R_and_t limit = 1
    compute_R_and_t(u3, Betas3, alphas3, pws3, R3, t3);
    compute_R_and_t(u4, Betas4, alphas4, pws4, R4, t4);
    compute_R_and_t(u5, Betas5, alphas5, pws5, R5, t5);
}

void compute_R_and_t_x3(FLOAT u[12 * 12],
                        FLOAT Beta3[4], FLOAT Beta4[4], FLOAT Beta5[4],
                        FLOAT alphas[MODEL_POINTS][4],
                        FLOAT pws[MODEL_POINTS][3],
                        FLOAT R[9][3], FLOAT t[9])
{
    FLOAT u3[12][4];
    FLOAT alphas3[MODEL_POINTS][4];
    FLOAT pws3[MODEL_POINTS][3];
    FLOAT R3[3][3];
    FLOAT t3[3];

    FLOAT u4[12][4];
    FLOAT alphas4[MODEL_POINTS][4];
    FLOAT pws4[MODEL_POINTS][3];
    FLOAT R4[3][3];
    FLOAT t4[3];

    FLOAT u5[12][4];
    FLOAT alphas5[MODEL_POINTS][4];
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

    estimate_R_t(u3, Beta3, alphas3, pws3,
                 u4, Beta4, alphas4, pws4,
                 u5, Beta5, alphas5, pws5, R3, t3, R4, t4, R5, t5);

merge_R_t:
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            R[i][j] = R3[i][j];
            R[i + 3][j] = R4[i][j];
            R[i + 6][j] = R5[i][j];
        }
        t[i] = t3[i];
        t[i + 3] = t4[i];
        t[i + 6] = t5[i];
    }
    return;
}

void gauss_newton(FLOAT L_6x10[6][10], FLOAT Rho[6], FLOAT betas[4])
{
    const int iterations_number = 5;

    FLOAT A[30];
    FLOAT B[6];
    FLOAT X[5];

    for (int k = 0; k < iterations_number; k++)
    {
        for (int i = 0; i < 6; i++)
        {
            A[i * 4] = 2 * L_6x10[i][0] * betas[0] +
                       L_6x10[i][1] * betas[1] +
                       L_6x10[i][3] * betas[2] +
                       L_6x10[i][6] * betas[3];
            A[i * 4 + 1] = L_6x10[i][1] * betas[0] +
                           2 * L_6x10[i][2] * betas[1] +
                           L_6x10[i][4] * betas[2] +
                           L_6x10[i][7] * betas[3];
            A[i * 4 + 2] = L_6x10[i][3] * betas[0] +
                           L_6x10[i][4] * betas[1] +
                           2 * L_6x10[i][5] * betas[2] +
                           L_6x10[i][8] * betas[3];
            A[i * 4 + 3] = L_6x10[i][6] * betas[0] +
                           L_6x10[i][7] * betas[1] +
                           L_6x10[i][8] * betas[2] +
                           2 * L_6x10[i][9] * betas[3];

            B[i] = Rho[i] - (L_6x10[i][0] * betas[0] * betas[0] +
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

        // solve<6, 4>(A,X,B);
        solve(A, X, B, 6, 4);

        for (int i = 0; i < 4; i++)
            betas[i] += X[i];
    }
}

void epnp(OPOINT opoint[MODEL_POINTS],
          IPOINT ipoint[MODEL_POINTS],
          FLOAT fx, FLOAT fy, FLOAT cx, FLOAT cy,
          FLOAT rmat[9], FLOAT tvec[3])
{
    FLOAT pws[MODEL_POINTS][3];
#pragma HLS ARRAY_PARTITION variable = pws type = complete
    // FLOAT us[MODEL_POINTS][2];

    for (int i = 0; i < MODEL_POINTS; i++)
    {
        pws[i][0] = opoint[i].x;
        pws[i][1] = opoint[i].y;
        pws[i][2] = opoint[i].z;

        // us[i][0] = ipoint[i].x;
        // us[i][1] = ipoint[i].y;
    }

    FLOAT cws[4][3];
    cws[0][0] = cws[0][1] = cws[0][2] = 0;
    for (int i = 0; i < MODEL_POINTS; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            cws[0][j] += pws[i][j];
        }
    }

    for (int j = 0; j < 3; j++)
        cws[0][j] /= MODEL_POINTS;

    FLOAT PW0[MODEL_POINTS * 3];
#pragma HLS ARRAY_PARTITION variable = PW0 type = complete
    for (int i = 0; i < MODEL_POINTS; i++)
        for (int j = 0; j < 3; j++)
            PW0[i * 3 + j] = pws[i][j] - cws[0][j];

    FLOAT PW0tPW0[12 * 12]; //[3*3];
#pragma HLS ARRAY_PARTITION variable = PW0tPW0 type = complete
    multrans(PW0, PW0tPW0, MODEL_POINTS, 3);

    FLOAT DC[12];      //[3];
    FLOAT UC[12 * 12]; //[3*3];
    FLOAT VC[12 * 12]; //[3*3];

#pragma HLS ALLOCATION function instances = xf::solver::svd limit = 1
    xf::solver::svd(PW0tPW0, DC, UC, VC, 3);

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
    inv(CC, CC_inv);

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
            M[2 * i * 12 + 3 * j + 2] = alphas[i][j] * (cx - ipoint[i].x);

            M[(2 * i + 1) * 12 + 3 * j] = 0.0;
            M[(2 * i + 1) * 12 + 3 * j + 1] = alphas[i][j] * fy;
            M[(2 * i + 1) * 12 + 3 * j + 2] = alphas[i][j] * (cy - ipoint[i].y);
        }
    }

    FLOAT MtM[12 * 12];
    multrans(M, MtM, 2 * MODEL_POINTS, 12);

    FLOAT DM[12];
    FLOAT UM[12 * 12];
    FLOAT VM[12 * 12];
    // svd<12, 12>(MtM, UM, DM, VM);

    xf::solver::svd(MtM, DM, UM, VM, 12);

    // compute_L_6x10();
    FLOAT L_6x10[6][10];
#pragma HLS ARRAY_PARTITION variable = L_6x10 type = complete
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
#pragma HLS ARRAY_PARTITION variable = L_6x4 type = complete
#pragma HLS ARRAY_PARTITION variable = L_6x3 type = complete
#pragma HLS ARRAY_PARTITION variable = L_6x5 type = complete

    for (int i = 0; i < 6; i++)
    {
        L_6x4[i * 4] = L_6x10[i][0];
        L_6x4[i * 4 + 1] = L_6x10[i][1];
        L_6x4[i * 4 + 2] = L_6x10[i][3];
        L_6x4[i * 4 + 3] = L_6x10[i][6];

        L_6x3[i * 3] = L_6x10[i][0];
        L_6x3[i * 3 + 1] = L_6x10[i][1];
        L_6x3[i * 3 + 2] = L_6x10[i][2];

        L_6x5[i * 5] = L_6x10[i][0];
        L_6x5[i * 5 + 1] = L_6x10[i][1];
        L_6x5[i * 5 + 2] = L_6x10[i][2];
        L_6x5[i * 5 + 3] = L_6x10[i][3];
        L_6x5[i * 5 + 4] = L_6x10[i][4];
    }

    FLOAT B4[5];
    FLOAT B3[5];
    FLOAT B5[5];
    // solve<6, 4>(L_6x4, B4, Rho);
    // solve<6, 3>(L_6x3, B3, Rho);
    // solve<6, 5>(L_6x5, B5, Rho);

#pragma HLS ALLOCATION function instances = solve limit = 1
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

        rep_errors3 += hls::pow(ipoint[i].x - ue3, 2.f) + hls::pow(ipoint[i].y - ve3, 2.f);
        rep_errors4 += hls::pow(ipoint[i].x - ue4, 2.f) + hls::pow(ipoint[i].y - ve4, 2.f);
        rep_errors5 += hls::pow(ipoint[i].x - ue5, 2.f) + hls::pow(ipoint[i].y - ve5, 2.f);
    }

    int N = 0;
    FLOAT rep_errors = rep_errors3;
    if (rep_errors4 < rep_errors3)
    {
        rep_errors = rep_errors4;
        N = 3;
    }
    if (rep_errors5 < rep_errors)
        N = 6;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            rmat[i * 3 + j] = Rs[i + N][j];
        }
        tvec[i] = ts[i + N];
    }
}
