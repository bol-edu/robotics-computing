#include "EPnP.h"

EPnP::EPnP(FLOAT opoint[MODEL_POINTS][3],
           FLOAT ipoint[MODEL_POINTS][2],
           FLOAT _fx, FLOAT _fy, FLOAT _cx, FLOAT _cy)
{
    pws = Matrix<FLOAT, MODEL_POINTS, 3>();
    us = Matrix<FLOAT, MODEL_POINTS, 2>();
    for (int32_t i = 0; i < MODEL_POINTS; i++)
    {
        pws.val[i][0] = opoint[i][0];
        pws.val[i][1] = opoint[i][1];
        pws.val[i][2] = opoint[i][2];

        us.val[i][0] = ipoint[i][0];
        us.val[i][1] = ipoint[i][1];
    }
    // pws = opoint;
    // us = ipoint;

    cx = _cx;
    cy = _cy;
    fx = _fx;
    fy = _fy;
}

void EPnP::compute()
{
    // control points in world-coord;
    cws = Matrix<FLOAT, 4, 3>();
    // Take C0 as the reference points centroid:
    cws.val[0][0] = cws.val[0][1] = cws.val[0][2] = 0;
    for (int i = 0; i < MODEL_POINTS; i++)
        for (int j = 0; j < 3; j++)
            cws.val[0][j] += pws.val[i][j];

    for (int j = 0; j < 3; j++)
        cws.val[0][j] /= MODEL_POINTS;

    // Take C1, C2, and C3 from PCA on the reference points:
    PW0 = Matrix<FLOAT, MODEL_POINTS, 3>();
    for (int i = 0; i < MODEL_POINTS; i++)
        for (int j = 0; j < 3; j++)
            PW0.val[i][j] = pws.val[i][j] - cws.val[0][j];

    PW0tPW0 = PW0.multrans();

    DC = Matrix<FLOAT, 3, 1>();
    UC = Matrix<FLOAT, 3, 3>();
    VC = Matrix<FLOAT, 3, 3>();
    PW0tPW0.svd(UC, DC, VC);

    for (int i = 1; i < 4; i++)
    {
        FLOAT k = sqrt(DC.val[i - 1][0] / MODEL_POINTS);
        for (int j = 0; j < 3; j++)
            cws.val[i][j] = cws.val[0][j] + k * UC.val[j][i - 1];
    }

    // compute_barycentric_coordinates();
    CC = Matrix<FLOAT, 3, 3>();

    for (int i = 0; i < 3; i++)
        for (int j = 1; j < 4; j++)
            CC.val[i][j - 1] = cws.val[j][i] - cws.val[0][i];

    CC_inv = CC.inv();

    alphas = Matrix<FLOAT, MODEL_POINTS, 4>();
    for (int i = 0; i < MODEL_POINTS; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            alphas.val[i][1 + j] =
                CC_inv.val[j][0] * (pws.val[i][0] - cws.val[0][0]) +
                CC_inv.val[j][1] * (pws.val[i][1] - cws.val[0][1]) +
                CC_inv.val[j][2] * (pws.val[i][2] - cws.val[0][2]);
        }
        alphas.val[i][0] = 1.0f - alphas.val[i][1] - alphas.val[i][2] - alphas.val[i][3];
    }

    // fill_M ();
    M = Matrix<FLOAT, 2 * MODEL_POINTS, 12>();
    for (int i = 0; i < MODEL_POINTS; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            M.val[2 * i][3 * j] = alphas.val[i][j] * fx;
            M.val[2 * i][3 * j + 1] = 0.0;
            M.val[2 * i][3 * j + 2] = alphas.val[i][j] * (cx - us.val[i][0]);

            M.val[2 * i + 1][3 * j] = 0.0;
            M.val[2 * i + 1][3 * j + 1] = alphas.val[i][j] * fy;
            M.val[2 * i + 1][3 * j + 2] = alphas.val[i][j] * (cy - us.val[i][1]);
        }
    }
    MtM = M.multrans();
    MtM.svd(UM, DM, VM);

    // compute_L_6x10();
    Matrix<FLOAT, 6, 10> L_6x10 = Matrix<FLOAT, 6, 10>();
    Matrix<FLOAT, 6, 1> Rho = Matrix<FLOAT, 6, 1>();
    for (int i = 0; i < 4; i++)
        dv[i] = Matrix<FLOAT, 6, 3>();

    for (int i = 0; i < 4; i++)
    {
        int a = 0, b = 1;
        for (int j = 0; j < 6; j++)
        {
            dv[i].val[j][0] = UM.val[3 * b][11 - i] - UM.val[3 * a][11 - i];
            dv[i].val[j][1] = UM.val[3 * b + 1][11 - i] - UM.val[3 * a + 1][11 - i];
            dv[i].val[j][2] = UM.val[3 * b + 2][11 - i] - UM.val[3 * a + 2][11 - i];

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
        L_6x10.val[i][0] = dot(dv[0].val[i], dv[0].val[i]);
        L_6x10.val[i][1] = 2.0f * dot(dv[0].val[i], dv[1].val[i]);
        L_6x10.val[i][2] = dot(dv[1].val[i], dv[1].val[i]);
        L_6x10.val[i][3] = 2.0f * dot(dv[0].val[i], dv[2].val[i]);
        L_6x10.val[i][4] = 2.0f * dot(dv[1].val[i], dv[2].val[i]);
        L_6x10.val[i][5] = dot(dv[2].val[i], dv[2].val[i]);
        L_6x10.val[i][6] = 2.0f * dot(dv[0].val[i], dv[3].val[i]);
        L_6x10.val[i][7] = 2.0f * dot(dv[1].val[i], dv[3].val[i]);
        L_6x10.val[i][8] = 2.0f * dot(dv[2].val[i], dv[3].val[i]);
        L_6x10.val[i][9] = dot(dv[3].val[i], dv[3].val[i]);
    }

    // compute_rho(rho);
    Rho.val[0][0] = (cws.val[0][0] - cws.val[1][0]) * (cws.val[0][0] - cws.val[1][0]) +
                    (cws.val[0][1] - cws.val[1][1]) * (cws.val[0][1] - cws.val[1][1]) +
                    (cws.val[0][2] - cws.val[1][2]) * (cws.val[0][2] - cws.val[1][2]);
    Rho.val[1][0] = (cws.val[0][0] - cws.val[2][0]) * (cws.val[0][0] - cws.val[2][0]) +
                    (cws.val[0][1] - cws.val[2][1]) * (cws.val[0][1] - cws.val[2][1]) +
                    (cws.val[0][2] - cws.val[2][2]) * (cws.val[0][2] - cws.val[2][2]);
    Rho.val[2][0] = (cws.val[0][0] - cws.val[3][0]) * (cws.val[0][0] - cws.val[3][0]) +
                    (cws.val[0][1] - cws.val[3][1]) * (cws.val[0][1] - cws.val[3][1]) +
                    (cws.val[0][2] - cws.val[3][2]) * (cws.val[0][2] - cws.val[3][2]);
    Rho.val[3][0] = (cws.val[1][0] - cws.val[2][0]) * (cws.val[1][0] - cws.val[2][0]) +
                    (cws.val[1][1] - cws.val[2][1]) * (cws.val[1][1] - cws.val[2][1]) +
                    (cws.val[1][2] - cws.val[2][2]) * (cws.val[1][2] - cws.val[2][2]);
    Rho.val[4][0] = (cws.val[1][0] - cws.val[3][0]) * (cws.val[1][0] - cws.val[3][0]) +
                    (cws.val[1][1] - cws.val[3][1]) * (cws.val[1][1] - cws.val[3][1]) +
                    (cws.val[1][2] - cws.val[3][2]) * (cws.val[1][2] - cws.val[3][2]);
    Rho.val[5][0] = (cws.val[2][0] - cws.val[3][0]) * (cws.val[2][0] - cws.val[3][0]) +
                    (cws.val[2][1] - cws.val[3][1]) * (cws.val[2][1] - cws.val[3][1]) +
                    (cws.val[2][2] - cws.val[3][2]) * (cws.val[2][2] - cws.val[3][2]);

    Matrix<FLOAT, 4, 4> Betas = Matrix<FLOAT, 4, 4>();
    FLOAT rep_errors[4] = {};
    ts = Matrix<FLOAT, 4, 3>();
    for (int i = 0; i < 4; i++)
        Rs[i] = Matrix<FLOAT, 3, 3>();

    // find_betas_approx_1(&L_6x10, &Rho, Betas[1]);
    L_6x4 = Matrix<FLOAT, 6, 4>();

    for (int i = 0; i < 6; i++)
    {
        L_6x4.val[i][0] = L_6x10.val[i][0];
        L_6x4.val[i][1] = L_6x10.val[i][1];
        L_6x4.val[i][2] = L_6x10.val[i][3];
        L_6x4.val[i][3] = L_6x10.val[i][6];
    }

    solve(L_6x4, B4, Rho);

    Betas.val[1][0] = sqrt(fabs(B4.val[0][0]));
    Betas.val[1][1] = fabs(B4.val[1][0]) / Betas.val[1][0];
    Betas.val[1][2] = fabs(B4.val[2][0]) / Betas.val[1][0];
    Betas.val[1][3] = fabs(B4.val[3][0]) / Betas.val[1][0];

    gauss_newton(L_6x10, Rho, Betas.val[1]);
    rep_errors[1] = compute_R_and_t(UM, Betas.val[1], Rs[1], ts.val[1]);

    // find_betas_approx_2(&L_6x10, &Rho, Betas[2]);
    L_6x3 = Matrix<FLOAT, 6, 3>();

    for (int i = 0; i < 6; i++)
    {
        L_6x3.val[i][0] = L_6x10.val[i][0];
        L_6x3.val[i][1] = L_6x10.val[i][1];
        L_6x3.val[i][2] = L_6x10.val[i][2];
    }

    // B3 = Matrix<FLOAT, 4, 1>();
    solve(L_6x3, B3, Rho);

    if (B3.val[0][0] < 0)
    {
        Betas.val[2][0] = sqrt(-B3.val[0][0]);
        Betas.val[2][1] = (B3.val[2][0] < 0) ? sqrt(-B3.val[2][0]) : 0.0;
    }
    else
    {
        Betas.val[2][0] = sqrt(B3.val[0][0]);
        Betas.val[2][1] = (B3.val[2][0] > 0) ? sqrt(B3.val[2][0]) : 0.0;
    }

    if (B3.val[1][0] < 0)
        Betas.val[2][0] = -Betas.val[2][0];

    Betas.val[2][2] = 0.0;
    Betas.val[2][3] = 0.0;
    gauss_newton(L_6x10, Rho, Betas.val[2]);
    rep_errors[2] = compute_R_and_t(UM, Betas.val[2], Rs[2], ts.val[2]);

    // find_betas_approx_3(&L_6x10, &Rho, Betas[3]);
    L_6x5 = Matrix<FLOAT, 6, 5>();

    for (int i = 0; i < 6; i++)
    {
        L_6x5.val[i][0] = L_6x10.val[i][0];
        L_6x5.val[i][1] = L_6x10.val[i][1];
        L_6x5.val[i][2] = L_6x10.val[i][2];
        L_6x5.val[i][3] = L_6x10.val[i][3];
        L_6x5.val[i][4] = L_6x10.val[i][4];
    }

    // Matrix B5;
    solve(L_6x5, B5, Rho);

    if (B5.val[0][0] < 0)
    {
        Betas.val[3][0] = sqrt(-B5.val[0][0]);
        Betas.val[3][1] = (B5.val[2][0] < 0) ? sqrt(-B5.val[2][0]) : 0.0;
    }
    else
    {
        Betas.val[3][0] = sqrt(B5.val[0][0]);
        Betas.val[3][1] = (B5.val[2][0] > 0) ? sqrt(B5.val[2][0]) : 0.0;
    }
    if (B5.val[1][0] < 0)
        Betas.val[3][0] = -Betas.val[3][0];
    Betas.val[3][2] = B5.val[3][0] / Betas.val[3][0];
    Betas.val[3][3] = 0.0;

    gauss_newton(L_6x10, Rho, Betas.val[3]);
    rep_errors[3] = compute_R_and_t(UM, Betas.val[3], Rs[3], ts.val[3]);

    int N = 1;
    if (rep_errors[2] < rep_errors[1])
        N = 2;
    if (rep_errors[3] < rep_errors[N])
        N = 3;

    rmat = Matrix<FLOAT, 3, 3>();
    tvec = Matrix<FLOAT, 3, 1>();
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
            rmat.val[i][j] = Rs[N].val[i][j];
        tvec.val[i][0] = ts.val[N][i];
    }/**/
}

void gauss_newton(const Matrix<FLOAT, 6, 10> &L_6x10, const Matrix<FLOAT, 6, 1> &Rho, FLOAT betas[4])
{
    const int32_t iterations_number = 5;

    Matrix<FLOAT, 6, 4> A = Matrix<FLOAT, 6, 4>();
    Matrix<FLOAT, 6, 1> B = Matrix<FLOAT, 6, 1>();
    Matrix<FLOAT, 4, 1> X = Matrix<FLOAT, 4, 1>();

    for (int32_t k = 0; k < iterations_number; k++)
    {
        for (int32_t i = 0; i < 6; i++)
        {
            A.val[i][0] = 2 * L_6x10.val[i][0] * betas[0] +
                          L_6x10.val[i][1] * betas[1] +
                          L_6x10.val[i][3] * betas[2] +
                          L_6x10.val[i][6] * betas[3];
            A.val[i][1] = L_6x10.val[i][1] * betas[0] +
                          2 * L_6x10.val[i][2] * betas[1] +
                          L_6x10.val[i][4] * betas[2] +
                          L_6x10.val[i][7] * betas[3];
            A.val[i][2] = L_6x10.val[i][3] * betas[0] +
                          L_6x10.val[i][4] * betas[1] +
                          2 * L_6x10.val[i][5] * betas[2] +
                          L_6x10.val[i][8] * betas[3];
            A.val[i][3] = L_6x10.val[i][6] * betas[0] +
                          L_6x10.val[i][7] * betas[1] +
                          L_6x10.val[i][8] * betas[2] +
                          2 * L_6x10.val[i][9] * betas[3];

            B.val[i][0] = Rho.val[i][0] -
                          (L_6x10.val[i][0] * betas[0] * betas[0] +
                           L_6x10.val[i][1] * betas[0] * betas[1] +
                           L_6x10.val[i][2] * betas[1] * betas[1] +
                           L_6x10.val[i][3] * betas[0] * betas[2] +
                           L_6x10.val[i][4] * betas[1] * betas[2] +
                           L_6x10.val[i][5] * betas[2] * betas[2] +
                           L_6x10.val[i][6] * betas[0] * betas[3] +
                           L_6x10.val[i][7] * betas[1] * betas[3] +
                           L_6x10.val[i][8] * betas[2] * betas[3] +
                           L_6x10.val[i][9] * betas[3] * betas[3]);
        }

        solve(A, X, B);

        for (int32_t i = 0; i < 4; i++)
            betas[i] += X.val[i][0];
    }
}

FLOAT EPnP::compute_R_and_t(const Matrix<FLOAT, 12, 12> &u, const FLOAT *betas,
                            Matrix<FLOAT, 3, 3> &R, FLOAT t[3])
{
    Matrix<FLOAT, 4, 3> ccs = Matrix<FLOAT, 4, 3>();
    pcs = Matrix<FLOAT, MODEL_POINTS, 3>();

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 3; k++)
                ccs.val[j][k] += betas[i] * u.val[3 * j + k][11 - i];
    }

    for (int i = 0; i < MODEL_POINTS; i++)
    {
        for (int j = 0; j < 3; j++)
            pcs.val[i][j] = alphas.val[i][0] * ccs.val[0][j] +
                            alphas.val[i][1] * ccs.val[1][j] +
                            alphas.val[i][2] * ccs.val[2][j] +
                            alphas.val[i][3] * ccs.val[3][j];
    }

    if (pcs.val[0][2] < 0.0)
    {
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 3; j++)
                ccs.val[i][j] = -ccs.val[i][j];

        for (int i = 0; i < MODEL_POINTS; i++)
        {
            pcs.val[i][0] = -pcs.val[i][0];
            pcs.val[i][1] = -pcs.val[i][1];
            pcs.val[i][2] = -pcs.val[i][2];
        }
    }

    // estimate_R_and_t(R, t);
    FLOAT pc0[3] = {}, pw0[3] = {};

    pc0[0] = pc0[1] = pc0[2] = 0.0;
    pw0[0] = pw0[1] = pw0[2] = 0.0;

    for (int i = 0; i < MODEL_POINTS; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            pc0[j] += pcs.val[i][j];
            pw0[j] += pws.val[i][j];
        }
    }

    for (int j = 0; j < 3; j++)
    {
        pc0[j] /= MODEL_POINTS;
        pw0[j] /= MODEL_POINTS;
    }

    Matrix<FLOAT, 3, 3> ABt = Matrix<FLOAT, 3, 3>();
    Matrix<FLOAT, 3, 1> ABt_D = Matrix<FLOAT, 3, 1>();
    Matrix<FLOAT, 3, 3> ABt_U = Matrix<FLOAT, 3, 3>();
    Matrix<FLOAT, 3, 3> ABt_V = Matrix<FLOAT, 3, 3>();

    for (int i = 0; i < MODEL_POINTS; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            ABt.val[j][0] += (pcs.val[i][j] - pc0[j]) * (pws.val[i][0] - pw0[0]);
            ABt.val[j][1] += (pcs.val[i][j] - pc0[j]) * (pws.val[i][1] - pw0[1]);
            ABt.val[j][2] += (pcs.val[i][j] - pc0[j]) * (pws.val[i][2] - pw0[2]);
        }
    }

    ABt.svd(ABt_U, ABt_D, ABt_V);

    R = Matrix<FLOAT, 3, 3>();
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            R.val[i][j] = dot(ABt_U.val[i], ABt_V.val[j]);
        }

    const FLOAT det =
        R.val[0][0] * R.val[1][1] * R.val[2][2] +
        R.val[0][1] * R.val[1][2] * R.val[2][0] +
        R.val[0][2] * R.val[1][0] * R.val[2][1] -
        R.val[0][2] * R.val[1][1] * R.val[2][0] -
        R.val[0][1] * R.val[1][0] * R.val[2][2] -
        R.val[0][0] * R.val[1][2] * R.val[2][1];

    if (det < 0)
    {
        R.val[2][0] = -R.val[2][0];
        R.val[2][1] = -R.val[2][1];
        R.val[2][2] = -R.val[2][2];
    }

    t[0] = pc0[0] - dot(R.val[0], pw0);
    t[1] = pc0[1] - dot(R.val[1], pw0);
    t[2] = pc0[2] - dot(R.val[2], pw0);

    FLOAT sum2 = 0.0;

    for (int i = 0; i < MODEL_POINTS; i++)
    {
        FLOAT Xc = dot(R.val[0], pws.val[i]) + t[0];
        FLOAT Yc = dot(R.val[1], pws.val[i]) + t[1];
        FLOAT inv_Zc = 1.0 / (dot(R.val[2], pws.val[i]) + t[2]);
        FLOAT ue = cx + fx * Xc * inv_Zc;
        FLOAT ve = cy + fy * Yc * inv_Zc;

        sum2 += sqrt((us.val[i][0] - ue) * (us.val[i][0] - ue) + (us.val[i][1] - ve) * (us.val[i][1] - ve));
    }

    return sum2 / MODEL_POINTS;
}
