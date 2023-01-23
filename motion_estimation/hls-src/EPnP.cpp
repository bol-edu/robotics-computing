#include "EPnP.h"

EPnP::EPnP(Matrix &opoint, Matrix &ipoint, Matrix &k)
{
    number_of_correspondences = opoint.m;

    pws = Matrix(number_of_correspondences, 3);
    us = Matrix(number_of_correspondences, 2);
    for (int i = 0; i < number_of_correspondences; i++)
    {
        for (int j = 0; j < 3; j++)
            pws.val[i][j] = opoint.val[i][j];
        for (int j = 0; j < 2; j++)
            pws.val[i][j] = ipoint.val[i][j];
    }

    cx = k.val[0][2];
    cy = k.val[1][2];
    fx = k.val[0][0];
    fy = k.val[1][1];

    max_nr = 0;
    A1 = NULL;
    A2 = NULL;
}

EPnP::~EPnP()
{
}

void EPnP::compute(Matrix &rmat, Matrix &tvec)
{

    /*********************/
    pws.val[0][0] = 2.624186;
    pws.val[0][1] = -4.314296;
    pws.val[0][2] = 55.163544;
    pws.val[1][0] = 4.615482;
    pws.val[1][1] = -1.873971;
    pws.val[1][2] = 15.445792;
    pws.val[2][0] = -14.614742;
    pws.val[2][1] = -1.752392;
    pws.val[2][2] = 61.783169;
    pws.val[3][0] = -9.580322;
    pws.val[3][1] = -1.012847;
    pws.val[3][2] = 31.362015;
    pws.val[4][0] = -11.107309;
    pws.val[4][1] = -2.848838;
    pws.val[4][2] = 35.712814;

    us.val[0][0] = 644.972718;
    us.val[0][1] = 128.994550;
    us.val[1][0] = 868.999998;
    us.val[1][1] = 84.000000;
    us.val[2][0] = 437.148199;
    us.val[2][1] = 168.409546;
    us.val[3][0] = 385.200015;
    us.val[3][1] = 164.400009;
    us.val[4][0] = 381.888037;
    us.val[4][1] = 129.600007;
    /*********************/

    // control points in world-coord;
    Matrix cws = Matrix(4, 3);
    // Take C0 as the reference points centroid:
    cws.val[0][0] = cws.val[0][1] = cws.val[0][2] = 0;
    for (int i = 0; i < number_of_correspondences; i++)
        for (int j = 0; j < 3; j++)
            cws.val[0][j] += pws.val[i][j];

    for (int j = 0; j < 3; j++)
        cws.val[0][j] /= number_of_correspondences;

    // Take C1, C2, and C3 from PCA on the reference points:
    Matrix PW0 = Matrix(number_of_correspondences, 3);
    for (int i = 0; i < number_of_correspondences; i++)
        for (int j = 0; j < 3; j++)
            PW0.val[i][j] = pws.val[i][j] - cws.val[0][j];

    Matrix PW0tPW0 = PW0.multrans();

    Matrix DC;
    Matrix UC;
    Matrix VC;
    PW0tPW0.svd(UC, DC, VC);
    //  svd(MtM, λ, ν, ...)

    PW0.releaseMemory();

    for (int i = 1; i < 4; i++)
    {
        double k = sqrt(DC.val[i - 1][0] / number_of_correspondences);
        for (int j = 0; j < 3; j++)
            cws.val[i][j] = cws.val[0][j] - k * UC.val[j][i - 1];
    }
    DC.releaseMemory();
    UC.releaseMemory();
    VC.releaseMemory();

    // compute_barycentric_coordinates();
    Matrix CC = Matrix(3, 3);

    for (int i = 0; i < 3; i++)
        for (int j = 1; j < 4; j++)
            CC.val[i][j - 1] = cws.val[j][i] - cws.val[0][i];

    Matrix CC_inv = CC.inv();

    alphas = Matrix(number_of_correspondences, 4);
    for (int i = 0; i < number_of_correspondences; i++)
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
    CC.releaseMemory();
    CC_inv.releaseMemory();

    // fill_M ();
    Matrix M = Matrix(2 * number_of_correspondences, 12);
    for (int i = 0; i < number_of_correspondences; i++)
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

    Matrix MtM = M.multrans(), DM, UM, VM;
    MtM.svd(UM, DM, VM);

    // compute_L_6x10();
    Matrix L_6x10 = Matrix(6, 10);
    Matrix Rho = Matrix(6, 1);
    Matrix dv[4];
    for (int i = 0; i < 4; i++)
        dv[i] = Matrix(6, 3);

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
    MtM.releaseMemory();
    DM.releaseMemory();
    VM.releaseMemory();

    for (int i = 0; i < 6; i++)
    {
        L_6x10.val[i][0] = Matrix::dot(dv[0].val[i], dv[0].val[i]);
        L_6x10.val[i][1] = 2.0f * Matrix::dot(dv[0].val[i], dv[1].val[i]);
        L_6x10.val[i][2] = Matrix::dot(dv[1].val[i], dv[1].val[i]);
        L_6x10.val[i][3] = 2.0f * Matrix::dot(dv[0].val[i], dv[2].val[i]);
        L_6x10.val[i][4] = 2.0f * Matrix::dot(dv[1].val[i], dv[2].val[i]);
        L_6x10.val[i][5] = Matrix::dot(dv[2].val[i], dv[2].val[i]);
        L_6x10.val[i][6] = 2.0f * Matrix::dot(dv[0].val[i], dv[3].val[i]);
        L_6x10.val[i][7] = 2.0f * Matrix::dot(dv[1].val[i], dv[3].val[i]);
        L_6x10.val[i][8] = 2.0f * Matrix::dot(dv[2].val[i], dv[3].val[i]);
        L_6x10.val[i][9] = Matrix::dot(dv[3].val[i], dv[3].val[i]);
    }

    for (int i = 0; i < 4; i++)
        dv[i].releaseMemory();

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

    Matrix Betas = Matrix(4, 4);
    double rep_errors[4] = {};
    Matrix Rs[4], ts = Matrix(4, 3);
    for (int i = 0; i < 4; i++)
        Rs[i] = Matrix(3, 3);

    // find_betas_approx_1(&L_6x10, &Rho, Betas[1]);
    Matrix L_6x4 = Matrix(6, 4);

    for (int i = 0; i < 6; i++)
    {
        L_6x4.val[i][0] = L_6x10.val[i][0];
        L_6x4.val[i][1] = L_6x10.val[i][1];
        L_6x4.val[i][2] = L_6x10.val[i][3];
        L_6x4.val[i][3] = L_6x10.val[i][6];
    }

    /*********************
    Matrix A = Matrix(6, 4);
    A.val[0][0] = 0.027984;
    A.val[0][1] = -0.060804;
    A.val[0][2] = 0.153016;
    A.val[0][3] = 0.303753;
    A.val[1][0] = 0.026396;
    A.val[1][1] = 0.296465;
    A.val[1][2] = 0.027311;
    A.val[1][3] = 0.075335;
    A.val[2][0] = 0.000780;
    A.val[2][1] = 0.006145;
    A.val[2][2] = -0.046242;
    A.val[2][3] = 0.033726;
    A.val[3][0] = 0.002901;
    A.val[3][1] = -0.044043;
    A.val[3][2] = 0.005372;
    A.val[3][3] = 0.040965;
    A.val[4][0] = 0.020374;
    A.val[4][1] = -0.081809;
    A.val[4][2] = 0.380252;
    A.val[4][3] = 0.072675;
    A.val[5][0] = 0.018863;
    A.val[5][1] = 0.215490;
    A.val[5][2] = 0.260294;
    A.val[5][3] = -0.104581;
    /*********************/
    Matrix B4;
    Matrix::solve(L_6x4, B4, Rho);

    Betas.val[1][0] = sqrt(fabs(B4.val[0][0]));
    Betas.val[1][1] = fabs(B4.val[1][0]) / Betas.val[1][0];
    Betas.val[1][2] = fabs(B4.val[2][0]) / Betas.val[1][0];
    Betas.val[1][3] = fabs(B4.val[3][0]) / Betas.val[1][0];

    /*********************/
    Betas.val[1][0] = 372.335187;
    Betas.val[1][1] = -24.028105;
    Betas.val[1][2] = -17.729598;
    Betas.val[1][3] = -27.636468;

    L_6x10.val[0][0] = 0.027984;
    L_6x10.val[0][1] = -0.060804;
    L_6x10.val[0][2] = 0.037383;
    L_6x10.val[0][3] = 0.153016;
    L_6x10.val[0][4] = -0.165378;
    L_6x10.val[0][5] = 0.209288;
    L_6x10.val[0][6] = 0.303753;
    L_6x10.val[0][7] = -0.277593;
    L_6x10.val[0][8] = 0.836437;
    L_6x10.val[0][9] = 0.984205;
    L_6x10.val[1][0] = 0.026396;
    L_6x10.val[1][1] = 0.296465;
    L_6x10.val[1][2] = 1.082638;
    L_6x10.val[1][3] = 0.027311;
    L_6x10.val[1][4] = 0.228063;
    L_6x10.val[1][5] = 0.012857;
    L_6x10.val[1][6] = 0.075335;
    L_6x10.val[1][7] = 0.666175;
    L_6x10.val[1][8] = 0.074947;
    L_6x10.val[1][9] = 0.112922;
    L_6x10.val[2][0] = 0.000780;
    L_6x10.val[2][1] = 0.006145;
    L_6x10.val[2][2] = 0.013014;
    L_6x10.val[2][3] = -0.046242;
    L_6x10.val[2][4] = -0.201442;
    L_6x10.val[2][5] = 0.797527;
    L_6x10.val[2][6] = 0.033726;
    L_6x10.val[2][7] = 0.145162;
    L_6x10.val[2][8] = -1.168311;
    L_6x10.val[2][9] = 0.442505;
    L_6x10.val[3][0] = 0.002901;
    L_6x10.val[3][1] = -0.044043;
    L_6x10.val[3][2] = 1.467889;
    L_6x10.val[3][3] = 0.005372;
    L_6x10.val[3][4] = -0.844602;
    L_6x10.val[3][5] = 0.126673;
    L_6x10.val[3][6] = 0.040965;
    L_6x10.val[3][7] = -1.531845;
    L_6x10.val[3][8] = 0.415164;
    L_6x10.val[3][9] = 0.432202;
    L_6x10.val[4][0] = 0.020374;
    L_6x10.val[4][1] = -0.081809;
    L_6x10.val[4][2] = 0.093550;
    L_6x10.val[4][3] = 0.380252;
    L_6x10.val[4][4] = -0.803126;
    L_6x10.val[4][5] = 1.816916;
    L_6x10.val[4][6] = 0.072675;
    L_6x10.val[4][7] = -0.080413;
    L_6x10.val[4][8] = 0.593281;
    L_6x10.val[4][9] = 0.184029;
    L_6x10.val[5][0] = 0.018863;
    L_6x10.val[5][1] = 0.215490;
    L_6x10.val[5][2] = 0.877640;
    L_6x10.val[5][3] = 0.260294;
    L_6x10.val[5][4] = 1.786321;
    L_6x10.val[5][5] = 0.984866;
    L_6x10.val[5][6] = -0.104581;
    L_6x10.val[5][7] = -0.622992;
    L_6x10.val[5][8] = -0.738535;
    L_6x10.val[5][9] = 0.146579;
    /*********************/
    gauss_newton(L_6x10, Rho, Betas.val[1]);
    rep_errors[1] = compute_R_and_t(UM, Betas.val[1], Rs[1], ts.val[1]);
}

FLOAT EPnP::compute_R_and_t(const Matrix &u, const FLOAT *betas,
                            Matrix &R, FLOAT t[3])
{

    /*********************/
    u.val[0][0] = 0.767192;
    u.val[0][1] = 0.452297;
    u.val[0][2] = 0.053717;
    u.val[0][3] = 0.006469;
    u.val[0][4] = -0.010221;
    u.val[0][5] = 0.002928;
    u.val[0][6] = -0.173080;
    u.val[0][7] = 0.028183;
    u.val[0][8] = 0.370516;
    u.val[0][9] = 0.033125;
    u.val[0][10] = 0.164935;
    u.val[0][11] = 0.086431;
    u.val[1][0] = -0.439012;
    u.val[1][1] = 0.788381;
    u.val[1][2] = -0.053660;
    u.val[1][3] = -0.004512;
    u.val[1][4] = 0.003649;
    u.val[1][5] = -0.000539;
    u.val[1][6] = -0.067308;
    u.val[1][7] = 0.408885;
    u.val[1][8] = -0.093302;
    u.val[1][9] = -0.022238;
    u.val[1][10] = -0.038789;
    u.val[1][11] = 0.019733;
    u.val[2][0] = -0.219183;
    u.val[2][1] = 0.005437;
    u.val[2][2] = 0.339823;
    u.val[2][3] = 0.017070;
    u.val[2][4] = -0.023522;
    u.val[2][5] = 0.019255;
    u.val[2][6] = 0.492334;
    u.val[2][7] = 0.059537;
    u.val[2][8] = 0.583452;
    u.val[2][9] = -0.028508;
    u.val[2][10] = 0.300949;
    u.val[2][11] = -0.396859;
    u.val[3][0] = -0.211735;
    u.val[3][1] = -0.117723;
    u.val[3][2] = 0.566538;
    u.val[3][3] = 0.372001;
    u.val[3][4] = -0.002772;
    u.val[3][5] = -0.257887;
    u.val[3][6] = -0.603957;
    u.val[3][7] = 0.018193;
    u.val[3][8] = 0.144739;
    u.val[3][9] = 0.106482;
    u.val[3][10] = 0.069220;
    u.val[3][11] = 0.115205;
    u.val[4][0] = 0.117703;
    u.val[4][1] = -0.207718;
    u.val[4][2] = -0.003524;
    u.val[4][3] = -0.063143;
    u.val[4][4] = 0.791919;
    u.val[4][5] = -0.168692;
    u.val[4][6] = 0.047841;
    u.val[4][7] = 0.527898;
    u.val[4][8] = 0.026780;
    u.val[4][9] = -0.002714;
    u.val[4][10] = -0.034121;
    u.val[4][11] = 0.023529;
    u.val[5][0] = 0.093372;
    u.val[5][1] = 0.014110;
    u.val[5][2] = 0.008422;
    u.val[5][3] = -0.030355;
    u.val[5][4] = 0.064734;
    u.val[5][5] = 0.016505;
    u.val[5][6] = -0.281868;
    u.val[5][7] = -0.013233;
    u.val[5][8] = -0.375094;
    u.val[5][9] = -0.479646;
    u.val[5][10] = 0.468878;
    u.val[5][11] = -0.561605;
    u.val[6][0] = -0.199361;
    u.val[6][1] = -0.119261;
    u.val[6][2] = 0.018427;
    u.val[6][3] = -0.676778;
    u.val[6][4] = 0.046362;
    u.val[6][5] = 0.457114;
    u.val[6][6] = -0.384715;
    u.val[6][7] = 0.017802;
    u.val[6][8] = 0.271103;
    u.val[6][9] = 0.006767;
    u.val[6][10] = 0.166454;
    u.val[6][11] = 0.164283;
    u.val[7][0] = 0.115303;
    u.val[7][1] = -0.208037;
    u.val[7][2] = 0.027101;
    u.val[7][3] = -0.359687;
    u.val[7][4] = -0.550147;
    u.val[7][5] = -0.480610;
    u.val[7][6] = 0.021295;
    u.val[7][7] = 0.523430;
    u.val[7][8] = -0.046273;
    u.val[7][9] = -0.025442;
    u.val[7][10] = 0.044006;
    u.val[7][11] = 0.022607;
    u.val[8][0] = 0.045517;
    u.val[8][1] = -0.006541;
    u.val[8][2] = -0.049386;
    u.val[8][3] = -0.071580;
    u.val[8][4] = -0.036905;
    u.val[8][5] = 0.008117;
    u.val[8][6] = -0.255751;
    u.val[8][7] = 0.021772;
    u.val[8][8] = 0.265919;
    u.val[8][9] = -0.138744;
    u.val[8][10] = -0.736249;
    u.val[8][11] = -0.539432;
    u.val[9][0] = -0.187540;
    u.val[9][1] = -0.121629;
    u.val[9][2] = -0.698967;
    u.val[9][3] = 0.283171;
    u.val[9][4] = -0.036222;
    u.val[9][5] = -0.199778;
    u.val[9][6] = -0.196168;
    u.val[9][7] = 0.016937;
    u.val[9][8] = 0.445144;
    u.val[9][9] = -0.223652;
    u.val[9][10] = 0.209343;
    u.val[9][11] = 0.096467;
    u.val[10][0] = 0.113788;
    u.val[10][1] = -0.208264;
    u.val[10][2] = 0.015952;
    u.val[10][3] = 0.427576;
    u.val[10][4] = -0.245630;
    u.val[10][5] = 0.651035;
    u.val[10][6] = 0.001271;
    u.val[10][7] = 0.518912;
    u.val[10][8] = -0.033577;
    u.val[10][9] = -0.067767;
    u.val[10][10] = -0.021513;
    u.val[10][11] = 0.031112;
    u.val[11][0] = 0.003139;
    u.val[11][1] = -0.030601;
    u.val[11][2] = -0.255704;
    u.val[11][3] = 0.028626;
    u.val[11][4] = -0.013245;
    u.val[11][5] = 0.024954;
    u.val[11][6] = -0.155777;
    u.val[11][7] = 0.070463;
    u.val[11][8] = -0.074856;
    u.val[11][9] = 0.825612;
    u.val[11][10] = 0.197297;
    u.val[11][11] = -0.420305;
    /*********************/

    Matrix ccs = Matrix(4, 3);
    pcs = Matrix(number_of_correspondences, 3);

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 3; k++)
                ccs.val[j][k] += betas[i] * u.val[3 * j + k][11 - i];
    }

    for (int i = 0; i < number_of_correspondences; i++)
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

        for (int i = 0; i < number_of_correspondences; i++)
        {
            pcs.val[i][0] = -pcs.val[i][0];
            pcs.val[i][1] = -pcs.val[i][1];
            pcs.val[i][2] = -pcs.val[i][2];
        }
    }

    // estimate_R_and_t(R, t);
    double pc0[3] = {}, pw0[3] = {};

    pc0[0] = pc0[1] = pc0[2] = 0.0;
    pw0[0] = pw0[1] = pw0[2] = 0.0;

    for (int i = 0; i < number_of_correspondences; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            pc0[j] += pcs.val[i][j];
            pw0[j] += pws.val[i][j];
        }
    }

    for (int j = 0; j < 3; j++)
    {
        pc0[j] /= number_of_correspondences;
        pw0[j] /= number_of_correspondences;
    }

    Matrix ABt = Matrix(3, 3);
    Matrix ABt_D = Matrix(3, 1);
    Matrix ABt_U = Matrix(3, 3);
    Matrix ABt_V = Matrix(3, 3);

    for (int i = 0; i < number_of_correspondences; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            ABt.val[j][0] += (pcs.val[i][j] - pc0[j]) * (pws.val[i][0] - pw0[0]);
            ABt.val[j][1] += (pcs.val[i][j] - pc0[j]) * (pws.val[i][1] - pw0[1]);
            ABt.val[j][2] += (pcs.val[i][j] - pc0[j]) * (pws.val[i][2] - pw0[2]);
        }
    }

    ABt.svd(ABt_U, ABt_D, ABt_V);

    cout << ABt_U << endl;
    cout << ABt_V << endl;
    R = Matrix(3, 3);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            R.val[i][j] = Matrix::dot(ABt_U.val[i], ABt_U.val[j]);

    const double det =
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

    // cvSVD(&ABt, &ABt_D, &ABt_U, &ABt_V, CV_SVD_MODIFY_A);

    /*for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            R[i][j] = dot(abt_u + 3 * i, abt_v + 3 * j);

    const double det =
        R[0][0] * R[1][1] * R[2][2] + R[0][1] * R[1][2] * R[2][0] + R[0][2] * R[1][0] * R[2][1] -
        R[0][2] * R[1][1] * R[2][0] - R[0][1] * R[1][0] * R[2][2] - R[0][0] * R[1][2] * R[2][1];

    if (det < 0)
    {
        R[2][0] = -R[2][0];
        R[2][1] = -R[2][1];
        R[2][2] = -R[2][2];
    }

    t[0] = pc0[0] - dot(R[0], pw0);
    t[1] = pc0[1] - dot(R[1], pw0);
    t[2] = pc0[2] - dot(R[2], pw0);

    double sum2 = 0.0;

    for (int i = 0; i < number_of_correspondences; i++)
    {
        double *pw = &pws[3 * i];
        double Xc = dot(R[0], pw) + t[0];
        double Yc = dot(R[1], pw) + t[1];
        double inv_Zc = 1.0 / (dot(R[2], pw) + t[2]);
        double ue = uc + fu * Xc * inv_Zc;
        double ve = vc + fv * Yc * inv_Zc;
        double u = us[2 * i], v = us[2 * i + 1];

        sum2 += sqrt((u - ue) * (u - ue) + (v - ve) * (v - ve));
    }

    return sum2 / number_of_correspondences;*/
    return 0;
}

void EPnP::gauss_newton(const Matrix &L_6x10, const Matrix &Rho, FLOAT betas[4])
{
    const int32_t iterations_number = 5;

    Matrix A = Matrix(6, 4);
    Matrix B = Matrix(6, 1);
    Matrix X = Matrix(4, 1);

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

        Matrix::solve(A, X, B);

        for (int32_t i = 0; i < 4; i++)
            betas[i] += X.val[i][0];
    }
}