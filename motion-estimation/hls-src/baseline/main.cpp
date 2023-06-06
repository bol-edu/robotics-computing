#include <iostream>
#include "EstimateMotion.h"

using namespace std;

int main(int argc, char *argv[])
{

    cout << ">> Start test!" << endl;

    int32_t match_val[2 * MAX_KEYPOINT_NUM];
    FILE *fpmatch;
    fpmatch = fopen("testdata/match.txt", "r");
    unsigned int match_m;
    fscanf(fpmatch, "%d", &match_m);
    for (int i = 0; i < match_m; i++)
    {
        int m, n;
        fscanf(fpmatch, "%d %d", &m, &n);
        match_val[2 * i] = m;
        match_val[2 * i + 1] = n;
    }
    fclose(fpmatch);

    FLOAT kp0_val[2 * MAX_KEYPOINT_NUM];
    FILE *fpkp0;
    fpkp0 = fopen("testdata/kp0.txt", "r");
    unsigned int kp0_m;
    fscanf(fpkp0, "%d", &kp0_m);
    for (int i = 0; i < kp0_m; i++)
    {
        FLOAT m, n;
        fscanf(fpkp0, "%f %f", &m, &n);
        kp0_val[2 * i] = m;
        kp0_val[2 * i + 1] = n;
    }
    fclose(fpkp0);

    FLOAT kp1_val[2 * MAX_KEYPOINT_NUM];
    FILE *fpkp1;
    fpkp1 = fopen("testdata/kp1.txt", "r");
    unsigned int kp1_m;
    fscanf(fpkp1, "%d", &kp1_m);
    for (int i = 0; i < kp1_m; i++)
    {
        FLOAT m, n;
        fscanf(fpkp1, "%f %f", &m, &n);
        kp1_val[2 * i] = m;
        kp1_val[2 * i + 1] = n;
    }
    fclose(fpkp1);

    FLOAT k_val[9];
    FILE *fpk;
    fpk = fopen("testdata/k.txt", "r");
    for (int i = 0; i < 9; i++)
    {
        FLOAT m;
        fscanf(fpk, "%f", &m);
        k_val[i] = m;
    }
    fclose(fpk);

    FLOAT depth_val[IMAGE_HEIGTH * IMAGE_WIDTH];
    FILE *fpdepth;
    fpdepth = fopen("testdata/depth.txt", "r");
    int row, col;
    fscanf(fpdepth, "%d %d", &row, &col);
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            FLOAT m;
            fscanf(fpdepth, "%f", &m);
            depth_val[col * i + j] = m;
        }
    }
    fclose(fpdepth);

    FLOAT rmat[9], tvec[3];
    estimate_motion(match_val, match_m, kp0_val, kp1_val, k_val[0], k_val[4], k_val[2], k_val[5], depth_val, rmat, tvec);

    cout << "rmat" << endl;
    for (int i = 0; i < 3; i++)
    {
        cout << i << " ";
        for (int j = 0; j < 3; j++)
        {
            cout << rmat[3 * i + j] << " ";
        }
        cout << endl;
    }
    cout << endl;

    cout << "tvec" << endl;
    for (int i = 0; i < 3; i++)
    {
        cout << i << " " << tvec[i] << endl;
    }

    cout << ">> Test passed!" << endl;
    cout << "------------------------" << endl;
    return 0;
}
