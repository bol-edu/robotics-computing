#include <iostream>
#include "helper.h"
#include "EstimateMotion.h"

using namespace std;

#define num_frames 130

void read_data(MATCH match[MAX_KEYPOINT_NUM], int &match_num,
               IPOINT kp0[MAX_KEYPOINT_NUM], IPOINT kp1[MAX_KEYPOINT_NUM],
               FLOAT &fx, FLOAT &fy, FLOAT &cx, FLOAT &cy, FLOAT depth[IMAGE_HEIGTH * IMAGE_WIDTH], int idx)
{
    char match_name[50];
    char kp0_name[50];
    char kp1_name[50];
    char depth_name[50];

    sprintf(match_name, "testdata/match_%d.txt", idx);
    sprintf(kp0_name, "testdata/KP0_iteration%d.txt", idx);
    sprintf(kp1_name, "testdata/KP1_iteration%d.txt", idx);
    sprintf(depth_name, "testdata/Depth_iteration%d.txt", idx);

    FILE *fp;

    cout << "\t--- Reading Matched Index" << endl;
    fp = fopen(match_name, "r");
    fscanf(fp, "%d", &match_num);
    for (int i = 0; i < match_num; i++)
    {
        int m, n;
        fscanf(fp, "%d %d", &m, &n);
        match[i].a = m;
        match[i].b = n;
    }
    fclose(fp);

    cout << "\t--- Reading Keypoint0" << endl;
    fp = fopen(kp0_name, "r");
    for (int i = 0; i < MAX_KEYPOINT_NUM; i++)
    {
        FLOAT m, n;
        fscanf(fp, "%f %f", &m, &n);
        kp0[i].x = m;
        kp0[i].y = n;
    }
    fclose(fp);

    cout << "\t--- Reading Keypoint1" << endl;
    fp = fopen(kp1_name, "r");
    for (int i = 0; i < MAX_KEYPOINT_NUM; i++)
    {
        FLOAT m, n;
        fscanf(fp, "%f %f", &m, &n);
        kp1[i].x = m;
        kp1[i].y = n;
    }
    fclose(fp);

    cout << "\t--- Reading Calibration Matix" << endl;
    fp = fopen("testdata/k.txt", "r");
    fscanf(fp, "%f %f %f %f", &fx, &cx, &fy, &cy);
    fclose(fp);

    cout << "\t--- Reading Depth Map" << endl;
    fp = fopen(depth_name, "r");
    for (int i = 0; i < IMAGE_HEIGTH; i++)
    {
        for (int j = 0; j < IMAGE_WIDTH; j++)
        {
            FLOAT m;
            fscanf(fp, "%f", &m);
            depth[IMAGE_WIDTH * i + j] = m;
        }
    }
    fclose(fp);
}

void display_output(FLOAT rmat[9], FLOAT tvec[3])
{
    cout << "rmat" << endl;
    for (int j = 0; j < 3; j++)
    {
        for (int k = 0; k < 3; k++)
        {
            cout << rmat[j * 3 + k] << " ";
        }
        cout << endl;
    }
    cout << endl;

    cout << "tvec" << endl;
    for (int j = 0; j < 3; j++)
    {
        cout << tvec[j] << " ";
    }
    cout << endl
         << endl;
}

int main(int argc, char *argv[])
{
    MATCH match[MAX_KEYPOINT_NUM];
    int match_num;
    IPOINT kp0[MAX_KEYPOINT_NUM];
    IPOINT kp1[MAX_KEYPOINT_NUM];
    FLOAT fx, fy, cx, cy;
    FLOAT depth[IMAGE_HEIGTH * IMAGE_WIDTH];
    FLOAT rmat[9], tvec[3];
    int threshold = 9;
    FLOAT confidence = 0.9999;
    int maxiter = 1000;

    int start = 0;
    for (int i = start; i < 10; i++)
    {
        cout << "=============== Iteration " << i << " ===============" << endl;
        cout << "================= Read Data =================" << endl;
        read_data(match, match_num, kp0, kp1, fx, fy, cx, cy, depth, i);
        bool take_last = (start != i);

        cout << "============== Starting Kernel ==============" << endl;
        estimate_motion(match, match_num, kp0, kp1, fx, fy, cx, cy, depth, threshold, confidence, maxiter, rmat, tvec, take_last);

        cout << "============ Output rmat & tvec =============" << endl;
        display_output(rmat, tvec);

        char out_name[50];
        sprintf(out_name, "output/out_%d.txt", i);
        cout << out_name << endl;
        FILE *fp;
        fp = fopen(out_name, "w");
        for (int ii = 0; ii < 9; ii++)
            fprintf(fp, "%f\n", rmat[ii]);
        for (int ii = 0; ii < 3; ii++)
            fprintf(fp, "%f\n", tvec[ii]);
        fclose(fp);
    }

    cout << "================= End test ==================" << endl;
    return 0;
}
