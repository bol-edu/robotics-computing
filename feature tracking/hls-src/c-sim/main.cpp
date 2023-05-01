#include <iostream>
#include "FeatureMatching.h"

using namespace std;

const FLOAT dist_threshold = 1;

void getData(uchar des0[MAX_KEYPOINT_NUM * DESCRIPTOR_COL],
             uchar des1[MAX_KEYPOINT_NUM * DESCRIPTOR_COL],
             int32 match_ans[2 * MAX_KEYPOINT_NUM])
{
    FILE *fp;
    fp = fopen("testdata/des0.txt", "r");
    for (int i = 0; i < MAX_KEYPOINT_NUM * DESCRIPTOR_COL; i++)
    {
        uchar m;
        fscanf(fp, "%hhu", &m);
        des0[i] = m;
    }
    fclose(fp);

    fp = fopen("testdata/des1.txt", "r");
    for (int i = 0; i < MAX_KEYPOINT_NUM * DESCRIPTOR_COL; i++)
    {
        uchar m;
        fscanf(fp, "%hhu", &m);
        des1[i] = m;
    }
    fclose(fp);

    fp = fopen("testdata/match_ans.txt", "r");
    for (int i = 0; i < 2 * MAX_KEYPOINT_NUM; i++)
    {
        int32 m;
        fscanf(fp, "%d", &m);
        match_ans[i] = m;
    }
    fclose(fp);
}

int main(int argc, char *argv[])
{

    cout << ">> Start test!" << endl;

    uchar des0[MAX_KEYPOINT_NUM * DESCRIPTOR_COL];
    uchar des1[MAX_KEYPOINT_NUM * DESCRIPTOR_COL];
    int32 match[2 * MAX_KEYPOINT_NUM];
    int32 match_ans[2 * MAX_KEYPOINT_NUM];
    //int32 match_size;

    getData(des0, des1, match_ans);

    match_feature(des0, des1, dist_threshold, match);//, match_size);

    bool pass = true;
    for (int i = 0; i < MAX_KEYPOINT_NUM; i++)
    {
        // printf("%d, %d\n", match[2 * i], match[2 * i + 1]);
        if (match[i] != match_ans[i])
        {
            pass = false;
            break;
        }
    }

    if (pass)
        cout << ">> Test passed!" << endl;
    else
        cout << ">> Test failed!" << endl;
    cout << "------------------------" << endl;
    return 0;
}
