#include <iostream>
#include "FeatureMatching.h"

using namespace std;

const float dist_threshold = 0.7;

void getData(uchar des0[MAX_KEYPOINT_NUM * DESCRIPTOR_COL],
             uchar des1[MAX_KEYPOINT_NUM * DESCRIPTOR_COL],
             int match_ans[2 * MAX_KEYPOINT_NUM])
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
        int m;
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
    int match[2 * MAX_KEYPOINT_NUM];
    int match_ans[2 * MAX_KEYPOINT_NUM];
    int match_num[1];

    getData(des0, des1, match_ans);

    match_feature(des0, des1, dist_threshold, match, match_num);

    printf("%d\n", match_num[0]);

    bool pass = true;
    for (int i = 0; i < MAX_KEYPOINT_NUM; i++)
    {
        printf("%d, %d\n", match[2 * i], match[2 * i + 1]);
    }

    if (pass)
        cout << ">> Test passed!" << endl;
    else
        cout << ">> Test failed!" << endl;
    cout << "------------------------" << endl;
    return 0;
}
