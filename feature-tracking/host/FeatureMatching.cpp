#include "FeatureMatching.h"
#include <limits.h>

void match_feature(uchar *des0, uchar *des1, FLOAT dist_threshold, MATCH *match, int match_num)
{
#pragma HLS INTERFACE m_axi port = des0 depth = DES0_SIZE offset = slave
#pragma HLS INTERFACE m_axi port = des1 depth = DES1_SIZE offset = slave
#pragma HLS INTERFACE m_axi port = match depth = MAX_KEYPOINT_NUM offset = slave
#pragma HLS INTERFACE s_axilite port = des0
#pragma HLS INTERFACE s_axilite port = des1
#pragma HLS INTERFACE s_axilite port = match
#pragma HLS INTERFACE s_axilite port = dist_threshold
#pragma HLS INTERFACE s_axilite port = match_num
#pragma HLS INTERFACE s_axilite port = return

    DMatch matches[MAX_KEYPOINT_NUM][2];
    uchar queryDescriptors[MAX_KEYPOINT_NUM][DESCRIPTOR_COL];
    uchar trainDescriptors[MAX_KEYPOINT_NUM][DESCRIPTOR_COL];

    for (int i = 0; i < MAX_KEYPOINT_NUM; i++)
    {
        for (int j = 0; j < DESCRIPTOR_COL; j++)
        {
            queryDescriptors[i][j] = des0[i * DESCRIPTOR_COL + j];
        }
    }

    for (int i = 0; i < MAX_KEYPOINT_NUM; i++)
    {
        for (int j = 0; j < DESCRIPTOR_COL; j++)
        {
            trainDescriptors[i][j] = des1[i * DESCRIPTOR_COL + j];
        }
    }

    knnMatch(queryDescriptors, trainDescriptors, matches);

    match_num = 0;
    match[MAX_KEYPOINT_NUM];
    for (int i = 0; i < MAX_KEYPOINT_NUM; i++)
    {
        if (matches[i][0].distance <= dist_threshold * matches[i][1].distance)
        {
            match[i].a = matches[i][0].queryIndex;
            match[i].b = matches[i][0].trainIndex;
            match_num++;
        }
    }
    return;
}

void knnMatch(uchar queryDescriptors[MAX_KEYPOINT_NUM][DESCRIPTOR_COL],
              uchar trainDescriptors[MAX_KEYPOINT_NUM][DESCRIPTOR_COL],
              DMatch matches[MAX_KEYPOINT_NUM][2])
{
    const int IMGIDX_SHIFT = 18;
    const int IMGIDX_ONE = (1 << IMGIDX_SHIFT);

    int dist[MAX_KEYPOINT_NUM][2];
    int nidx[MAX_KEYPOINT_NUM][2];

    batchDistance(queryDescriptors, trainDescriptors, dist, nidx);

    for (int qIdx = 0; qIdx < MAX_KEYPOINT_NUM; qIdx++)
    {
        for (int k = 0; k < K; k++)
        {
            if (nidx[qIdx][k] < 0)
                break;
            matches[qIdx][k].queryIndex = qIdx;
            matches[qIdx][k].trainIndex = nidx[qIdx][k];
            matches[qIdx][k].distance = dist[qIdx][k];
        }
    }
}

void batchDistance(uchar src1[MAX_KEYPOINT_NUM][DESCRIPTOR_COL],
                   uchar src2[MAX_KEYPOINT_NUM][DESCRIPTOR_COL],
                   int dist[MAX_KEYPOINT_NUM][2],
                   int nidx[MAX_KEYPOINT_NUM][2])
{
    for (int i = 0; i < MAX_KEYPOINT_NUM; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            dist[i][j] = INT_MAX;
            nidx[i][j] = -1;
        }
    }

    int buf[MAX_KEYPOINT_NUM];

    for (int i = 0; i < MAX_KEYPOINT_NUM; i++)
    {
        batchDistHamming2(src1[i], src2, buf);

        for (int j = 0; j < MAX_KEYPOINT_NUM; j++)
        {
            int d = buf[j];
            if (d < dist[i][K - 1])
            {
                int k;
                for (k = K - 2; k >= 0 && dist[i][k] > d; k--)
                {
                    nidx[i][k + 1] = nidx[i][k];
                    dist[i][k + 1] = dist[i][k];
                }
                nidx[i][k + 1] = j;
                dist[i][k + 1] = d;
            }
        }
    }
}

void batchDistHamming2(const uchar src1[DESCRIPTOR_COL],
                       const uchar src2[MAX_KEYPOINT_NUM][DESCRIPTOR_COL],
                       int dist[MAX_KEYPOINT_NUM])
{
    int step = 0;
    for (int i = 0; i < MAX_KEYPOINT_NUM; i++)
    {
        dist[i] = normHamming(src1, src2[i]);
    }
}

int normHamming(const uchar a[DESCRIPTOR_COL], const uchar b[DESCRIPTOR_COL])
{
    const uchar *tab = popCountTable2;
    int result = 0;
    for (int i = 0; i < DESCRIPTOR_COL; i++)
    {
        result += tab[a[i] ^ b[i]];
    }
    return result;
}
