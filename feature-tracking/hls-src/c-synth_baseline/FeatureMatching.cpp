#include "FeatureMatching.h"
#include <limits.h>

void match_feature(uchar *des0, uchar *des1, FLOAT dist_threshold, int32 *match)
{
#pragma HLS INTERFACE m_axi port = des0 depth = DES0_SIZE offset = slave bundle = gmem
#pragma HLS INTERFACE m_axi port = des1 depth = DES0_SIZE offset = slave bundle = gmem
#pragma HLS INTERFACE m_axi port = match depth = MATCH_SIZE offset = slave bundle = gmem
#pragma HLS INTERFACE s_axilite port = des0 bundle = control
#pragma HLS INTERFACE s_axilite port = des1 bundle = control
#pragma HLS INTERFACE s_axilite port = match bundle = control
#pragma HLS INTERFACE s_axilite port = dist_threshold bundle = control
#pragma HLS INTERFACE s_axilite port = return bundle = control

    DMatch matches[MAX_KEYPOINT_NUM][2];
    uchar queryDescriptors[MAX_KEYPOINT_NUM][DESCRIPTOR_COL];
    uchar trainDescriptors[MAX_KEYPOINT_NUM][DESCRIPTOR_COL];
    for (int i = 0; i < MAX_KEYPOINT_NUM; i++)
    {
        for (int j = 0; j < DESCRIPTOR_COL; j++)
        {
            queryDescriptors[i][j] = des0[i * DESCRIPTOR_COL + j];
            trainDescriptors[i][j] = des1[i * DESCRIPTOR_COL + j];
        }
    }

    knnMatch(queryDescriptors, trainDescriptors, matches);

    // match_size = 0;
    match[2 * MAX_KEYPOINT_NUM];
    for (int i = 0; i < MAX_KEYPOINT_NUM; i++)
    {
        if (matches[i][0].distance <= dist_threshold * matches[i][1].distance)
        {
            match[2 * i] = matches[i][0].queryIndex;
            match[2 * i + 1] = matches[i][0].trainIndex;
            // match_size++;
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

    int32 dist[MAX_KEYPOINT_NUM][2];
    int32 nidx[MAX_KEYPOINT_NUM][2];

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
                   int32 dist[MAX_KEYPOINT_NUM][2],
                   int32 nidx[MAX_KEYPOINT_NUM][2])
{
    for (int i = 0; i < MAX_KEYPOINT_NUM; i++)
        for (int j = 0; j < 2; j++)
        {
            dist[i][j] = INT_MAX;
            nidx[i][j] = -1;
        }

    int32 buf[MAX_KEYPOINT_NUM];

    for (int i = 0; i < MAX_KEYPOINT_NUM; i++)
    {
        uchar a[32];
        for (int j = 0; j < 32; j++)
        {
            a[j] = src1[(i * 32 + j) / DESCRIPTOR_COL][(i * 32 + j) % DESCRIPTOR_COL];
        }
        batchDistHamming2(a, src2, buf);

        // since positive float's can be compared just like int's,
        // we handle both CV_32S and CV_32F cases with a single branch
        int j, k;

        for (j = 0; j < MAX_KEYPOINT_NUM; j++)
        {
            int d = buf[j];
            if (d < dist[i][K - 1])
            {
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
    uchar b[32];
    int step = 0;
    for (int i = 0; i < MAX_KEYPOINT_NUM; i++)
    {
        for (int j = 0; j < 32; j++)
        {
            b[j] = src2[(i * 32 + j) / DESCRIPTOR_COL][(i * 32 + j) % DESCRIPTOR_COL];
        }
        dist[i] = normHamming(src1, b);
    }
    // system("pause");
}

int normHamming(const uchar a[DESCRIPTOR_COL], const uchar b[32])
{
    const uchar *tab = popCountTable2;
    int result = 0;
    for (int i = 0; i < 32; i++)
    {
        result += tab[a[i] ^ b[i]];
        // printf("i = %d, a[i] = %d, b[i] = %d\n", i, a[i], b[i]);
    }
    return result;
}